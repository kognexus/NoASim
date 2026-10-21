//! WebAssembly wrapper that runs exactly one Nexus node per module instance.
//!
//! The harness instantiates this module once per joint. Every instance has its own
//! linear memory, so a node cannot see another node's state or the plant: the only way
//! in is the [`Io`] block. Before each [`nexus_tick`] the harness writes the node's own
//! sensor frame, the bus cycle and the frames the bus delivered; afterwards it reads the
//! torque and the frames the node sent.
//!
//! The ABI is plain C rather than `wasm-bindgen`, because `wasm-bindgen` glue keeps a
//! single instance per JavaScript module, which would put all joints in one memory
//! (ADR-0002). The harness learns the block's layout from [`nexus_layout`] instead of
//! hard-coding offsets.
//!
//! This crate is the only place in the workspace that uses `unsafe`: it owns the one
//! global node of its instance and hands the harness a pointer to its I/O block.

#![cfg_attr(target_arch = "wasm32", no_std)]
#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::mem::{offset_of, size_of};
use nexus_node::{
    default_actuator, DefaultActuator, Inbox, ModelError, MotorBalance, NodeConfig, Outbox, SensorFrame,
    MODEL_WORDS,
};

/// Capacity of the inbox, bytes.
pub const INBOX_BYTES: usize = 4096;
/// Capacity of the outbox, bytes.
pub const OUTBOX_BYTES: usize = 1024;

/// Shared memory between the harness and the node.
#[repr(C)]
pub struct Io {
    /// Written once by the harness before [`nexus_init`].
    pub config: NodeConfig,
    /// The node's own sensor readings, written before every tick.
    pub sensors: SensorFrame,
    /// Bus cycle (distributed clock).
    pub cycle: u32,
    /// Bytes of `inbox` in use.
    pub inbox_len: u32,
    /// Bytes of `outbox` in use, written by the node.
    pub outbox_len: u32,
    /// Motor torque, output-referred N·m, written by the node.
    pub torque: f32,
    /// 1 to engage the holding brake, written by the node.
    pub brake: u32,
    /// 1 to pull the hardwired safe-torque-off line, written by the node.
    pub safe_torque_off: u32,
    /// Frames delivered by the bus: `[len: u16][frame]` records.
    pub inbox: [u8; INBOX_BYTES],
    /// Frames sent by the node, same record format.
    pub outbox: [u8; OUTBOX_BYTES],
    /// Words of `model` in use, written by the harness before [`nexus_load_model`].
    pub model_len: u32,
    /// The joint's frozen learned model (B11), as exported by `learning/`.
    pub model: [f32; MODEL_WORDS],
}

/// Fields whose offsets [`nexus_layout`] reports, in this order.
const LAYOUT: [usize; 18] = [
    offset_of!(Io, config),
    offset_of!(Io, sensors),
    offset_of!(Io, cycle),
    offset_of!(Io, inbox_len),
    offset_of!(Io, outbox_len),
    offset_of!(Io, torque),
    offset_of!(Io, inbox),
    offset_of!(Io, outbox),
    INBOX_BYTES,
    OUTBOX_BYTES,
    size_of::<Io>(),
    size_of::<NodeConfig>(),
    size_of::<SensorFrame>(),
    offset_of!(Io, brake),
    offset_of!(Io, safe_torque_off),
    offset_of!(Io, model_len),
    offset_of!(Io, model),
    MODEL_WORDS,
];

struct Instance {
    node: Option<DefaultActuator>,
    io: Io,
}

/// Holder for the instance's global state.
struct Global(UnsafeCell<Instance>);

// SAFETY: WebAssembly instances are single-threaded; the harness never calls into one
// instance from two threads, so there is no concurrent access.
unsafe impl Sync for Global {}

static INSTANCE: Global = Global(UnsafeCell::new(Instance {
    node: None,
    io: Io {
        config: NodeConfig::example(),
        sensors: SensorFrame {
            motor_angle: 0.0,
            output_angle: 0.0,
            hall_sector: 0.0,
            gyro: [0.0; 3],
            accel: [0.0; 3],
            current: 0.0,
            temperature: 0.0,
        },
        cycle: 0,
        inbox_len: 0,
        outbox_len: 0,
        torque: 0.0,
        brake: 0,
        safe_torque_off: 0,
        inbox: [0; INBOX_BYTES],
        outbox: [0; OUTBOX_BYTES],
        model_len: 0,
        model: [0.0; MODEL_WORDS],
    },
}));

fn instance() -> &'static mut Instance {
    // SAFETY: single-threaded (see `Global`), and no exported function holds the
    // reference across a call back into the harness.
    unsafe { &mut *INSTANCE.0.get() }
}

/// Address of the I/O block in this instance's linear memory.
#[unsafe(no_mangle)]
pub extern "C" fn nexus_io_ptr() -> *mut Io {
    &raw mut instance().io
}

/// Layout of the I/O block: offsets of config, sensors, cycle, `inbox_len`, `outbox_len`,
/// torque, inbox, outbox (fields 0–7), then inbox and outbox capacity, total size, config
/// size and sensor-frame size (8–12), then offsets of brake and safe-torque-off (13–14),
/// then the offsets of `model_len` and `model` and the model capacity in words (15–17).
/// Returns `u32::MAX` for an unknown field.
#[unsafe(no_mangle)]
pub extern "C" fn nexus_layout(field: u32) -> u32 {
    LAYOUT.get(field as usize).map_or(u32::MAX, |&v| u32::try_from(v).unwrap_or(u32::MAX))
}

/// Starts the node from `io.config` at bus line position `line_position`.
#[unsafe(no_mangle)]
pub extern "C" fn nexus_init(line_position: u32) {
    let instance = instance();
    let position = u8::try_from(line_position).unwrap_or(u8::MAX);
    instance.node = Some(default_actuator(instance.io.config, position));
}

/// Loads the learned model from `io.model[..io.model_len]` into the started node.
/// Returns 0 on success, 1 if the node is not started, or 2 + the reason it was refused
/// (too short, version, wrong joint, too many orders, invalid).
#[unsafe(no_mangle)]
pub extern "C" fn nexus_load_model() -> u32 {
    let Instance { node, io } = instance();
    let Some(node) = node.as_mut() else { return 1 };
    let len = (io.model_len as usize).min(MODEL_WORDS);
    match node.load_model(&io.model[..len]) {
        Ok(()) => 0,
        Err(reason) => {
            2 + match reason {
                ModelError::TooShort => 0,
                ModelError::Version => 1,
                ModelError::WrongJoint => 2,
                ModelError::TooManyOrders => 3,
                ModelError::Invalid => 4,
            }
        }
    }
}

/// What the loaded learned model leaves unexplained of one motor-side balance, N·m, or
/// NaN without a model. A diagnostic: tests use it to check the node's evaluation against
/// the trainer's. Datasheet inertia and stiffness are not used by the learned model.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn nexus_model_residual(
    angle: f32,
    velocity: f32,
    acceleration: f32,
    twist: f32,
    twist_rate: f32,
    applied: f32,
    temperature: f32,
) -> f32 {
    let Some(node) = instance().node.as_ref() else { return f32::NAN };
    let balance = MotorBalance {
        angle,
        velocity,
        acceleration,
        twist,
        twist_rate,
        applied,
        inertia: 0.0,
        stiffness: 0.0,
    };
    node.learned_residual(&balance, temperature).unwrap_or(f32::NAN)
}

/// Runs one bus cycle: reads sensors, cycle and inbox; writes torque and outbox.
#[unsafe(no_mangle)]
pub extern "C" fn nexus_tick() {
    let Instance { node, io } = instance();
    let Some(node) = node.as_mut() else { return };
    let inbox_len = (io.inbox_len as usize).min(INBOX_BYTES);
    let mut outbox = Outbox::new(&mut io.outbox);
    io.torque = node.tick(io.cycle, &io.sensors, &Inbox::new(&io.inbox[..inbox_len]), &mut outbox);
    io.outbox_len = u32::try_from(outbox.len()).unwrap_or(0);
    let outputs = node.safety_outputs();
    io.brake = u32::from(outputs.brake);
    io.safe_torque_off = u32::from(outputs.safe_torque_off);
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

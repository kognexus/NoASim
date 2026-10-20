//! WebAssembly wrapper that runs exactly one Nexus node per module instance.
//!
//! The harness instantiates this module once per joint. Every instance has its own
//! linear memory, so a node cannot see another node's state or the plant: the only
//! way in is the [`Io`] block, which the harness fills with the node's own sensor
//! frame before each [`nexus_tick`] and reads the command from afterwards.
//!
//! The ABI is plain C rather than `wasm-bindgen`, because `wasm-bindgen` glue keeps a
//! single instance per JavaScript module, which would put all joints in one memory.
//!
//! This crate is the only place in the workspace that uses `unsafe`: it owns the one
//! global node of its instance and hands the harness a pointer to its I/O block.

#![cfg_attr(target_arch = "wasm32", no_std)]
#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use nexus_node::{Actuator, Command, NodeId, SensorFrame};

/// Shared-memory block between the harness and the node.
///
/// Layout is `#[repr(C)]` and made of `f32` only, so the harness can view it as a
/// `Float32Array` at [`nexus_io_ptr`].
#[repr(C)]
#[derive(Default)]
pub struct Io {
    /// Written by the harness before each tick.
    pub sensors: SensorFrame,
    /// Written by the node during each tick.
    pub command: Command,
}

/// The single node of this instance plus its I/O block.
struct Instance {
    node: Option<Actuator>,
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
        sensors: SensorFrame {
            motor_angle: 0.0,
            motor_velocity: 0.0,
        },
        command: Command { torque: 0.0 },
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

/// Size of the I/O block in bytes, so the harness can check its view of the layout.
#[unsafe(no_mangle)]
pub extern "C" fn nexus_io_len() -> usize {
    core::mem::size_of::<Io>()
}

/// Creates the node with bus address `id` and a position-hold target (NX-01 placeholder).
#[unsafe(no_mangle)]
pub extern "C" fn nexus_init(id: u8, target: f32, stiffness: f32, damping: f32) {
    instance().node = Some(Actuator::new(NodeId(id), target, stiffness, damping));
}

/// Runs one bus cycle: reads `io.sensors`, writes `io.command`.
#[unsafe(no_mangle)]
pub extern "C" fn nexus_tick() {
    let instance = instance();
    if let Some(node) = instance.node.as_mut() {
        instance.io.command = node.tick(&instance.io.sensors);
    }
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

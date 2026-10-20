//! The node's own sensor readings for one bus cycle.

/// What the node's own sensors report each cycle.
///
/// `#[repr(C)]` and all `f32`, in this order, so the harness can write it directly into
/// the node's memory. Angles are output-referred radians.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorFrame {
    /// Motor-shaft encoder, referred to the output.
    pub motor_angle: f32,
    /// Output encoder.
    pub output_angle: f32,
    /// Hall commutation sector, 0…5.
    pub hall_sector: f32,
    /// Gyro, housing IMU frame, rad/s.
    pub gyro: [f32; 3],
    /// Accelerometer, housing IMU frame, m/s².
    pub accel: [f32; 3],
    /// Measured q-axis current, A.
    pub current: f32,
    /// Winding temperature, °C.
    pub temperature: f32,
}

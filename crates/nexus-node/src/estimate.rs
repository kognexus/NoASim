//! Estimation: from raw sensor readings to the state the controller works with.

use crate::config::NodeConfig;
use crate::filters::LowPass;
use crate::sensors::SensorFrame;

/// The node's best knowledge of its own joint for this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Estimate {
    /// Motor-side angle (output-referred), rad.
    pub motor_angle: f32,
    /// Motor-side velocity, rad/s.
    pub motor_velocity: f32,
    /// Output angle, rad.
    pub output_angle: f32,
    /// Output velocity, rad/s.
    pub output_velocity: f32,
    /// Gear twist (output minus motor side), rad.
    pub gear_twist: f32,
    /// Motor-side acceleration, low-pass filtered, rad/s².
    pub motor_acceleration: f32,
    /// Torque the gear transmits, from the twist and the nominal stiffness, N·m.
    pub gear_torque: f32,
    /// Motor torque from the measured current and the nominal torque constant, N·m.
    pub motor_torque: f32,
    /// Measured current, A.
    pub current: f32,
    /// Winding temperature, °C.
    pub temperature: f32,
    /// Gyro of the housing, rad/s.
    pub gyro: [f32; 3],
    /// Accelerometer of the housing, m/s².
    pub accel: [f32; 3],
}

/// Turns sensor readings into an [`Estimate`].
pub trait Estimator {
    /// Processes one cycle of readings.
    fn update(&mut self, sensors: &SensorFrame) -> Estimate;
}

/// Second-order tracking observer: follows an angle and estimates its velocity without the
/// noise of differentiating encoder steps.
#[derive(Clone, Copy, Debug)]
pub struct TrackingObserver {
    angle: f32,
    velocity: f32,
    k_angle: f32,
    k_velocity: f32,
    dt: f32,
    started: bool,
}

impl TrackingObserver {
    /// Observer with natural frequency `bandwidth_hz` (critically damped) at timestep `dt`.
    #[must_use]
    pub fn new(bandwidth_hz: f32, dt: f32) -> Self {
        let omega = 2.0 * core::f32::consts::PI * bandwidth_hz;
        Self {
            angle: 0.0,
            velocity: 0.0,
            k_angle: 2.0 * omega,
            k_velocity: omega * omega,
            dt,
            started: false,
        }
    }

    /// Feeds one measured angle; returns `(angle, velocity)`.
    pub fn update(&mut self, measured: f32) -> (f32, f32) {
        if !self.started {
            self.angle = measured;
            self.started = true;
        }
        let error = measured - self.angle;
        self.angle += (self.velocity + self.k_angle * error) * self.dt;
        self.velocity += self.k_velocity * error * self.dt;
        (self.angle, self.velocity)
    }
}

/// Default estimator: tracking observers on both encoders, a filtered motor acceleration,
/// and torque from current and from gear twist (no torque sensor needed).
#[derive(Clone, Copy, Debug)]
pub struct EncoderEstimator {
    motor: TrackingObserver,
    output: TrackingObserver,
    acceleration: LowPass,
    last_motor_velocity: Option<f32>,
    dt: f32,
    gear_stiffness: f32,
    torque_constant: f32,
}

/// Corner of the acceleration filter, Hz.
const ACCELERATION_FILTER_HZ: f32 = 40.0;

impl EncoderEstimator {
    /// Estimator using the observer bandwidth from `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        let dt = config.cycle_seconds();
        Self {
            motor: TrackingObserver::new(config.observer_hz, dt),
            output: TrackingObserver::new(config.observer_hz, dt),
            acceleration: LowPass::new(ACCELERATION_FILTER_HZ, dt),
            last_motor_velocity: None,
            dt,
            gear_stiffness: config.gear_stiffness,
            torque_constant: config.output_torque_constant,
        }
    }
}

impl Estimator for EncoderEstimator {
    fn update(&mut self, s: &SensorFrame) -> Estimate {
        let (_, motor_velocity) = self.motor.update(s.motor_angle);
        let (_, output_velocity) = self.output.update(s.output_angle);
        let raw_acceleration =
            (motor_velocity - self.last_motor_velocity.unwrap_or(motor_velocity)) / self.dt;
        self.last_motor_velocity = Some(motor_velocity);
        let gear_twist = s.output_angle - s.motor_angle;
        Estimate {
            motor_angle: s.motor_angle,
            motor_velocity,
            output_angle: s.output_angle,
            output_velocity,
            gear_twist,
            motor_acceleration: self.acceleration.update(raw_acceleration),
            gear_torque: self.gear_stiffness * gear_twist,
            motor_torque: self.torque_constant * s.current,
            current: s.current,
            temperature: s.temperature,
            gyro: s.gyro,
            accel: s.accel,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::TrackingObserver;

    #[test]
    fn observer_tracks_a_ramp_velocity() {
        let mut observer = TrackingObserver::new(100.0, 0.00025);
        let mut velocity = 0.0;
        for k in 0..4000_u16 {
            (_, velocity) = observer.update(2.0 * f32::from(k) * 0.00025);
        }
        assert!((velocity - 2.0).abs() < 1e-3, "velocity {velocity}");
    }
}

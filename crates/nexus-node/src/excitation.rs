//! Commissioning excitation (B11).
//!
//! To learn a model of itself, a joint must move through the speeds, accelerations and
//! loads it will meet. On the host's `Commission` request it follows a *multisine*
//! around the angle it stands at: a sum of sines over a band of frequencies, with
//! Schroeder phases (which keep the peak low for a given power) perturbed by the request's
//! seed, so that two runs give independent data. The amplitudes are scaled so the motion
//! stays inside the joint's position range and well under its speed limit. A smooth
//! envelope fades the motion in and out, so the reference never jumps.

use crate::reference::Reference;
use core::f32::consts::{PI, TAU};
use nexus_proto::{Commission, Limits};

/// Number of sine components.
const COMPONENTS: usize = 8;
/// Lowest and highest component frequency, Hz. The lowest gives near-steady speeds in
/// both directions (friction); the highest reaches the gear's elastic range.
const BAND_HZ: (f32, f32) = (0.2, 6.0);
/// Largest excursion from the start angle, rad.
const MAX_EXCURSION: f32 = 0.35;
/// Margin kept from the position limits, rad.
const LIMIT_MARGIN: f32 = 0.1;
/// Peak speed as a share of the speed limit.
const SPEED_SHARE: f32 = 0.5;
/// Duration of the fade in and out, s.
const FADE_SECONDS: f32 = 1.0;

/// A running excitation.
#[derive(Clone, Copy, Debug)]
pub struct Excitation {
    start_angle: f32,
    start_cycle: u32,
    duration: u32,
    dt: f32,
    amplitude: [f32; COMPONENTS],
    omega: [f32; COMPONENTS],
    phase: [f32; COMPONENTS],
}

impl Excitation {
    /// Excitation requested by `request` at bus cycle `now`, around `start_angle`, inside
    /// `limits`, for a node with timestep `dt`.
    #[must_use]
    pub fn new(start_angle: f32, now: u32, request: &Commission, limits: &Limits, dt: f32) -> Self {
        let mut omega = [0.0; COMPONENTS];
        let mut amplitude = [0.0; COMPONENTS];
        let mut phase = [0.0; COMPONENTS];
        let mut random = Lcg(request.seed ^ 0x9E37_79B9);
        let band_scale = if request.frequency_scale > 0.0 { request.frequency_scale } else { 1.0 };
        #[allow(clippy::cast_precision_loss)]
        for k in 0..COMPONENTS {
            let share = k as f32 / (COMPONENTS - 1) as f32;
            let hz = band_scale * BAND_HZ.0 * libm::powf(BAND_HZ.1 / BAND_HZ.0, share);
            omega[k] = TAU * hz;
            // Between equal position and equal velocity per component, so both slow
            // sweeps and quick reversals carry energy.
            amplitude[k] = 1.0 / libm::sqrtf(hz);
            let schroeder = -PI * (k * (k + 1)) as f32 / COMPONENTS as f32;
            phase[k] = schroeder + 0.5 * PI * (random.unit() - 0.5);
        }
        // Scale to the tighter of the excursion and the speed budget.
        let room = (start_angle - limits.min_position).min(limits.max_position - start_angle) - LIMIT_MARGIN;
        let excursion = MAX_EXCURSION.min(room.max(0.0));
        let peak_position: f32 = amplitude.iter().sum();
        // The fade adds its own rate times the excursion to the speed.
        let fade_rate = PI / (2.0 * FADE_SECONDS);
        let peak_speed: f32 =
            amplitude.iter().zip(&omega).map(|(a, w)| a * w).sum::<f32>() + fade_rate * peak_position;
        let scale = (excursion / peak_position).min(SPEED_SHARE * limits.max_velocity / peak_speed);
        for a in &mut amplitude {
            *a *= scale;
        }
        Self { start_angle, start_cycle: now, duration: request.duration_cycles, dt, amplitude, omega, phase }
    }

    /// The angle the excitation started from; the joint holds it afterwards.
    #[must_use]
    pub const fn start_angle(&self) -> f32 {
        self.start_angle
    }

    /// The reference at bus cycle `now`, or `None` once the excitation is over.
    #[must_use]
    pub fn reference(&self, now: u32) -> Option<Reference> {
        let elapsed = now.wrapping_sub(self.start_cycle);
        if elapsed >= self.duration {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let (t, total) = (elapsed as f32 * self.dt, self.duration as f32 * self.dt);
        // Multisine and its first two derivatives.
        let (mut s, mut ds, mut dds) = (0.0, 0.0, 0.0);
        for k in 0..COMPONENTS {
            let (a, w) = (self.amplitude[k], self.omega[k]);
            let (sin, cos) = libm::sincosf(w * t + self.phase[k]);
            s += a * sin;
            ds += a * w * cos;
            dds -= a * w * w * sin;
        }
        let (e, de, dde) = envelope(t, total);
        Some(Reference {
            position: self.start_angle + e * s,
            velocity: de * s + e * ds,
            acceleration: dde * s + 2.0 * de * ds + e * dds,
        })
    }
}

/// Smooth window: rises as sin² over the first `FADE_SECONDS`, falls the same way over the
/// last. Returns the value and its first two time derivatives.
fn envelope(t: f32, total: f32) -> (f32, f32, f32) {
    let fade = FADE_SECONDS.min(0.5 * total);
    let (x, direction) = if t < fade {
        (t, 1.0)
    } else if t > total - fade {
        (total - t, -1.0)
    } else {
        return (1.0, 0.0, 0.0);
    };
    let w = PI / (2.0 * fade);
    let (sin, cos) = libm::sincosf(w * x);
    // d/dx sin²(wx) = w sin(2wx); d²/dx² = 2w² cos(2wx).
    (sin * sin, direction * w * 2.0 * sin * cos, 2.0 * w * w * (cos * cos - sin * sin))
}

/// Small linear congruential generator: enough to perturb phases reproducibly.
struct Lcg(u32);

impl Lcg {
    /// Uniform in [0, 1).
    fn unit(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        #[allow(clippy::cast_precision_loss)]
        let value = (self.0 >> 8) as f32 / (1u32 << 24) as f32;
        value
    }
}

#[cfg(test)]
mod tests {
    use super::Excitation;
    use nexus_proto::{Commission, Limits};

    const DT: f32 = 0.00025;

    fn limits() -> Limits {
        Limits {
            max_torque: 90.0,
            max_velocity: 3.0,
            min_position: -3.0,
            max_position: 3.0,
            max_temperature: 120.0,
            network_torque_share: 0.25,
        }
    }

    fn run(excitation: &Excitation) -> (f32, f32, f32) {
        let (mut max_excursion, mut max_speed, mut max_jump) = (0.0_f32, 0.0_f32, 0.0_f32);
        let mut last = excitation.start_angle();
        let mut k = 0;
        while let Some(r) = excitation.reference(1000 + k) {
            max_excursion = max_excursion.max((r.position - excitation.start_angle()).abs());
            max_speed = max_speed.max(r.velocity.abs());
            max_jump = max_jump.max((r.position - last).abs());
            last = r.position;
            k += 1;
        }
        assert_eq!(k, 40_000);
        (max_excursion, max_speed, max_jump)
    }

    #[test]
    fn stays_inside_its_limits_and_starts_and_ends_at_rest() {
        let request = Commission { duration_cycles: 40_000, seed: 7, frequency_scale: 1.0 };
        let excitation = Excitation::new(0.5, 1000, &request, &limits(), DT);
        let (excursion, speed, jump) = run(&excitation);
        assert!(excursion <= 0.35 + 1e-4 && excursion > 0.1, "{excursion}");
        assert!(speed <= 1.5 + 1e-3, "{speed}");
        assert!(jump < 2.0 * 1.5 * DT, "reference jumps by {jump}");
        let first = excitation.reference(1000).unwrap();
        assert!((first.position - 0.5).abs() < 1e-6 && first.velocity.abs() < 1e-6);
        assert!(excitation.reference(41_000).is_none());
    }

    #[test]
    fn keeps_clear_of_a_near_position_limit() {
        let request = Commission { duration_cycles: 40_000, seed: 7, frequency_scale: 1.0 };
        let excitation = Excitation::new(2.8, 0, &request, &limits(), DT);
        let mut k = 0;
        while let Some(r) = excitation.reference(k) {
            assert!(r.position < 3.0 - 0.1 + 1e-4, "{}", r.position);
            k += 1;
        }
    }

    #[test]
    fn seeds_give_different_runs() {
        let a = Excitation::new(
            0.0,
            0,
            &Commission { duration_cycles: 8000, seed: 1, frequency_scale: 1.0 },
            &limits(),
            DT,
        );
        let b = Excitation::new(
            0.0,
            0,
            &Commission { duration_cycles: 8000, seed: 2, frequency_scale: 1.0 },
            &limits(),
            DT,
        );
        let differ = (0..8000)
            .any(|k| (a.reference(k).unwrap().position - b.reference(k).unwrap().position).abs() > 1e-3);
        assert!(differ);
    }
}

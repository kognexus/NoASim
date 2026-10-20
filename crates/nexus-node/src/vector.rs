//! Three-vector helpers for the node (`no_std`, `f32`).

/// A 3-vector.
pub type Vec3 = [f32; 3];

/// Dot product.
#[must_use]
pub fn dot(a: Vec3, b: Vec3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product.
#[must_use]
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// `a + b`.
#[must_use]
pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// `a · s`.
#[must_use]
pub fn scale(a: Vec3, s: f32) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// Euclidean length.
#[must_use]
pub fn norm(a: Vec3) -> f32 {
    libm::sqrtf(dot(a, a))
}

/// Rotates `v` by `angle` about the unit `axis` (Rodrigues).
#[must_use]
pub fn rotate(v: Vec3, axis: Vec3, angle: f32) -> Vec3 {
    let (s, c) = (libm::sinf(angle), libm::cosf(angle));
    let k_cross_v = cross(axis, v);
    let k_dot_v = dot(axis, v);
    add(add(scale(v, c), scale(k_cross_v, s)), scale(axis, k_dot_v * (1.0 - c)))
}

#[cfg(test)]
mod tests {
    use super::{dot, rotate};

    #[test]
    fn rotating_x_about_z_by_a_quarter_turn_gives_y() {
        let r = rotate([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], core::f32::consts::FRAC_PI_2);
        assert!((r[0]).abs() < 1e-6 && (r[1] - 1.0).abs() < 1e-6);
        assert!((dot(r, r) - 1.0).abs() < 1e-6);
    }
}

use glam::DVec3;

pub const G: f64 = 9.80665;
pub const GRAVITY: DVec3 = DVec3::new(0.0, -G, 0.0);
pub const DT: f64 = 1.0 / 1000.0; // 1000Hz
pub const EPSILON: f64 = 0.00001;

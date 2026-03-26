use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    constants::{DT, G},
    transitions::{FastTransitions, Forces, Transitions},
    TrackPoint, TrackSpline,
};

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct TrackConfig {
    parameter: f64,
    resistance: f64,
    heartline_height: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Track {
    pub sections: Vec<TrackSection>,
    pub config: TrackConfig,
    pub anchor: TrackPoint,
}

impl Track {
    pub fn get_spline(&self) -> (TrackSpline, Vec<f64>) {
        let splines = self.make_splines();

        // Compute section start distances in one pass.
        let mut len_accum = 0.0;
        let section_starts: Vec<f64> = splines
            .iter()
            .map(|s| {
                let start = len_accum;
                len_accum += s.total_distance();
                start
            })
            .collect();

        let points = splines
            .iter()
            .flat_map(|s| s.points.iter().cloned())
            .step_by(4)
            .collect();

        (TrackSpline { points }, section_starts)
    }

    pub fn make_splines(&self) -> Vec<TrackSpline> {
        let mut splines: Vec<TrackSpline> = Vec::with_capacity(self.sections.len());
        let mut current_point = TrackPoint {
            rot: DQuat::IDENTITY.into(),
            time: 0.0,
            ..self.anchor
        };
        let mut forces = Forces {
            vert: 1.0,
            lat: 0.0,
            roll: 0.0,
        };

        for section in &self.sections {
            let spline = self.make_spline(section, current_point, forces);

            // Carry the tail of this spline into the next section.
            if let Some(last) = spline.points.last() {
                current_point = *last;
            }
            forces = spline
                .forces(spline.total_distance() - 0.005)
                .unwrap_or(forces);

            splines.push(spline);
        }

        splines
    }

    fn make_spline(
        &self,
        section: &TrackSection,
        start: TrackPoint,
        start_forces: Forces,
    ) -> TrackSpline {
        match section {
            TrackSection::Straight {
                length,
                fixed_speed,
            } => {
                // Straight sections have constant orientation.
                self.make_arc_spline(start, *fixed_speed, *length, 0.01, |rot, _dp| rot)
            }
            TrackSection::Curved {
                fixed_speed,
                radius,
                direction,
                angle,
            } => {
                let arc_len = angle.to_radians() * radius;
                let dp = arc_len / 200.0;
                let rad_per_m = 1.0 / radius;
                let axis = DQuat::from_axis_angle(DVec3::Z, direction.to_radians()) * DVec3::NEG_X;

                self.make_arc_spline(start, *fixed_speed, arc_len, dp, move |rot, dp| {
                    rot * DQuat::from_axis_angle(axis, rad_per_m * dp)
                })
            }
            TrackSection::Force {
                fixed_speed,
                transitions,
            } => self.make_force_spline(start, *fixed_speed, transitions, start_forces),
        }
    }

    /// Integrates a section where the train moves along an arc of a given length.
    ///
    /// `rotate` receives the current orientation and the step length, and returns
    /// the new orientation after that step.  For straight sections pass `|rot, _| rot`;
    /// for curved sections pass the appropriate axis-angle increment.
    fn make_arc_spline(
        &self,
        start: TrackPoint,
        fixed_speed: Option<f64>,
        length: f64,
        dp: f64,
        rotate: impl Fn(DQuat, f64) -> DQuat,
    ) -> TrackSpline {
        let mut points = Vec::with_capacity((length / dp) as usize + 1);
        let mut pos = start.pos;
        let mut rot = start.rot.0;
        let mut velocity = start.velocity;
        let mut time = start.time;
        let mut p = 0.0_f64;

        while p < length {
            pos += rot * (DVec3::Z * dp);
            rot = rotate(rot, dp);

            if let Some(speed) = fixed_speed {
                velocity = speed;
            } else {
                let prev = points.last().copied().unwrap_or(start);
                let curr = TrackPoint {
                    pos,
                    rot: rot.into(),
                    velocity,
                    time,
                };
                velocity = self.compute_friction(&prev, &curr, dp / velocity);
                if velocity <= 0.0 {
                    return TrackSpline { points };
                }
            }

            time += dp / velocity;
            points.push(TrackPoint {
                pos,
                rot: rot.into(),
                velocity,
                time,
            });
            p += dp;
        }

        TrackSpline { points }
    }

    /// Integrates a section driven by an explicit force/transition profile using RK4.
    ///
    /// Rather than Euler-stepping `state += derivative * DT` (first-order, O(DT) error),
    /// RK4 samples the derivative at four points within each step and combines them with
    /// a weighted average (O(DT⁴) error).  This allows a 5–10× larger timestep for the
    /// same accuracy, and produces a correspondingly sparser output spline.
    fn make_force_spline(
        &self,
        start: TrackPoint,
        fixed_speed: Option<f64>,
        transitions: &Transitions,
        start_forces: Forces,
    ) -> TrackSpline {
        let transitions = FastTransitions::new(transitions);
        let mut points = Vec::with_capacity((transitions.length / DT) as usize);
        let mut state = IntegState {
            pos: start.pos,
            rot: start.rot.0,
            velocity: fixed_speed.unwrap_or(start.velocity),
        };
        let mut time = 0.0_f64;

        // Closure so the derivative call only needs `t` and `state`.
        let deriv = |t: f64, s: &IntegState| -> Option<Deriv> {
            eval_deriv(t, s, &transitions, start_forces, &self.config, fixed_speed)
        };

        while time < transitions.length {
            // RK4: four derivative evaluations per step.
            let Some(k1) = deriv(time, &state) else { break };
            let Some(k2) = deriv(time + DT * 0.5, &state.advance(&k1, DT * 0.5)) else {
                break;
            };
            let Some(k3) = deriv(time + DT * 0.5, &state.advance(&k2, DT * 0.5)) else {
                break;
            };
            let Some(k4) = deriv(time + DT, &state.advance(&k3, DT)) else {
                break;
            };

            // Weighted average of the four slopes.
            let combined = Deriv {
                dpos: (k1.dpos + k2.dpos * 2.0 + k3.dpos * 2.0 + k4.dpos) / 6.0,
                omega: (k1.omega + k2.omega * 2.0 + k3.omega * 2.0 + k4.omega) / 6.0,
                dvelocity: (k1.dvelocity + k2.dvelocity * 2.0 + k3.dvelocity * 2.0 + k4.dvelocity)
                    / 6.0,
            };

            state = state.advance(&combined, DT);

            if state.velocity <= 0.0 {
                return TrackSpline { points };
            }

            points.push(TrackPoint {
                pos: state.pos,
                rot: state.rot.into(),
                velocity: state.velocity,
                time: time + start.time,
            });
            time += DT;
        }

        TrackSpline { points }
    }

    /// Returns the speed after applying gravity and rolling resistance over one integration step.
    ///
    /// Returns `0.0` when the train has stalled (insufficient energy to continue).
    /// Used only by `make_arc_spline`; `make_force_spline` derives velocity analytically.
    fn compute_friction(&self, last: &TrackPoint, curr: &TrackPoint, dt: f64) -> f64 {
        let cfg = &self.config;
        let offset = DVec3::NEG_Y * cfg.heartline_height * 0.9;

        let last_pos = last.pos + last.rot.0 * offset;
        let curr_pos = curr.pos + curr.rot.0 * offset;
        let delta = curr_pos - last_pos;

        // Kinetic energy minus resistive losses minus potential + friction work.
        let energy = 0.5 * last.velocity * last.velocity
            - last.velocity.powi(3) * dt * cfg.resistance
            - (delta.y + delta.length() * cfg.parameter) * G;

        if energy <= 0.0 {
            0.0
        } else {
            (2.0 * energy).sqrt()
        }
    }
}

// ---------------------------------------------------------------------------
// RK4 helpers
// ---------------------------------------------------------------------------

/// Flat integration state: position, orientation, and speed.
#[derive(Clone, Copy)]
struct IntegState {
    pos: DVec3,
    rot: DQuat,
    velocity: f64,
}

/// Time-derivative of `IntegState`.
struct Deriv {
    /// dpos/dt  — velocity vector in world space.
    dpos: DVec3,
    /// Angular velocity vector; magnitude = rad/s, direction = axis of rotation.
    omega: DVec3,
    /// dv/dt    — scalar speed change rate.
    dvelocity: f64,
}

impl IntegState {
    /// Returns `self + deriv * dt`, re-normalizing the quaternion after rotation.
    fn advance(&self, d: &Deriv, dt: f64) -> Self {
        // Integrate rotation via the exponential map: exp(omega * dt/2) * q.
        // `from_scaled_axis` handles the zero-vector case safely.
        let rot = (DQuat::from_scaled_axis(d.omega * dt) * self.rot).normalize();
        Self {
            pos: self.pos + d.dpos * dt,
            rot,
            velocity: self.velocity + d.dvelocity * dt,
        }
    }
}

/// Computes the time-derivative of `IntegState` at a given instant.
///
/// Returns `None` when `t` is outside the transition table (signals end of section).
fn eval_deriv(
    t: f64,
    state: &IntegState,
    transitions: &FastTransitions,
    start_forces: Forces,
    cfg: &TrackConfig,
    fixed_speed: Option<f64>,
) -> Option<Deriv> {
    let forces = transitions.evaluate(t)? + start_forces;
    let rot = state.rot;
    let v = state.velocity;

    // --- Angular velocity ---
    //
    // Three independent contributions to the frame's rotation rate:
    //   1. Roll:    explicit roll rate from the transition, around the local Z axis.
    //   2. Normal:  centripetal curvature in the vertical plane, around local -X.
    //   3. Lateral: centripetal curvature in the horizontal plane, around local Y.
    let force_vec = (rot * DVec3::Y) * -forces.vert + (rot * DVec3::NEG_X) * -forces.lat + DVec3::Y;

    let normal_accel = -force_vec.dot(rot * DVec3::Y) * G;
    let lateral_accel = -force_vec.dot(rot * DVec3::NEG_X) * G;

    let roll_rate = if forces.roll.abs() > 0.01 {
        forces.roll.to_radians()
    } else {
        0.0
    };

    let omega = rot * DVec3::Z * roll_rate
        + rot * DVec3::NEG_X * (normal_accel / v)
        + rot * DVec3::Y * -(lateral_accel / v);

    // --- Speed derivative ---
    //
    // Derived analytically from energy conservation:
    //   dE/dt = -v³·resistance  −  (track_dir.y + parameter)·G·v
    //   E = ½v²  →  dv/dt = dE/dt / v
    let track_dir = rot * DVec3::Z;
    let dvelocity = if fixed_speed.is_some() {
        0.0
    } else {
        -v * v * cfg.resistance - (track_dir.y + cfg.parameter) * G
    };

    Some(Deriv {
        dpos: track_dir * v,
        omega,
        dvelocity,
    })
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all_fields = "camelCase")]
pub enum TrackSection {
    #[serde(rename = "straight")]
    Straight {
        length: f64,
        fixed_speed: Option<f64>,
    },

    #[serde(rename = "force")]
    Force {
        fixed_speed: Option<f64>,
        transitions: Transitions,
    },

    #[serde(rename = "curved")]
    Curved {
        fixed_speed: Option<f64>,
        radius: f64,
        direction: f64,
        angle: f64,
    },
}

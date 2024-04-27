use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    constants::{DT, G},
    transitions::{Forces, Transitions},
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
        let mut section_start = Vec::new();
        let mut len_accum = 0.0;
        for spline in &splines {
            section_start.push(len_accum);
            len_accum += spline.total_distance();
        }
        let points = splines
            .iter()
            .flat_map(|s| s.points.iter())
            .cloned()
            .step_by(4)
            .collect();
        let spline = TrackSpline { points };

        (spline, section_start)
    }

    pub fn make_splines(&self) -> Vec<TrackSpline> {
        let mut splines: Vec<TrackSpline> = Vec::with_capacity(self.sections.len());
        let mut initial_point = self.anchor;
        initial_point.rot = DQuat::IDENTITY.into();
        initial_point.time = 0.0;

        let mut forces = Forces {
            vert: 1.0,
            lat: 0.0,
            roll: 0.0,
        };

        for section in &self.sections {
            if splines.is_empty() {
                splines.push(self.make_spline(section, initial_point, forces));
            } else {
                let point = splines.last().unwrap().points.last().unwrap();
                splines.push(self.make_spline(section, *point, forces));
            }
            let spline = splines.last().unwrap();
            forces = spline.forces(spline.total_distance() - 0.005).unwrap();
        }

        splines
    }

    fn make_spline(
        &self,
        section: &TrackSection,
        start: TrackPoint,
        start_forces: Forces,
    ) -> TrackSpline {
        let mut spline = TrackSpline { points: Vec::new() };
        match section {
            TrackSection::Straight {
                length,
                fixed_speed,
            } => {
                let dp = 0.01;
                let mut pos = start.pos;
                let mut velocity = start.velocity;

                let mut p = 0.0;

                while p < *length {
                    let last_point = spline.points.last();
                    pos = start.rot.0 * (DVec3::Z * dp) + pos;
                    if let Some(fixed_speed) = fixed_speed {
                        velocity = *fixed_speed;
                    } else {
                        let dt = dp / velocity;
                        let point = TrackPoint {
                            pos,
                            rot: start.rot,
                            velocity,
                            time: p / velocity + start.time,
                        };
                        velocity = track_friction(
                            self.config.parameter,
                            self.config.resistance,
                            self.config.heartline_height,
                            last_point.unwrap_or(&point),
                            &point,
                            dt,
                        );
                        if velocity <= 0.0 {
                            return spline;
                        }
                    }
                    spline.points.push(TrackPoint {
                        pos,
                        rot: start.rot,
                        velocity,
                        time: p / velocity + start.time,
                    });

                    p += dp;
                }
            }
            TrackSection::Curved {
                fixed_speed,
                radius,
                direction,
                angle,
            } => {
                let mut pos = start.pos;
                let mut velocity = start.velocity;
                let mut rot = start.rot.0;

                let angle = angle.to_radians();
                let rad_per_m = 1.0 / radius;

                let dp = (angle * radius) / 200.0;

                let axis = DQuat::from_axis_angle(DVec3::Z, direction.to_radians()) * DVec3::NEG_X;

                let mut p = 0.0;

                while p < angle * radius {
                    let last_point = spline.points.last();

                    pos += rot * (dp * DVec3::Z);
                    if let Some(fixed_speed) = fixed_speed {
                        velocity = *fixed_speed;
                    } else {
                        let dt = dp / velocity;
                        let point = TrackPoint {
                            pos,
                            rot: start.rot,
                            velocity,
                            time: p / velocity + start.time,
                        };
                        velocity = track_friction(
                            self.config.parameter,
                            self.config.resistance,
                            self.config.heartline_height,
                            last_point.unwrap_or(&point),
                            &point,
                            dt,
                        );
                        if velocity <= 0.0 {
                            return spline;
                        }
                    }
                    rot *= DQuat::from_axis_angle(axis, rad_per_m * dp);
                    spline.points.push(TrackPoint {
                        pos,
                        rot: rot.into(),
                        velocity,
                        time: p / velocity + start.time,
                    });

                    p += dp;
                }
            }
            TrackSection::Force {
                fixed_speed,
                transitions,
            } => {
                let mut velocity = fixed_speed.unwrap_or(start.velocity);
                let mut pos = start.pos;
                let mut rot = start.rot.0;
                let mut time = 0.0;
                let transitions_length = transitions.length();
                while time < transitions.length() {
                    let delta_distance = velocity * DT;
                    if time >= transitions_length {
                        break;
                    }
                    if let Some(forces) = transitions.evaluate(time) {
                        let forces = forces + start_forces;
                        let mut next_rot = rot;

                        if forces.roll.abs() > 0.01 {
                            next_rot = DQuat::from_axis_angle(
                                next_rot * DVec3::Z,
                                forces.roll.to_radians() * DT,
                            ) * next_rot;
                        }

                        let force_vec = ((next_rot * DVec3::Y) * -forces.vert)
                            + ((next_rot * DVec3::NEG_X) * -forces.lat)
                            + DVec3::Y;

                        let normal_force = -force_vec.dot(next_rot * DVec3::Y) * G;
                        let lateral_force = -force_vec.dot(next_rot * DVec3::NEG_X) * G;

                        next_rot = (DQuat::from_axis_angle(
                            next_rot * DVec3::NEG_X,
                            (normal_force / velocity) * DT,
                        ) * DQuat::from_axis_angle(
                            next_rot * DVec3::Y,
                            -(lateral_force / velocity) * DT,
                        )) * next_rot;

                        pos += (next_rot * DVec3::Z) * delta_distance;

                        rot = next_rot;
                    } else {
                        break;
                    }

                    let track_point = TrackPoint {
                        pos,
                        rot: rot.into(),
                        velocity,
                        time: time + start.time,
                    };
                    if fixed_speed.is_none() {
                        velocity = track_friction(
                            self.config.parameter,
                            self.config.resistance,
                            self.config.heartline_height,
                            spline.points.last().unwrap_or(&track_point),
                            &track_point,
                            DT,
                        );
                    }

                    if velocity <= 0.0 {
                        return spline;
                    }

                    spline.points.push(TrackPoint {
                        pos,
                        rot: rot.into(),
                        velocity,
                        time: time + start.time,
                    });
                    time += DT;
                }
            }
        }
        spline
    }
}

fn track_friction(
    parameter: f64,
    resistance: f64,
    heartline_height: f64,
    last_point: &TrackPoint,
    point: &TrackPoint,
    dt: f64,
) -> f64 {
    let track_pos_friction = point.pos + (point.rot.0 * (DVec3::NEG_Y * heartline_height * 0.9));

    let last_track_pos_friction =
        last_point.pos + (last_point.rot.0 * (DVec3::NEG_Y * heartline_height * 0.9));

    let mut energy = 0.5 * last_point.velocity * last_point.velocity;

    energy -= last_point.velocity * last_point.velocity * last_point.velocity * dt * resistance;

    if energy
        - (track_pos_friction[1] - last_track_pos_friction[1]
            + (track_pos_friction - last_track_pos_friction).length() * parameter)
            * G
        <= 0.0
    {
        return 0.0;
    }

    (2.0 * (energy
        - (track_pos_friction[1] - last_track_pos_friction[1]
            + (track_pos_friction - last_track_pos_friction).length() * parameter)
            * G))
        .sqrt()
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

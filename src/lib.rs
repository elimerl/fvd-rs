pub mod constants;
pub mod math;
pub mod track;
pub mod transitions;

use constants::G;
use glam::DVec3;
use math::{deg_diff, euler, WrapperDQuat};
use serde::{Deserialize, Serialize};
use track::Track;
use transitions::Forces;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct TrackPoint {
    pub pos: DVec3,
    pub rot: WrapperDQuat,
    pub velocity: f64,
    pub time: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct TrackSpline {
    pub points: Vec<TrackPoint>,
}

impl TrackSpline {
    pub fn eval_closest(&self, distance: f64) -> Option<(&TrackPoint, &TrackPoint)> {
        let mut total_dist = 0.0;
        for i in 1..self.points.len() {
            let dist = (self.points[i].pos - self.points[i - 1].pos).length();
            total_dist += dist;
            if total_dist >= distance {
                return Some((&self.points[i - 1], &self.points[i]));
            }
        }
        None
    }
    pub fn forces(&self, pos: f64) -> Option<Forces> {
        let points = self.eval_closest(pos);
        if let Some((last_point, point)) = points {
            let delta_dist = (point.pos - last_point.pos).length();

            let (last_yaw, last_pitch, _last_roll) = euler(last_point);
            let (yaw, pitch, roll) = euler(point);

            let pitch_from_last = deg_diff(last_pitch, pitch).to_radians();
            let yaw_from_last = deg_diff(last_yaw, yaw).to_radians();

            let temp = pitch.abs().to_radians().cos();

            let normal_d_angle = pitch_from_last * (-roll).to_radians().cos()
                - temp * -yaw_from_last * (-roll).to_radians().sin();
            let lateral_d_angle = -pitch_from_last * (roll).to_radians().sin()
                - temp * yaw_from_last * (roll).to_radians().cos();

            let force_vec = DVec3::Y
                + ((point.rot.0 * DVec3::Y)
                    * ((point.velocity * point.velocity) / (delta_dist / normal_d_angle) / G))
                + ((point.rot.0 * DVec3::NEG_X)
                    * ((point.velocity * point.velocity) / (delta_dist / lateral_d_angle) / G));

            Some(Forces {
                vert: force_vec.dot(point.rot.0 * DVec3::Y),
                lat: force_vec.dot(point.rot.0 * DVec3::NEG_X),
                roll: 0.0,
            })
        } else {
            None
        }
    }

    fn total_distance(&self) -> f64 {
        self.points.windows(2).fold(0.0, |acc, points| {
            acc + (points[1].pos - points[0].pos).length()
        })
    }
}

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg_attr(target_arch = "wasm32", wasm_bindgen)]
pub fn get_spline(track_json: &str) -> String {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    let track = serde_json::from_str::<Track>(track_json).unwrap();
    serde_json::to_string(&track.get_spline()).unwrap()
}

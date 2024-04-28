use std::ops::{Add, Sub};

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TransitionCurve {
    Linear,
    Quadratic,
    Cubic,
    Plateau,
    QuarticBump,
}

impl TransitionCurve {
    pub fn eval_timewarp(&self, t: f64, center: f64, tension: f64) -> f64 {
        self.eval(timewarp(t, center, tension))
    }
    pub fn eval(&self, mut t: f64) -> f64 {
        t = t.clamp(0., 1.);
        match self {
            TransitionCurve::Linear => t,
            TransitionCurve::Cubic => {
                if t < 0.5 {
                    4.0 * t * t * t
                } else {
                    1.0 - (-2.0 * t + 2.0).powf(3.0) / 2.0
                }
            }
            TransitionCurve::Quadratic => {
                if t < 0.5 {
                    2.0 * t * t
                } else {
                    1.0 - (-2.0 * t + 2.0).powf(2.0) / 2.0
                }
            }
            TransitionCurve::Plateau => {
                1.0 - (-15.0 * ((1.0 - (2.0 * t - 1.0).abs()).powi(3))).exp()
            }
            TransitionCurve::QuarticBump => t * t * (16.0 + t * (-32.0 + t * 16.0)),
        }
    }
}

fn timewarp(t: f64, center: f64, tension: f64) -> f64 {
    timewarp_tension(timewarp_center(t, center), tension)
}

fn timewarp_center(t: f64, center: f64) -> f64 {
    if center.abs() < 0.01 {
        t
    } else if center > 0.0 {
        t.powf(2.0_f64.powf(center / 2.0))
    } else {
        1.0 - (1.0 - t).powf(2.0_f64.powf(-center / 2.0))
    }
}

fn timewarp_tension(t: f64, tension: f64) -> f64 {
    if tension.abs() < 0.01 {
        t
    } else if tension > 0.0 {
        0.5 * ((2.0 * tension * (t - 0.5)).sinh() / tension.sinh() + 1.0)
    } else {
        0.5 * ((2.0 * tension.sinh() * (t - 0.5)).asinh() / tension + 1.0)
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Transition {
    pub curve: TransitionCurve,
    pub value: f64,
    pub length: f64,
    pub center: f64,
    pub tension: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Transitions {
    pub vert: Vec<Transition>,
    pub lat: Vec<Transition>,
    pub roll: Vec<Transition>,
}

impl Transitions {
    pub fn length(&self) -> f64 {
        self.vert
            .iter()
            .map(|t| t.length)
            .sum::<f64>()
            .min(self.lat.iter().map(|t| t.length).sum::<f64>())
            .min(self.roll.iter().map(|t| t.length).sum::<f64>())
    }

    fn evaluate_single(transitions: &[Transition], time: f64) -> Option<f64> {
        if time < 0.0 {
            return None;
        }
        let mut time_accum = 0.0;
        let mut value = 0.0;

        for transition in transitions {
            if time_accum <= time && time <= time_accum + transition.length {
                value += transition.curve.eval_timewarp(
                    (time - time_accum) / transition.length,
                    transition.center,
                    transition.tension,
                ) * transition.value;
                break;
            }
            value += transition.value * transition.curve.eval(1.0);
            time_accum += transition.length;
        }

        Some(value)
    }

    pub fn evaluate(&self, time: f64) -> Option<Forces> {
        if time < 0.0 {
            return None;
        }

        let vert = Self::evaluate_single(&self.vert, time)?;
        let lat = Self::evaluate_single(&self.lat, time)?;
        let roll = Self::evaluate_single(&self.roll, time)?;

        Some(Forces { vert, lat, roll })
    }
}

struct AbsoluteTransition {
    pub curve: TransitionCurve,
    pub value: f64,
    pub start_value: f64,
    pub start: f64,
    pub length: f64,
    pub center: f64,
    pub tension: f64,
}

pub struct FastTransitions {
    vert: Vec<AbsoluteTransition>,
    lat: Vec<AbsoluteTransition>,
    roll: Vec<AbsoluteTransition>,
    pub length: f64,
}

impl FastTransitions {
    pub fn new(transitions: &Transitions) -> Self {
        let mut vert = Vec::new();
        let mut lat = Vec::new();
        let mut roll = Vec::new();

        let mut time_accum = 0.0;
        let mut value_accum = 0.0;

        for transition in transitions.vert.iter() {
            vert.push(AbsoluteTransition {
                curve: transition.curve,
                value: transition.value,
                start_value: value_accum,
                start: time_accum,
                length: transition.length,
                center: transition.center,
                tension: transition.tension,
            });
            value_accum += transition.value * transition.curve.eval(1.0);
            time_accum += transition.length;
        }
        time_accum = 0.0;
        value_accum = 0.0;
        for transition in transitions.lat.iter() {
            lat.push(AbsoluteTransition {
                curve: transition.curve,
                value: transition.value,
                start_value: value_accum,
                start: time_accum,
                length: transition.length,
                center: transition.center,
                tension: transition.tension,
            });
            value_accum += transition.value * transition.curve.eval(1.0);
            time_accum += transition.length;
        }
        time_accum = 0.0;
        value_accum = 0.0;
        for transition in transitions.roll.iter() {
            roll.push(AbsoluteTransition {
                curve: transition.curve,
                value: transition.value,
                start_value: value_accum,
                start: time_accum,
                length: transition.length,
                center: transition.center,
                tension: transition.tension,
            });
            value_accum += transition.value * transition.curve.eval(1.0);
            time_accum += transition.length;
        }

        Self {
            vert,
            lat,
            roll,
            length: transitions.length(),
        }
    }

    pub fn evaluate(&self, time: f64) -> Option<Forces> {
        if time < 0.0 || time >= self.length {
            return None;
        }
        Some(Forces {
            vert: self.evaluate_single(&self.vert, time)?,
            lat: self.evaluate_single(&self.lat, time)?,
            roll: self.evaluate_single(&self.roll, time)?,
        })
    }

    fn evaluate_single(&self, transitions: &[AbsoluteTransition], time: f64) -> Option<f64> {
        if time < 0.0 || time >= self.length {
            return None;
        }

        let idx = find_range_index(transitions, time);

        let transition = &transitions[idx];
        let time_relative = time - transition.start;
        let value = transition.start_value
            + transition.value
                * transition.curve.eval_timewarp(
                    time_relative / transition.length,
                    transition.center,
                    transition.tension,
                );

        Some(value)
    }
}

fn find_range_index(arr: &[AbsoluteTransition], target: f64) -> usize {
    let mut low = 0;
    let mut high = arr.len();

    while low < high {
        let mid = (low + high) / 2;
        if arr[mid].start <= target {
            low = mid + 1;
        } else {
            high = mid;
        }
    }

    low.clamp(0, arr.len() - 1)
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Forces {
    pub vert: f64,
    pub lat: f64,
    pub roll: f64,
}

impl Add<Forces> for Forces {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            vert: self.vert + other.vert,
            lat: self.lat + other.lat,
            roll: self.roll + other.roll,
        }
    }
}

impl Sub<Forces> for Forces {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            vert: self.vert - other.vert,
            lat: self.lat - other.lat,
            roll: self.roll - other.roll,
        }
    }
}

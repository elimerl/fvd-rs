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
    pub dynamic_length: bool,
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

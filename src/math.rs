use glam::{DQuat, DVec3};
use serde::{Deserialize, Deserializer, Serialize};

use crate::TrackPoint;

pub fn euler(p: &TrackPoint) -> (f64, f64, f64) {
    let dir = p.rot.0 * DVec3::Z;
    let yaw = (-dir[0]).atan2(-dir[2]);
    let pitch = dir[1].atan2((dir[0] * dir[0] + dir[2] * dir[2]).sqrt());

    let up_dir = p.rot.0 * DVec3::Y;
    let right_dir = p.rot.0 * DVec3::NEG_X;

    let roll = (-right_dir[1]).atan2(up_dir[1]);
    (yaw.to_degrees(), pitch.to_degrees(), roll.to_degrees())
}

pub fn deg_diff(a: f64, b: f64) -> f64 {
    let mut diff = b - a;
    while diff < -180.0 {
        diff += 360.0;
    }
    while diff > 180.0 {
        diff -= 360.0;
    }
    diff
}

use serde::ser::SerializeTuple;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WrapperDQuat(pub DQuat);

impl From<WrapperDQuat> for DQuat {
    fn from(w: WrapperDQuat) -> Self {
        w.0
    }
}
impl From<DQuat> for WrapperDQuat {
    fn from(d: DQuat) -> Self {
        WrapperDQuat(d)
    }
}

impl Serialize for WrapperDQuat {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut tup = serializer.serialize_tuple(4)?;
        tup.serialize_element(&self.0.w)?;
        tup.serialize_element(&self.0.x)?;
        tup.serialize_element(&self.0.y)?;
        tup.serialize_element(&self.0.z)?;
        tup.end()
    }
}
impl<'de> Deserialize<'de> for WrapperDQuat {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct DQuatVisitor;

        impl<'de> serde::de::Visitor<'de> for DQuatVisitor {
            type Value = WrapperDQuat;
            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("a tuple of 4 floats")
            }
            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let w = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;
                let x = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(1, &self))?;
                let y = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(2, &self))?;
                let z = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(3, &self))?;

                let q = DQuat::from_xyzw(x, y, z, w);

                Ok(WrapperDQuat(q))
            }
        }
        deserializer.deserialize_tuple(4, DQuatVisitor)
    }
}

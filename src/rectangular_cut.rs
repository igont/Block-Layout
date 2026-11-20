use serde::{Deserialize, Serialize};

const EPS: f64 = 1e-7;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    Left,
    Right,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct RectangularCut {
    pub side: Side,
    #[serde(rename = "offset_mm", with = "millimeters")]
    pub offset_centimm: i64,
    #[serde(rename = "bottom_mm", with = "millimeters")]
    pub bottom_centimm: i64,
    #[serde(rename = "width_mm", with = "millimeters")]
    pub width_centimm: i64,
    #[serde(rename = "height_mm", with = "millimeters")]
    pub height_centimm: i64,
    #[serde(rename = "depth_mm", with = "millimeters")]
    pub depth_centimm: i64,
    pub source_beam_id: String,
}

mod millimeters {
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(value: &i64, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_f64(*value as f64 / 100.0)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<i64, D::Error> {
        let units = crate::precision::centimm(f64::deserialize(deserializer)?);
        if !(0.0..i64::MAX as f64).contains(&units) {
            return Err(serde::de::Error::custom("Invalid cut dimension"));
        }
        Ok(units as i64)
    }
}

pub(crate) struct BlindBeamEnd {
    pub side: Side,
    pub depth: f64,
}

impl BlindBeamEnd {
    // The wall section is centered at local Y=0. Both endpoints and thickness
    // and thickness are expressed in millimeters. Callers have checked the vertical frame.
    pub(crate) fn project(start: (f64, f64), end: (f64, f64), half_thickness: f64) -> Option<Self> {
        if (end.0 - start.0).abs() > EPS {
            return None;
        }
        let low = start.1.min(end.1);
        let high = start.1.max(end.1);
        let (side, depth) = if low <= -half_thickness + EPS
            && high > -half_thickness + EPS
            && high < half_thickness - EPS
        {
            (Side::Right, high + half_thickness)
        } else if high >= half_thickness - EPS
            && low < half_thickness - EPS
            && low > -half_thickness + EPS
        {
            (Side::Left, half_thickness - low)
        } else {
            return None;
        };
        // A residual skin that vanishes on the exchange grid is a through cut.
        (crate::precision::centimm(depth) < crate::precision::centimm(2.0 * half_thickness))
            .then_some(Self { side, depth })
    }
}

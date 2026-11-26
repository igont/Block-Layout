//! Virtual source geometry for calculation; the source snapshot stays unchanged.
use crate::api::{Beam, LayoutRequest, Volume};
use crate::domain::RawPoint;
use crate::topology::BeamAbutment;
use serde_json::{json, Value};

pub const BEAM_RULE: &str = "horizontal_beam_320_to_315";

pub(crate) fn horizontal(beam: &Beam) -> bool {
    let geometry = &beam.geometry;
    geometry.height_direction_x.abs() < 1e-7
        && geometry.height_direction_y.abs() < 1e-7
        && (geometry.height_direction_z.abs() - 1.0).abs() < 1e-7
        && (beam.end_zmm - beam.start_zmm).abs() < 1e-7
}

pub(crate) fn beam_abutments(request: &LayoutRequest) -> Vec<BeamAbutment> {
    request.beams.iter().filter(|beam| horizontal(beam)).filter_map(|beam| {
        let other_z = beam.start_zmm + beam.geometry.height_direction_z * beam.geometry.height_mm;
        BeamAbutment::new(
            RawPoint { x_mm: beam.start_xmm, y_mm: beam.start_ymm },
            RawPoint { x_mm: beam.end_xmm, y_mm: beam.end_ymm },
            beam.start_zmm.min(other_z),
            beam.start_zmm.max(other_z),
        )
    }).collect()
}

/// Normalize a cloned snapshot once before any beam constraints or body clipping.
/// SUP's axis is the lower face for +Z and the upper face for -Z.
pub fn normalize_beams(request: &mut LayoutRequest) -> Vec<Value> {
    let mut adjustments = Vec::new();
    for beam in &mut request.beams {
        if !horizontal(beam) || (beam.geometry.height_mm * 100.0).round() != 32_000.0 {
            continue;
        }
        let geometry = &mut beam.geometry;
        let original_height = geometry.height_mm;
        let direction = geometry.height_direction_z;
        let bottom_start = beam.start_zmm + (original_height * direction).min(0.0);
        let bottom_end = beam.end_zmm + (original_height * direction).min(0.0);
        geometry.height_mm = 315.0;
        beam.start_zmm = bottom_start - (315.0 * direction).min(0.0);
        beam.end_zmm = bottom_end - (315.0 * direction).min(0.0);
        adjustments.push(json!({
            "beam_id":beam.guid,"rule_id":BEAM_RULE,"original_height_mm":original_height,
            "height_mm":315.0,"start_z_mm":beam.start_zmm,"end_z_mm":beam.end_zmm,
            "bottom_start_z_mm":bottom_start,"bottom_end_z_mm":bottom_end,
            "height_direction":[geometry.height_direction_x,geometry.height_direction_y,direction]
        }));
    }
    adjustments
}

pub fn beam_adjustments(request: &LayoutRequest) -> Vec<Value> {
    normalize_beams(&mut request.clone())
}

pub(crate) fn normalize_openings(request: &mut LayoutRequest, course_height: f64) {
    for opening in &mut request.opening_volumes {
        if !opening.is_outside && opening.start_bottom_zmm.abs() < 0.01
            && opening.end_bottom_zmm.abs() < 0.01 {
            opening.start_bottom_zmm = -2.0 * course_height;
            opening.end_bottom_zmm = opening.start_bottom_zmm;
        }
    }
}

pub(crate) fn is_fb_opening(opening: &Volume) -> bool {
    matches!(opening.opening_type.as_str(), "OPENING" | "CONSOLE" | "WINDOW" | "DOOR")
}

/// The part of a wall opening that clears a full course, in local centimillimeters.
pub(crate) fn opening_course_interval(
    opening: &Volume,
    (a, av): (f64, f64),
    (b, bv): (f64, f64),
    half_width: f64,
    [bottom, top]: [f64; 2],
) -> Option<(i64, i64)> {
    const EPS: f64 = 1e-7;
    let za = opening.start_bottom_zmm;
    let zb = opening.end_bottom_zmm;
    let ha = opening.start_top_zmm;
    let hb = opening.end_top_zmm;
    if (av - bv).abs() > 0.02 || av.abs() > half_width + 0.02
        || top <= za.min(zb) + EPS || bottom >= ha.max(hb) - EPS {
        return None;
    }
    let quantize = |x: f64| (x * 100.0).round() as i64;
    let crossing = |at, from, to| quantize(a + (b - a) * (at - from) / (to - from));
    let mut low = quantize(a.min(b));
    let mut high = quantize(a.max(b));
    if (ha - hb).abs() > EPS && bottom > ha.min(hb) {
        let x = crossing(bottom, ha, hb);
        if (hb > ha) == (b > a) { low = x; } else { high = x; }
    }
    if (za - zb).abs() > EPS && top < za.max(zb) {
        let x = crossing(top, za, zb);
        if (zb > za) == (b > a) { high = high.min(x); } else { low = low.max(x); }
    }
    (low < high).then_some((low, high))
}

//! Virtual source geometry for calculation; the source snapshot stays unchanged.
use crate::api::LayoutRequest;
use serde_json::{json, Value};

pub const BEAM_RULE: &str = "horizontal_beam_320_to_315";

/// Normalize a cloned snapshot once before any beam constraints or body clipping.
/// SUP's axis is the lower face for +Z and the upper face for -Z.
pub fn normalize_beams(request: &mut LayoutRequest) -> Vec<Value> {
    let mut adjustments = Vec::new();
    for beam in &mut request.beams {
        let geometry = &mut beam.geometry;
        let vertical = geometry.height_direction_x.abs() < 1e-7
            && geometry.height_direction_y.abs() < 1e-7
            && (geometry.height_direction_z.abs() - 1.0).abs() < 1e-7;
        if !vertical || (beam.end_zmm - beam.start_zmm).abs() >= 1e-7
            || (geometry.height_mm * 100.0).round() != 32_000.0 {
            continue;
        }
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

//! Independent lamella calculation reads the saved course grid.
//! It never calls the block solver or publishes block changes.
use crate::api::ApiFailure;
use crate::domain::WallRun;
use crate::exchange::ExchangeRequest;
use crate::lamella::{Lamella, LamellaPlan};
use crate::layout::Profile;
use crate::solid_geometry::Mesh;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::BTreeSet;

const EPS: f64 = 1e-7;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SavedPlacement {
    pub origin_mm: [f64; 3],
    pub x_axis: [f64; 3],
    pub y_axis: [f64; 3],
    pub z_axis: [f64; 3],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SavedBlock {
    pub id: String,
    pub placement: SavedPlacement,
    pub length_mm: f64,
    pub width_mm: f64,
    pub height_mm: f64,
    pub course_index: i64,
    pub wall_ids: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub solids: Vec<Mesh>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub grid_start_mm: Option<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub grid_end_mm: Option<[f64; 3]>,
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn delta(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn invalid(id: &str, message: &str) -> ApiFailure {
    ApiFailure::new("INVALID_SAVED_BLOCK", message, Some(id.to_owned()))
}

fn validate(blocks: &[SavedBlock]) -> Result<(), ApiFailure> {
    let mut ids = BTreeSet::new();
    for b in blocks {
        if b.id.trim().is_empty() || !ids.insert(&b.id) {
            return Err(invalid(&b.id, "Пустой или повторный ID сохранённого блока"));
        }
        let p = &b.placement;
        let axes = [p.x_axis, p.y_axis, p.z_axis];
        if p.origin_mm
            .iter()
            .chain(axes.iter().flatten())
            .chain([b.length_mm, b.width_mm, b.height_mm].iter())
            .any(|v| !v.is_finite())
            || [b.length_mm, b.width_mm, b.height_mm]
                .iter()
                .any(|v| *v <= 0.0)
            || axes.iter().any(|a| (dot(*a, *a) - 1.0).abs() > 1e-6)
            || dot(axes[0], axes[1]).abs() > 1e-6
            || dot(axes[0], axes[2]).abs() > 1e-6
            || dot(axes[1], axes[2]).abs() > 1e-6
            || p.z_axis != [0.0, 0.0, 1.0]
        {
            return Err(invalid(
                &b.id,
                "Некорректное положение или габариты сохранённого блока",
            ));
        }
        if b.grid_start_mm.is_some() != b.grid_end_mm.is_some()
            || b.grid_start_mm
                .iter()
                .chain(b.grid_end_mm.iter())
                .flatten()
                .any(|v| !v.is_finite())
        {
            return Err(invalid(
                &b.id,
                "Сохранённые точки сетки должны быть парой конечных координат",
            ));
        }
    }
    Ok(())
}

/// Continue whole saved modules, not the cut ends beside a beam. If a course
/// has no whole module, its nearest saved course supplies the bonded phase.
pub(crate) fn seams(
    blocks: &[SavedBlock],
    run: &WallRun,
    wall: &str,
    bottom: f64,
    top: f64,
    course_index: i64,
    profile: &Profile,
    low: f64,
    high: f64,
) -> Option<Vec<f64>> {
    let module = profile.ordinary_length_centimm as f64 / 100.0;
    let o = [run.start.x as f64 / 100.0, run.start.y as f64 / 100.0];
    let d = [
        (run.end.x - run.start.x) as f64 / 100.0,
        (run.end.y - run.start.y) as f64 / 100.0,
    ];
    let len = d[0].hypot(d[1]);
    let u = [d[0] / len, d[1] / len];
    let v = [-u[1], u[0]];
    let mut references = Vec::new();
    let mut matching_saved = false;
    for b in blocks {
        let p = &b.placement;
        if !b.wall_ids.is_empty() && !b.wall_ids.iter().any(|id| id == wall) {
            continue;
        }
        if (p.x_axis[0] * u[0] + p.x_axis[1] * u[1]).abs() < 1.0 - 1e-6 {
            continue;
        }
        if b.wall_ids.is_empty()
            && ((p.origin_mm[0] - o[0]) * v[0] + (p.origin_mm[1] - o[1]) * v[1]).abs() > 0.01
        {
            continue;
        }
        matching_saved = true;
        let a = b.grid_start_mm.unwrap_or(p.origin_mm);
        let z = b.grid_end_mm.unwrap_or(std::array::from_fn(|i| {
            p.origin_mm[i] + p.x_axis[i] * b.length_mm
        }));
        let start = (a[0] - o[0]) * u[0] + (a[1] - o[1]) * u[1];
        let end = (z[0] - o[0]) * u[0] + (z[1] - o[1]) * u[1];
        // Nominal endpoints exclude corner stock extensions. A short remnant
        // does not establish a new facade joint at the end of the beam.
        let nominal = (end - start).abs();
        if nominal < module - 10.0 - 0.01 || nominal > module + 0.01 {
            continue;
        }
        let same_course = p.origin_mm[2] < top - EPS && p.origin_mm[2] + b.height_mm > bottom + EPS;
        let distance = if same_course {
            0
        } else {
            b.course_index.abs_diff(course_index)
        };
        let shift = if same_course {
            0.0
        } else {
            (b.course_index.rem_euclid(2) != course_index.rem_euclid(2)) as u8 as f64 * module / 2.0
        };
        let begin = start.min(end);
        references.push((distance, begin + shift, begin + module + shift));
    }
    if references.is_empty() {
        if !matching_saved {
            return None;
        }
        let residue = crate::layout::ordinary_residue(run, course_index, profile)? as f64 / 100.0;
        let begin = residue + ((low - residue) / module).floor() * module;
        let mut result = Vec::new();
        let mut at = begin;
        while at < high + module - EPS {
            result.push(at);
            at += module;
        }
        return Some(result);
    }
    let nearest = references.iter().map(|r| r.0).min()?;
    let mut joints: Vec<f64> = references
        .into_iter()
        .filter(|r| r.0 == nearest)
        .flat_map(|r| [r.1, r.2])
        .collect();
    joints.sort_by(f64::total_cmp);
    joints.dedup_by(|a, b| (*a - *b).abs() < EPS);
    let first = joints[0];
    let last = joints[joints.len() - 1];
    let mut at = first - module;
    while at > low - module - EPS {
        joints.push(at);
        at -= module;
    }
    let mut at = last + module;
    while at < high + module + EPS {
        joints.push(at);
        at += module;
    }
    joints.sort_by(f64::total_cmp);
    let mut result = Vec::new();
    for pair in joints.windows(2) {
        result.push(pair[0]);
        let mut at = pair[0] + module;
        while at < pair[1] - EPS {
            result.push(at);
            at += module;
        }
    }
    result.push(*joints.last()?);
    Some(result)
}

pub fn calculate(standard: &ExchangeRequest, profile: &Profile) -> Result<LamellaPlan, ApiFailure> {
    let request = standard.to_layout_request()?;
    standard.validate_profile(profile)?;
    let saved = standard.model.saved_blocks.as_deref().ok_or_else(|| {
        ApiFailure::new(
            "INVALID_EXCHANGE",
            "Не передана сетка сохранённых блоков",
            None,
        )
    })?;
    validate(saved)?;
    let mut normalized = saved.to_vec();
    for b in &mut normalized {
        b.placement.origin_mm = b.placement.origin_mm.map(crate::precision::mm);
        b.length_mm = crate::precision::mm(b.length_mm);
        b.width_mm = crate::precision::mm(b.width_mm);
        b.height_mm = crate::precision::mm(b.height_mm);
        b.grid_start_mm = b.grid_start_mm.map(|p| p.map(crate::precision::mm));
        b.grid_end_mm = b.grid_end_mm.map(|p| p.map(crate::precision::mm));
    }
    let mut plan = crate::lamella::calculate_from_saved(&request, profile, &normalized)?;
    assign_grid_recesses(&mut plan.lamellas, &normalized);
    Ok(plan)
}

pub fn assign_grid_recesses(lamellas: &mut [Lamella], blocks: &[SavedBlock]) {
    let quantum = crate::precision::QUANTUM_MM;
    for lamella in lamellas {
        lamella.recess_block_ids.clear();
        let axis = lamella.placement.x_axis;
        let start = dot(lamella.grid_start_mm, axis);
        let end = dot(lamella.grid_end_mm, axis);
        for block in blocks {
            if block.course_index != lamella.course_index
                || dot(block.placement.x_axis, axis).abs() < 1.0 - 1e-6
                || (!block.wall_ids.is_empty()
                    && !block
                        .wall_ids
                        .iter()
                        .any(|id| lamella.wall_ids.contains(id)))
                || dot(
                    delta(block.placement.origin_mm, lamella.grid_start_mm),
                    lamella.placement.y_axis,
                )
                .abs()
                    > block.width_mm / 2.0 + quantum
            {
                continue;
            }
            let a = block.grid_start_mm.unwrap_or(block.placement.origin_mm);
            let b = block.grid_end_mm.unwrap_or(std::array::from_fn(|i| {
                block.placement.origin_mm[i] + block.placement.x_axis[i] * block.length_mm
            }));
            let a = dot(a, axis);
            let b = dot(b, axis);
            if end.max(start).min(a.max(b)) - end.min(start).max(a.min(b)) > quantum {
                lamella.recess_block_ids.insert(block.id.clone());
            }
        }
    }
}

pub fn export(standard: &ExchangeRequest, profile: &Profile) -> Result<Value, ApiFailure> {
    let plan = calculate(standard, profile)?;
    let request = standard.to_layout_request()?;
    let mut result = crate::exchange::success_result_with_warnings(
        standard,
        vec![],
        crate::effective_geometry::beam_adjustments(&request),
        &plan.warnings,
    );
    result["format"] = json!("fb-layout-codes/1");
    result["lamellas"] = json!(plan.lamellas);
    crate::precision::normalize_result(&mut result);
    Ok(result)
}

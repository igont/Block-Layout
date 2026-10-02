//! Детерминированная раскладка по курсовым ребрам; координаты в 0,01 мм.

use crate::choice::{self, BinaryConstraint, Problem, SolveError, UnaryConstraint, Variable};
use crate::constraints::{
    Constraints, Exclusion, Interval, LintelCandidate, MaskCoverage, MaskSource, SolidBox,
};
use crate::domain::{CourseEdge, Point, Topology};
use crate::grid::joint_residue;
use crate::node_assembly::{assemble_node, NodeKind};
use crate::node_geometry::{
    active_rays, resolve_node, NodeArmInput, NodeGeometry, NodeGeometryError, NodeRayInput,
};
use crate::vertical::{self, JointEvidence, VerticalError, VerticalVerdict};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

const MAX_BLOCKS: usize = 100_000;
const MAX_SEARCH_STATES: usize = 100_000;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Profile {
    pub profile_id: String,
    pub revision: String,
    pub catalog_version: String,
    #[serde(default)]
    pub node_assembly_family: Option<String>,
    #[serde(default)]
    pub world_joint_policy: Option<String>,
    pub lintel_support_mm: f64,
    #[serde(default)]
    pub longitudinal_full_wall_thickness: bool,
    #[serde(default)]
    pub full_course_clearance: bool,
    #[serde(default)]
    pub assembly_clearance_mm: f64,
    pub ordinary_length_centimm: i64,
    #[serde(default)]
    pub maximum_ordinary_blank_centimm: Option<i64>,
    #[serde(default)]
    pub maximum_special_blank_centimm: Option<i64>,
    pub ordinary_nominal_lengths_centimm: Vec<i64>,
    #[serde(default = "one")]
    pub max_ordinary_cuts_per_free_span: usize,
    pub minimum_cut_centimm: i64,
    pub index_centimm: i64,
    pub coordinate_tolerance_centimm: i64,
    pub phase_count: i64,
    pub nodes: Vec<NodeSpec>,
    pub bridge_nominal_lengths_centimm: Vec<i64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NodeSpec {
    pub kind: String,
    pub phase: i64,
    #[serde(default)]
    pub variant: Option<String>,
    pub rays: Vec<RayLength>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RayLength {
    pub direction_deg: u16,
    pub length_centimm: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DiagnosticCode {
    UnsupportedCatalog,
    UnsupportedGeometry,
    InvalidMask,
    InvalidSupport,
    UnsupportedNodeCut,
    NodeCollision,
    PhaseConflict,
    NodeVerticalRuleMissing,
    VerticalJointConflict,
    NodeDistalWallTrim,
    WorldGridIncompatible,
    MinimumOrdinarySpan,
    UncoveredSolid,
    SearchExhausted,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Diagnostic {
    pub code: DiagnosticCode,
    pub message: String,
    pub wall_id: Option<String>,
    pub edge_id: Option<String>,
    pub course_index: Option<i64>,
    #[serde(default = "one")]
    pub occurrences: usize,
    #[serde(default)]
    pub source_ids: Vec<String>,
}

fn one() -> usize {
    1
}

fn natural_end() -> bool {
    true
}

fn mask_source(source: &MaskSource) -> String {
    match source {
        MaskSource::Opening(id) => format!("opening:{id}"),
        MaskSource::Beam(id) => format!("beam:{id}"),
    }
}

fn node_error(error: NodeGeometryError, edges: &[CourseEdge], course: i64) -> Diagnostic {
    let (code, run_id, source_id, interval, reason) = match error {
        NodeGeometryError::InvalidMask {
            run_id,
            source_id,
            interval,
        } => (
            DiagnosticCode::InvalidMask,
            run_id,
            source_id,
            interval,
            "Некорректная маска узла",
        ),
        NodeGeometryError::UnsupportedNodeCut {
            run_id,
            source_id,
            interval,
            reason,
        } => (
            DiagnosticCode::UnsupportedNodeCut,
            run_id,
            source_id,
            interval,
            reason,
        ),
    };
    let edge = edges.iter().find(|e| e.edge_id == run_id);
    let mut result = diagnostic(
        code,
        format!(
            "{reason}: {source_id} run {run_id} [{}, {})",
            interval.start, interval.end
        ),
        edge,
        Some(course),
    );
    result.source_ids.push(source_id);
    result.source_ids.push(run_id);
    result
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObstacleEnd {
    pub left: bool,
    pub source_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Block {
    pub id: String,
    pub wall_id: String,
    pub edge_id: String,
    pub course_index: i64,
    pub z_centimm: i64,
    pub start: Point,
    pub end: Point,
    pub length_centimm: i64,
    pub kind: String,
    pub product_key: Option<String>,
    pub catalog_status: String,
    pub rotation_deg: u16,
    pub local_origin: Option<Point>,
    pub local_rotation_deg: Option<u16>,
    pub is_bridge: bool,
    pub hide_spikes_left: bool,
    pub hide_spikes_right: bool,
    #[serde(default = "natural_end")]
    pub natural_end_left: bool,
    #[serde(default = "natural_end")]
    pub natural_end_right: bool,
    pub cuts: Vec<String>,
    pub source_ids: Vec<String>,
    pub catalog_nominal_centimm: Option<i64>,
    #[serde(default)]
    pub arms: Vec<BlockArm>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<Block>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub obstacle_ends: Vec<ObstacleEnd>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BlockArm {
    pub wall_id: String,
    pub edge_id: String,
    pub start: Point,
    pub end: Point,
    pub length_centimm: i64,
}

/// Окончательные детали: препятствие разделяет всю толщину и высоту изделия.
/// Вызывается и на границе экспорта для кандидатов, созданных до изменения входа.
pub fn finalize_parts(
    request: &crate::api::LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
) -> Result<Vec<Block>, crate::api::ApiFailure> {
    if blocks.len() > MAX_BLOCKS {
        return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT", "Превышено число исходных деталей", None));
    }
    // Проверяем дерево до первого clone: глубокая входная сборка не должна
    // переполнить стек внутри автоматически выведенного Clone.
    let mut pending: Vec<_> = blocks.iter().map(|part| (part, 0_usize)).collect();
    let mut count = 0_usize;
    while let Some((part, depth)) = pending.pop() {
        count += 1;
        if depth > 16 || count.saturating_add(pending.len()).saturating_add(part.components.len()) > MAX_BLOCKS {
            return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT", "Превышены пределы числа деталей или вложенности сборки", Some(part.id.clone())));
        }
        pending.extend(part.components.iter().map(|child| (child, depth + 1)));
    }
    let mut work = 0;
    let nominal = finalize_parts_inner(request, profile, blocks, 0, &mut work, &[])?;
    let reveals = finished_opening_reveals(request, profile, &nominal, &mut work)?;
    if reveals.is_empty() { return Ok(nominal); }
    finalize_parts_inner(request, profile, &nominal, 0, &mut work, &reveals)
}

/// Натуральные скрытые торцы задают общую готовую плоскость обсады.
/// Длина опорной детали позволяет подгонять только более короткие фрагменты.
fn finished_opening_reveals(
    request: &crate::api::LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
    work: &mut usize,
) -> Result<Vec<(usize, bool, i64)>, crate::api::ApiFailure> {
    let mut reveals: Vec<(usize, bool, i64)> = Vec::new();
    let mut pending: Vec<_> = blocks.iter().collect();
    while let Some(part) = pending.pop() {
        *work = work.saturating_add(request.opening_volumes.len());
        if *work > MAX_BLOCKS * 256 {
            return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT", "Превышен объём проверки общих плоскостей проёмов", Some(part.id.clone())));
        }
        pending.extend(part.components.iter());
        let bottom = part.z_centimm as f64 / 100.0;
        let top = bottom + profile.index_centimm as f64 / 100.0;
        let angle = f64::from(part.rotation_deg).to_radians();
        for (index, opening) in request.opening_volumes.iter().enumerate() {
            if !matches!(opening.opening_type.as_str(), "OPENING" | "WINDOW" | "DOOR")
                || top <= opening.start_bottom_zmm.min(opening.end_bottom_zmm) + 1e-7
                || bottom >= opening.start_top_zmm.max(opening.end_top_zmm) - 1e-7 { continue; }
            let dx = opening.end_xmm - opening.start_xmm;
            let dy = opening.end_ymm - opening.start_ymm;
            if (dx * angle.sin() - dy * angle.cos()).abs() > 0.02 { continue; }
            for (is_start, x, y) in [(true, opening.start_xmm, opening.start_ymm),
                (false, opening.end_xmm, opening.end_ymm)] {
                let matches = [(part.start, part.natural_end_left, part.hide_spikes_left),
                    (part.end, part.natural_end_right, part.hide_spikes_right)].iter()
                    .any(|(point, natural, hidden)| *natural && *hidden
                        && (point.x as f64 / 100.0 - x).abs() < 0.02
                        && (point.y as f64 / 100.0 - y).abs() < 0.02);
                if !matches { continue; }
                if let Some((_, _, size)) = reveals.iter_mut().find(|(i, side, _)| *i == index && *side == is_start) {
                    *size = (*size).max(part.length_centimm);
                } else { reveals.push((index, is_start, part.length_centimm)); }
            }
        }
    }
    Ok(reveals)
}

fn finalize_parts_inner(
    request: &crate::api::LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
    depth: usize,
    work: &mut usize,
    reveals: &[(usize, bool, i64)],
) -> Result<Vec<Block>, crate::api::ApiFailure> {
    if blocks.len() > MAX_BLOCKS || depth > 16 {
        return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT",
            "Превышены пределы числа деталей или вложенности сборки", None));
    }
    let mut result = Vec::new();
    for original in blocks {
        *work = work.saturating_add(request.beams.len())
            .saturating_add(request.opening_volumes.len())
            .saturating_add(request.wall_volumes.len().saturating_mul(3));
        if *work > MAX_BLOCKS * 256 {
            return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT",
                "Превышен объём проверки препятствий", Some(original.id.clone())));
        }
        if (!original.natural_end_left && !original.hide_spikes_left)
            || (!original.natural_end_right && !original.hide_spikes_right) {
            return Err(crate::api::ApiFailure::new("INVALID_END_STATE",
                "Искусственный торец не может иметь шипы", Some(original.id.clone())));
        }
        if original.length_centimm <= 0 {
            continue;
        }
        let angle = f64::from(original.rotation_deg).to_radians();
        let (ux, uy) = (angle.cos(), angle.sin());
        let origin = (original.start.x as f64 / 100.0, original.start.y as f64 / 100.0);
        let length = original.length_centimm as f64 / 100.0;
        let owns_wall = |id: &str| original.wall_id.strip_prefix("wall:").unwrap_or(&original.wall_id) == id
            || original.source_ids.iter().any(|source| source.strip_prefix("wall:").unwrap_or(source) == id)
            || original.arms.iter().any(|arm| arm.wall_id.strip_prefix("wall:").unwrap_or(&arm.wall_id) == id);
        let width = request.wall_volumes.iter()
            .find(|w| owns_wall(&w.guid))
            .map_or(193.0, |w| w.thickness_mm);
        let project = |x: f64, y: f64| {
            let (x, y) = (x - origin.0, y - origin.1);
            (x * ux + y * uy, -x * uy + y * ux)
        };
        let bottom = original.z_centimm as f64 / 100.0;
        let top = bottom + profile.index_centimm as f64 / 100.0;
        let mut exclusions = Vec::<(i64, i64, String)>::new();
        for beam in &request.beams {
            let (dx, dy) = (beam.end_xmm - beam.start_xmm, beam.end_ymm - beam.start_ymm);
            let column = dx.hypot(dy) <= 1e-7 && (beam.end_zmm - beam.start_zmm).abs() > 1e-7;
            if !column && ((beam.end_zmm - beam.start_zmm).abs() > 1e-7
                || beam.geometry.height_direction_x.abs() > 1e-7
                || beam.geometry.height_direction_y.abs() > 1e-7) {
                continue;
            }
            let other_z = if column { beam.end_zmm } else {
                beam.start_zmm + beam.geometry.height_mm * beam.geometry.height_direction_z
            };
            if top <= beam.start_zmm.min(other_z) + 1e-7
                || bottom >= beam.start_zmm.max(other_z) - 1e-7 {
                continue;
            }
            let (a, av) = project(beam.start_xmm, beam.start_ymm);
            let (end_x, end_y) = if column {
                (beam.start_xmm + beam.geometry.height_direction_x * beam.geometry.height_mm,
                    beam.start_ymm + beam.geometry.height_direction_y * beam.geometry.height_mm)
            } else { (beam.end_xmm, beam.end_ymm) };
            let beam_length = (end_x - beam.start_xmm).hypot(end_y - beam.start_ymm);
            if beam_length <= 1e-7 { continue; }
            let (b, bv) = project(end_x, end_y);
            let axial = (b - a) / beam_length;
            let across = (bv - av) / beam_length;
            let half = beam.geometry.width_mm / 2.0;
            let corners = [(a - across * half, av + axial * half),
                (a + across * half, av - axial * half),
                (b - across * half, bv + axial * half),
                (b + across * half, bv - axial * half)];
            // SAT: касание прямоугольников не удаляет материал соседней детали.
            let intersects = [(1.0, 0.0), (0.0, 1.0), (axial, across), (-across, axial)]
                .into_iter().all(|(x, y)| {
                    let beam_min = corners.iter().map(|p| p.0 * x + p.1 * y).fold(f64::INFINITY, f64::min);
                    let beam_max = corners.iter().map(|p| p.0 * x + p.1 * y).fold(f64::NEG_INFINITY, f64::max);
                    let block_min = (length * x).min(0.0) - width / 2.0 * y.abs();
                    let block_max = (length * x).max(0.0) + width / 2.0 * y.abs();
                    if y.abs() < 1e-7 && (x.abs() - 1.0).abs() < 1e-7 {
                        beam_max >= block_min - 1e-7 && block_max >= beam_min - 1e-7
                    } else {
                        beam_max > block_min + 1e-7 && block_max > beam_min + 1e-7
                    }
                });
            if !intersects { continue; }
            let (mut low, mut high) = if !column && across.abs() > axial.abs() {
                let center = a - av * (b - a) / (bv - av);
                (center - half, center + half)
            } else {
                (corners.iter().map(|p| p.0).fold(f64::INFINITY, f64::min),
                    corners.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max))
            };
            if !column && profile.world_joint_policy.as_deref() == Some("affine_checkerboard_v1")
                && matches!(original.rotation_deg, 0 | 90 | 180 | 270)
                && across.abs() < 1e-7 && av.abs() < 1e-7 && bv.abs() < 1e-7 {
                let sign = if ux + uy >= 0.0 { 1.0 } else { -1.0 };
                let residue = -(origin.0 + origin.1) * 100.0 * sign;
                let normalized = crate::constraints::nominal_axis_bounds(
                    (low * 100.0, high * 100.0), (a.min(b) * 100.0, a.max(b) * 100.0), residue);
                low = normalized.0 / 100.0;
                high = normalized.1 / 100.0;
            }
            exclusions.push(((low * 100.0).round() as i64, (high * 100.0).round() as i64,
                format!("beam:{}", beam.guid)));
        }
        for (opening_index, opening) in request.opening_volumes.iter().enumerate() {
            if !matches!(opening.opening_type.as_str(), "OPENING" | "CONSOLE" | "WINDOW" | "DOOR") {
                continue;
            }
            let (a, av) = project(opening.start_xmm, opening.start_ymm);
            let (b, bv) = project(opening.end_xmm, opening.end_ymm);
            if (av - bv).abs() > 0.02 || av.abs() > width / 2.0 + 0.02
                || top <= opening.start_bottom_zmm.min(opening.end_bottom_zmm) + 1e-7
                || bottom >= opening.start_top_zmm.max(opening.end_top_zmm) - 1e-7 {
                continue;
            }
            let mut low = (a.min(b) * 100.0).round() as i64;
            let mut high = (a.max(b) * 100.0).round() as i64;
            for &(index, is_start, reference_length) in reveals {
                if index != opening_index || original.length_centimm >= reference_length { continue; }
                let boundary = ((if is_start { a } else { b }) * 100.0).round() as i64;
                if !original.natural_end_right && boundary == original.length_centimm && boundary == low {
                    low -= 500;
                }
                if !original.natural_end_left && boundary == 0 && boundary == high {
                    high += 500;
                }
            }
            exclusions.push((low, high, format!("opening:{}", opening.guid)));
        }
        let mut stock_span = Span { start: 0, end: original.length_centimm };
        if original.kind.starts_with("node_") {
            let wall_spans: Vec<_> = request.wall_volumes.iter().filter(|wall| {
                owns_wall(&wall.guid)
            }).filter_map(|wall| {
                let (a, av) = project(wall.start_xmm, wall.start_ymm);
                let (b, bv) = project(wall.end_xmm, wall.end_ymm);
                ((bv - av).abs() < 0.02).then_some((a.min(b), a.max(b)))
            }).collect();
            if !wall_spans.is_empty() {
                stock_span.start = (wall_spans.iter().map(|s| s.0).fold(f64::INFINITY, f64::min) * 100.0).round() as i64;
                stock_span.end = (wall_spans.iter().map(|s| s.1).fold(f64::NEG_INFINITY, f64::max) * 100.0).round() as i64;
                stock_span.start = stock_span.start.max(0);
                stock_span.end = stock_span.end.min(original.length_centimm);
            }
        }
        let mut spans = vec![stock_span];
        for &(start, end, _) in &exclusions {
            *work = work.saturating_add(spans.len());
            if *work > MAX_BLOCKS * 256 {
                return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT", "Превышен объём проверки препятствий", Some(original.id.clone())));
            }
            spans = subtract(spans, Span { start, end });
        }
        for span in spans {
            if span.len() <= 0 { continue; }
            *work = work.saturating_add(request.wall_volumes.len().saturating_mul(3)).saturating_add(1);
            if *work > MAX_BLOCKS * 256 {
                return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT", "Превышен объём проверки препятствий", Some(original.id.clone())));
            }
            let mut part = original.clone();
            let split = span.start != 0 || span.end != original.length_centimm;
            if split {
                crate::node_shapes::relocate_node_cuts(&mut part, span.start, span.end)?;
                part.id = format!("{}:part:{}-{}", original.id, span.start, span.end);
            }
            part.start = Point {
                x: original.start.x + (span.start as f64 * ux).round() as i64,
                y: original.start.y + (span.start as f64 * uy).round() as i64,
            };
            part.end = Point {
                x: original.start.x + (span.end as f64 * ux).round() as i64,
                y: original.start.y + (span.end as f64 * uy).round() as i64,
            };
            part.length_centimm = span.len();
            part.arms = original.arms.iter().filter_map(|arm| {
                let a = project(arm.start.x as f64 / 100.0, arm.start.y as f64 / 100.0).0 * 100.0;
                let b = project(arm.end.x as f64 / 100.0, arm.end.y as f64 / 100.0).0 * 100.0;
                let (from, to) = if (b - a).abs() < 1e-7 {
                    if a < span.start as f64 - 1e-7 || a > span.end as f64 + 1e-7 { return None; }
                    (0.0, 1.0)
                } else {
                    let left = (span.start as f64 - a) / (b - a);
                    let right = (span.end as f64 - a) / (b - a);
                    (left.min(right).max(0.0), left.max(right).min(1.0))
                };
                if to - from <= 1e-7 { return None; }
                let point = |t: f64| Point {
                    x: arm.start.x + ((arm.end.x - arm.start.x) as f64 * t).round() as i64,
                    y: arm.start.y + ((arm.end.y - arm.start.y) as f64 * t).round() as i64,
                };
                let mut clipped = arm.clone();
                clipped.start = point(from);
                clipped.end = point(to);
                clipped.length_centimm = (arm.length_centimm as f64 * (to - from)).round() as i64;
                (clipped.length_centimm > 0).then_some(clipped)
            }).collect();
            part.natural_end_left = original.natural_end_left && span.start == 0;
            part.natural_end_right = original.natural_end_right && span.end == original.length_centimm;
            let free_end = |point: Point| {
                let mut own_endpoints = 0;
                for wall in &request.wall_volumes {
                    let px = point.x as f64 / 100.0;
                    let py = point.y as f64 / 100.0;
                    let dx = wall.end_xmm - wall.start_xmm;
                    let dy = wall.end_ymm - wall.start_ymm;
                    let size = dx.hypot(dy);
                    if size <= 1e-7 { continue; }
                    let u = ((px - wall.start_xmm) * dx + (py - wall.start_ymm) * dy) / size;
                    let v = ((px - wall.start_xmm) * dy - (py - wall.start_ymm) * dx) / size;
                    if v.abs() > 0.02 || u < -0.02 || u > size + 0.02 { continue; }
                    let t = (u / size).clamp(0.0, 1.0);
                    let wall_bottom = wall.start_bottom_zmm + (wall.end_bottom_zmm - wall.start_bottom_zmm) * t;
                    let wall_top = wall.start_top_zmm + (wall.end_top_zmm - wall.start_top_zmm) * t;
                    if top <= wall_bottom + 1e-7 || bottom >= wall_top - 1e-7 { continue; }
                    if !owns_wall(&wall.guid) { return false; }
                    if u.abs() > 0.02 && (u - size).abs() > 0.02 { return false; }
                    own_endpoints += 1;
                }
                own_endpoints == 1
            };
            let obstacle_hide = original.source_ids.iter().any(|id| id.starts_with("beam:") || id.starts_with("opening:"))
                || original.cuts.iter().any(|cut| cut.starts_with("beam_volume:") || cut.starts_with("opening_volume:"));
            part.hide_spikes_left = !part.natural_end_left || free_end(part.start)
                || (!obstacle_hide && span.start == 0 && original.hide_spikes_left)
                || exclusions.iter().any(|&(_, end, _)| end == span.start);
            part.hide_spikes_right = !part.natural_end_right || free_end(part.end)
                || (!obstacle_hide && span.end == original.length_centimm && original.hide_spikes_right)
                || exclusions.iter().any(|&(start, _, _)| start == span.end);
            part.cuts.retain(|cut| !cut.starts_with("beam_volume:")
                && !cut.starts_with("opening_volume:") && cut != "left" && cut != "right"
                && !(cut.starts_with("arm:") && cut.ends_with(":distal")));
            // Причина открытого торца переживает разделение и экспорт. Короткая
            // длина сама по себе не разрешает использовать деталь как добор.
            part.obstacle_ends.retain(|end| if end.left { span.start == 0 }
                else { span.end == original.length_centimm });
            for (start, end, source) in &exclusions {
                for (left, adjacent) in [(true, *end == span.start), (false, *start == span.end)] {
                    if adjacent {
                        let reason = ObstacleEnd { left, source_id: source.clone() };
                        if !part.obstacle_ends.contains(&reason) { part.obstacle_ends.push(reason); }
                        if !part.source_ids.contains(source) { part.source_ids.push(source.clone()); }
                    }
                }
            }
            part.catalog_nominal_centimm = Some(part.length_centimm);
            if split && part.kind.starts_with("node_") && !part.cuts.iter().any(|cut| cut.starts_with("Type")) {
                part.kind = "ordinary".into();
                part.catalog_status = "length_only".into();
                part.local_origin = None;
                part.local_rotation_deg = None;
                part.arms.clear();
            }
            if !part.kind.starts_with("node_") { part.product_key = None; }
            if !original.components.is_empty() {
                let components = finalize_parts_inner(request, profile, &original.components, depth + 1, work, reveals)?;
                part.components = components.into_iter().filter(|child| {
                    let a = project(child.start.x as f64 / 100.0, child.start.y as f64 / 100.0).0 * 100.0;
                    let b = project(child.end.x as f64 / 100.0, child.end.y as f64 / 100.0).0 * 100.0;
                    a.min(b) < span.end as f64 && a.max(b) > span.start as f64
                }).collect();
            }
            result.push(part);
            if result.len() > MAX_BLOCKS {
                return Err(crate::api::ApiFailure::new("GEOMETRY_LIMIT", "Превышено число окончательных деталей", Some(original.id.clone())));
            }
        }
    }
    Ok(result)
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Layout {
    pub blocks: Vec<Block>,
}

/// Полная физическая геометрия кандидата и нарушения, запрещающие производственный выпуск.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CandidateCalculation {
    pub layout: Layout,
    pub diagnostics: Vec<Diagnostic>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Span {
    start: i64,
    end: i64,
}

impl Span {
    fn len(self) -> i64 {
        self.end - self.start
    }
    fn intersects(self, other: Self) -> bool {
        self.start < other.end && other.start < self.end
    }
}

fn diagnostic(
    code: DiagnosticCode,
    message: impl Into<String>,
    edge: Option<&CourseEdge>,
    course: Option<i64>,
) -> Diagnostic {
    Diagnostic {
        code,
        message: message.into(),
        wall_id: edge.map(|e| e.wall_id.clone()),
        edge_id: edge.map(|e| e.edge_id.clone()),
        course_index: course,
        occurrences: 1,
        source_ids: Vec::new(),
    }
}

fn aggregate_errors(errors: Vec<Diagnostic>) -> Vec<Diagnostic> {
    let mut groups: BTreeMap<(DiagnosticCode, String, Option<String>), Diagnostic> =
        BTreeMap::new();
    for item in errors {
        let key = (
            item.code.clone(),
            item.message.clone(),
            item.wall_id.clone(),
        );
        if let Some(previous) = groups.get_mut(&key) {
            previous.occurrences = previous.occurrences.saturating_add(item.occurrences);
            previous.source_ids.extend(item.source_ids);
            previous.source_ids.sort();
            previous.source_ids.dedup();
        } else {
            groups.insert(key, item);
        }
    }
    groups.into_values().collect()
}

fn edge_length(edge: &CourseEdge, tolerance: i64) -> Result<i64, Diagnostic> {
    let dx = i128::from(edge.end.x) - i128::from(edge.start.x);
    let dy = i128::from(edge.end.y) - i128::from(edge.start.y);
    let ax = dx.abs();
    let ay = dy.abs();
    if ax == 0 && ay == 0 {
        return Err(diagnostic(
            DiagnosticCode::UnsupportedGeometry,
            "Нулевое ребро",
            Some(edge),
            None,
        ));
    }
    if ax != 0 && ay != 0 && (ax - ay).abs() > i128::from(tolerance) {
        return Err(diagnostic(
            DiagnosticCode::UnsupportedGeometry,
            "Допустимы осевые и локальные 45° ребра",
            Some(edge),
            None,
        ));
    }
    let squared = dx
        .checked_mul(dx)
        .and_then(|v| dy.checked_mul(dy).and_then(|w| v.checked_add(w)))
        .ok_or_else(|| {
            diagnostic(
                DiagnosticCode::UnsupportedGeometry,
                "Переполнение длины",
                Some(edge),
                None,
            )
        })?;
    let estimate = (squared as f64).sqrt().round();
    if !estimate.is_finite() || estimate > i64::MAX as f64 {
        return Err(diagnostic(
            DiagnosticCode::UnsupportedGeometry,
            "Длина вне диапазона",
            Some(edge),
            None,
        ));
    }
    Ok(estimate as i64)
}

fn classify_candidate(
    topology: &Topology,
    constraints: &Constraints,
    course: &crate::domain::Course,
    edge: &CourseEdge,
    span: Span,
    profile: &Profile,
) -> Result<Exclusion, Diagnostic> {
    if constraints.evidence.is_empty() {
        return Ok(Exclusion::None);
    }
    let run = course
        .runs
        .iter()
        .find(|r| r.id == edge.edge_id)
        .ok_or_else(|| {
            diagnostic(
                DiagnosticCode::UnsupportedGeometry,
                "Нет run для проверки 3D-объёма",
                Some(edge),
                Some(course.index),
            )
        })?;
    let thickness = run
        .sources
        .iter()
        .filter_map(|s| {
            topology
                .walls
                .iter()
                .find(|w| w.id == s.wall_id)
                .map(|w| w.thickness)
        })
        .collect::<BTreeSet<_>>();
    if thickness.len() != 1 {
        return Err(diagnostic(
            DiagnosticCode::UnsupportedGeometry,
            "Невозможно определить единую толщину run для 3D-проверки",
            Some(edge),
            Some(course.index),
        ));
    }
    let Some(&width) = thickness.first() else {
        unreachable!()
    };
    let box_volume = SolidBox {
        u: Interval {
            start: span.start,
            end: span.end,
        },
        v: Interval {
            start: -width / 2,
            end: width - width / 2,
        },
        z: Interval {
            start: course.z,
            end: course.z + profile.index_centimm,
        },
    };
    let result = if !constraints.evidence.is_empty() || profile.full_course_clearance
        || profile.longitudinal_full_wall_thickness
        || profile.assembly_clearance_mm > 0.0
    {
        constraints.classify_assembly_box(&run.id, course.index, box_volume)
    } else {
        constraints.classify_box(&run.id, course.index, box_volume)
    };
    result.map_err(|e| {
        diagnostic(
            DiagnosticCode::UnsupportedGeometry,
            format!("3D-проверка {}: {}", e.source_id, e.message),
            Some(edge),
            Some(course.index),
        )
    })
}

fn node_masks(
    topology: &Topology,
    constraints: &Constraints,
    course: &crate::domain::Course,
    edges: &[CourseEdge],
    rays: &[NodeRayInput],
    nominals: Option<&BTreeMap<u16, i64>>,
    profile: &Profile,
) -> Result<Vec<crate::constraints::Mask>, Diagnostic> {
    let mut selected = Vec::new();
    for ray in rays {
        let Some(edge) = edges.iter().find(|e| e.edge_id == ray.run_id) else {
            continue;
        };
        let reach = nominals
            .and_then(|n| n.get(&ray.direction_deg))
            .copied()
            .unwrap_or(1)
            .min(ray.run_length_centimm);
        let arm = if ray.at_start {
            Span {
                start: 0,
                end: reach,
            }
        } else {
            Span {
                start: ray.run_length_centimm - reach,
                end: ray.run_length_centimm,
            }
        };
        for mask in constraints
            .masks
            .iter()
            .filter(|m| m.course_index == course.index && m.run_id == ray.run_id)
        {
            let part = Span {
                start: arm.start.max(mask.interval.start),
                end: arm.end.min(mask.interval.end),
            };
            if part.len() <= 0 {
                continue;
            }
            let mut copy = mask.clone();
            copy.interval = Interval {
                start: part.start,
                end: part.end,
            };
            if mask.coverage == MaskCoverage::PartialDepth && !constraints.evidence.is_empty() {
                match classify_candidate(topology, constraints, course, edge, part, profile)? {
                    Exclusion::None => continue,
                    Exclusion::FullVoid { .. } => copy.coverage = MaskCoverage::FullSectionVoid,
                    Exclusion::Partial { .. } => {}
                }
            }
            selected.push(copy);
        }
    }
    Ok(selected)
}

fn point_at(edge: &CourseEdge, offset: i64, length: i64) -> Point {
    if offset == 0 {
        return edge.start;
    }
    if offset == length {
        return edge.end;
    }
    let t = offset as f64 / length as f64;
    Point {
        x: (edge.start.x as f64 + (edge.end.x as f64 - edge.start.x as f64) * t).round() as i64,
        y: (edge.start.y as f64 + (edge.end.y as f64 - edge.start.y as f64) * t).round() as i64,
    }
}

fn subtract(solid: Vec<Span>, void: Span) -> Vec<Span> {
    let mut result = Vec::new();
    for span in solid {
        if !span.intersects(void) {
            result.push(span);
            continue;
        }
        if span.start < void.start {
            result.push(Span {
                start: span.start,
                end: void.start,
            });
        }
        if void.end < span.end {
            result.push(Span {
                start: void.end,
                end: span.end,
            });
        }
    }
    result
}

fn is_straight(directions: &[(i128, i128)]) -> bool {
    directions.len() == 2
        && directions[0].0 * directions[1].1 == directions[0].1 * directions[1].0
        && directions[0].0 * directions[1].0 + directions[0].1 * directions[1].1 < 0
}

fn direction_deg(v: (i128, i128)) -> u16 {
    match (v.0.signum(), v.1.signum()) {
        (1, 0) => 0,
        (1, 1) => 45,
        (0, 1) => 90,
        (-1, 1) => 135,
        (-1, 0) => 180,
        (-1, -1) => 225,
        (0, -1) => 270,
        (1, -1) => 315,
        _ => 999,
    }
}

#[derive(Clone)]
struct NodeChoice {
    kind: String,
    phase: i64,
    variant: Option<String>,
    rotation_deg: u16,
    geometry: NodeGeometry,
}

fn candidate_specs(
    profile: &Profile,
    wanted: &BTreeSet<u16>,
) -> Vec<(String, i64, Option<String>, u16, BTreeMap<u16, i64>)> {
    let mut result = Vec::new();
    for spec in &profile.nodes {
        if spec.rays.len() != wanted.len() {
            continue;
        }
        for rotation in (0..360).step_by(45) {
            let map: BTreeMap<u16, i64> = spec
                .rays
                .iter()
                .map(|r| {
                    (
                        ((usize::from(r.direction_deg) + rotation) % 360) as u16,
                        r.length_centimm,
                    )
                })
                .collect();
            if map.keys().copied().collect::<BTreeSet<_>>() == *wanted {
                let item = (
                    spec.kind.clone(),
                    spec.phase,
                    spec.variant.clone(),
                    rotation as u16,
                    map,
                );
                if !result.contains(&item) {
                    result.push(item);
                }
            }
        }
    }
    result.sort_by(|a, b| (&a.1, &a.0, &a.2, &a.3, &a.4).cmp(&(&b.1, &b.0, &b.2, &b.3, &b.4)));
    result
}

fn choice_arm(choice: &NodeChoice, run_id: &str) -> i64 {
    choice
        .geometry
        .arms
        .iter()
        .filter(|a| a.run_id == run_id)
        .filter_map(|a| a.active_interval.map(|v| v.end - v.start))
        .sum()
}

fn tilable_between(
    run: &crate::domain::WallRun,
    course_index: i64,
    constraints: &Constraints,
    profile: &Profile,
    left: i64,
    right: i64,
    phase: i64,
) -> bool {
    if left < 0 || right < 0 || left + right > run.length {
        return false;
    }
    let residue = ordinary_residue(run, course_index, profile);
    free_spans(run, course_index, constraints, left, right)
        .into_iter()
        .all(|s| choose_pieces_on_span(s.start, s.end, profile, phase, residue).is_some())
}

fn free_spans(
    run: &crate::domain::WallRun,
    course_index: i64,
    constraints: &Constraints,
    left: i64,
    right: i64,
) -> Vec<Span> {
    let mut solid = vec![Span {
        start: 0,
        end: run.length,
    }];
    for mask in constraints.masks.iter().filter(|m| {
        m.course_index == course_index
            && m.run_id == run.id
            && m.coverage == MaskCoverage::FullSectionVoid
    }) {
        solid = subtract(
            solid,
            Span {
                start: mask.interval.start,
                end: mask.interval.end,
            },
        );
    }
    for lintel in constraints
        .lintel_candidates
        .iter()
        .filter(|l| l.course_index == course_index && l.run_id == run.id)
    {
        if constraints.opening_axis(&lintel.opening_id).is_some() {
            continue;
        }
        if let (Some(a), Some(b)) = (lintel.left_support, lintel.right_support) {
            solid = subtract(
                solid,
                Span {
                    start: a.start,
                    end: b.end,
                },
            );
        }
    }
    solid
        .into_iter()
        .filter_map(|s| {
            let start = s.start.max(left);
            let end = s.end.min(run.length - right);
            (end > start).then_some(Span { start, end })
        })
        .collect()
}

fn grid_issue_for_choice(
    run: &crate::domain::WallRun,
    course_index: i64,
    constraints: &Constraints,
    profile: &Profile,
    left: i64,
    right: i64,
    phase: i64,
) -> Diagnostic {
    let residue = ordinary_residue(run, course_index, profile);
    let free = free_spans(run, course_index, constraints, left, right);
    let bad = free.iter().find(|span| {
        choose_pieces_on_span(span.start, span.end, profile, phase, residue).is_none()
    });
    let example = bad.and_then(|span| {
        residue.and_then(|value| {
            choose_pieces(span.len(), profile, phase)
                .and_then(|pieces| first_offgrid_joint(span.start, &pieces, value))
        })
    });
    let edge = CourseEdge {
        edge_id: run.id.clone(),
        wall_id: run.id.clone(),
        start: run.start,
        end: run.end,
    };
    let mut issue = diagnostic(DiagnosticCode::WorldGridIncompatible,
        format!("Несовместимые ordinary-русты на run {} длина {}, плечи left={} right={} phase={}, свободные интервалы {:?}, residue {:?}, пример недопустимого руста {:?}",
            run.id,run.length,left,right,phase,free,residue,example),
        Some(&edge),Some(course_index));
    issue.source_ids = run
        .sources
        .iter()
        .map(|s| format!("wall:{}", s.wall_id))
        .collect();
    issue
}

fn ordinary_residue(
    run: &crate::domain::WallRun,
    course_index: i64,
    profile: &Profile,
) -> Option<i64> {
    if profile.world_joint_policy.as_deref() != Some("affine_checkerboard_v1") {
        return None;
    }
    match joint_residue(course_index, run) {
        Ok(Some(residue)) => Some(residue),
        Ok(None) => Some((32_000_i64 * (course_index.rem_euclid(2) + 1)).rem_euclid(64_000)),
        Err(_) => None,
    }
}

fn choose_pieces_on_span(
    start: i64,
    end: i64,
    profile: &Profile,
    phase: i64,
    residue: Option<i64>,
) -> Option<Vec<(i64, bool)>> {
    choose_pieces_with_roof_ends(start, end, profile, phase, residue, false, false)
}

fn choose_pieces_with_roof_ends(
    start: i64,
    end: i64,
    profile: &Profile,
    phase: i64,
    residue: Option<i64>,
    roof_start: bool,
    roof_end: bool,
) -> Option<Vec<(i64, bool)>> {
    if end <= start {
        return Some(Vec::new());
    }
    let Some(residue) = residue else {
        return choose_pieces(end - start, profile, phase);
    };
    if let Some(special_limit) = profile.maximum_special_blank_centimm {
        let mut result: Vec<(i64, bool)> = Vec::new();
        let mut at = start;
        while at < end {
            let remaining = end - at;
            let mut distance = (residue - at).rem_euclid(profile.ordinary_length_centimm);
            if distance == 0 {
                distance = profile.ordinary_length_centimm;
            }
            let piece = distance.min(remaining);
            if piece > special_limit {
                return None;
            }
            // Сращиваются только два коротких фрагмента. Соседняя длинная
            // деталь сохраняет руст; на фронтоне сращивание запрещено.
            if !roof_start && !roof_end && piece < profile.minimum_cut_centimm
                && result.last().is_some_and(|last| last.0 < profile.minimum_cut_centimm)
            {
                if let Some(last) = result.last_mut() {
                    last.0 += piece;
                    last.1 = !profile.ordinary_nominal_lengths_centimm.contains(&last.0);
                }
            } else {
                result.push((piece, !profile.ordinary_nominal_lengths_centimm.contains(&piece)));
            }
            at += piece;
            if result.len() >= MAX_BLOCKS {
                return None;
            }
        }
        if result.iter().filter(|(_, cut)| *cut).count() > profile.max_ordinary_cuts_per_free_span {
            return None;
        }
        return Some(result);
    }
    let blank = profile
        .maximum_ordinary_blank_centimm
        .unwrap_or(profile.ordinary_length_centimm);
    let mut cursor = start;
    let mut pieces = Vec::new();
    let mut cuts = 0;
    while cursor < end {
        if pieces.len() >= MAX_BLOCKS {
            return None;
        }
        let remaining = end - cursor;
        let piece = if remaining <= blank {
            remaining
        } else {
            let mut distance = (residue - cursor).rem_euclid(64_000);
            if distance == 0 {
                distance = 64_000;
            }
            if distance < profile.minimum_cut_centimm {
                distance += 64_000;
            }
            if distance > blank {
                return None;
            }
            distance
        };
        let nominal = profile.ordinary_nominal_lengths_centimm.contains(&piece);
        let cut = !nominal;
        if cut && (piece < profile.minimum_cut_centimm || piece > blank) {
            return None;
        }
        cuts += usize::from(cut);
        if cuts > profile.max_ordinary_cuts_per_free_span {
            return None;
        }
        pieces.push((piece, cut));
        cursor += piece;
    }
    Some(pieces)
}

/// Наклонный верх обрывает run внутри исходной стены в текущем венце.
/// Настоящий торец стены и нижние венцы сохраняют обычные правила подрезки.
fn roof_clips_end(
    topology: &Topology,
    course: &crate::domain::Course,
    run: &crate::domain::WallRun,
    point: Point,
    profile: &Profile,
) -> bool {
    if point != run.start && point != run.end {
        return false;
    }
    run.sources.iter().any(|source| {
        let Some(wall) = topology.walls.iter().find(|w| w.id == source.wall_id) else {
            return false;
        };
        if wall.top_start == wall.top_end {
            return false;
        }
        let dx = (i128::from(wall.end.x) - i128::from(wall.start.x)) as f64;
        let dy = (i128::from(wall.end.y) - i128::from(wall.start.y)) as f64;
        let axis_squared = dx * dx + dy * dy;
        if axis_squared <= 0.0 {
            return false;
        }
        let t = ((i128::from(point.x) - i128::from(wall.start.x)) as f64 * dx
            + (i128::from(point.y) - i128::from(wall.start.y)) as f64 * dy)
            / axis_squared;
        if !(0.0 < t && t < 1.0) {
            return false;
        }
        let top = wall.top_start as f64
            + (i128::from(wall.top_end) - i128::from(wall.top_start)) as f64 * t;
        let tolerance = profile.coordinate_tolerance_centimm as f64 + 1e-7;
        top >= course.z as f64 - tolerance && top < (course.z + profile.index_centimm) as f64 - 1e-7
    })
}

fn first_offgrid_joint(start: i64, pieces: &[(i64, bool)], residue: i64) -> Option<i64> {
    let mut joint = start;
    for &(length, _) in pieces.iter().take(pieces.len().saturating_sub(1)) {
        joint += length;
        if joint.rem_euclid(64_000) != residue {
            return Some(joint);
        }
    }
    None
}

fn node_var(course: i64, point: Point) -> String {
    format!("c{course}:v:{}:{}", point.x, point.y)
}

fn irreducible_conflict(
    problem: &Problem,
    component: &choice::ConflictComponent,
) -> Option<Problem> {
    if component.unary_constraint_indices.len() + component.binary_constraint_indices.len() > 128 {
        return None;
    }
    let ids: BTreeSet<&str> = component.variables.iter().map(String::as_str).collect();
    let mut core = Problem {
        variables: problem
            .variables
            .iter()
            .filter(|v| ids.contains(v.id.as_str()))
            .cloned()
            .collect(),
        unary: component
            .unary_constraint_indices
            .iter()
            .map(|&i| problem.unary[i].clone())
            .collect(),
        binary: component
            .binary_constraint_indices
            .iter()
            .map(|&i| problem.binary[i].clone())
            .collect(),
        max_states: problem.max_states,
    };
    if choice::solve(&core) != Err(SolveError::Unsat) {
        return None;
    }
    let mut index = core.binary.len();
    while index > 0 {
        index -= 1;
        let mut smaller = core.clone();
        smaller.binary.remove(index);
        if choice::solve(&smaller) == Err(SolveError::Unsat) {
            core = smaller;
        }
    }
    let mut index = core.unary.len();
    while index > 0 {
        index -= 1;
        let mut smaller = core.clone();
        smaller.unary.remove(index);
        if choice::solve(&smaller) == Err(SolveError::Unsat) {
            core = smaller;
        }
    }
    Some(core)
}

fn node_joint_evidence(
    choice: &NodeChoice,
    point: Point,
    course: &crate::domain::Course,
    constraints: &Constraints,
    profile: &Profile,
) -> Result<JointEvidence, String> {
    let missing = || "Нет подтверждённого физического каталога узла".to_string();
    if profile.node_assembly_family.as_deref() != Some("forestbrick_legacy_observed") {
        return Err(missing());
    }
    let kind = match choice.kind.as_str() {
        "L" => NodeKind::L,
        "T" => NodeKind::T,
        "X" => NodeKind::X,
        "Y" => NodeKind::Y,
        _ => return Err(missing()),
    };
    let phase = u8::try_from(choice.phase).map_err(|_| missing())?;
    let sources = course
        .runs
        .iter()
        .map(|run| {
            (
                run.id.clone(),
                run.sources
                    .iter()
                    .map(|s| format!("wall:{}", s.wall_id))
                    .collect(),
            )
        })
        .collect();
    let parts = assemble_node(
        kind,
        phase,
        choice.rotation_deg,
        point,
        &choice.geometry.arms,
        &sources,
    )
    .map_err(|error| {
        format!(
            "Не собран физический узел ({},{}): {error:?}; плечи {:?}",
            point.x,
            point.y,
            choice
                .geometry
                .arms
                .iter()
                .map(|a| (
                    &a.run_id,
                    a.nominal_length_centimm,
                    a.active_interval,
                    a.cut_at_distal_end,
                    &a.cut_source_ids
                ))
                .collect::<Vec<_>>()
        )
    })?;
    let mut evidence = vertical::extract_node_evidence(point, &parts, &course.runs)
        .map_err(|error| format!("Не извлечены швы узла: {error:?}"))?;
    // На distal-конце узла шов существует лишь при соседнем solid.
    // Короткий сегмент представляет только факт контакта с другой деталью;
    // окончательная проверка ниже использует опубликованные ordinary целиком.
    for arm in &choice.geometry.arms {
        let Some(interval) = arm.active_interval else {
            continue;
        };
        let Some(run) = course.runs.iter().find(|run| run.id == arm.run_id) else {
            continue;
        };
        let (left, right) = if arm.at_start {
            (interval.end, 0)
        } else {
            (0, run.length - interval.start)
        };
        for span in free_spans(run, course.index, constraints, left, right) {
            let touching = if arm.at_start && span.start == interval.end {
                Some(Interval {
                    start: span.start,
                    end: (span.start + 1).min(span.end),
                })
            } else if !arm.at_start && span.end == interval.start {
                Some(Interval {
                    start: (span.end - 1).max(span.start),
                    end: span.end,
                })
            } else {
                None
            };
            let Some(touching) = touching else { continue };
            let edge = CourseEdge {
                edge_id: run.id.clone(),
                wall_id: run.id.clone(),
                start: run.start,
                end: run.end,
            };
            evidence.segments.push(
                vertical::material_segment(
                    run,
                    point_at(&edge, touching.start, run.length),
                    point_at(&edge, touching.end, run.length),
                    parts.len() + evidence.segments.len(),
                    &[run.id.clone()],
                )
                .map_err(|error| format!("Не доказан node→ordinary контакт: {error:?}"))?,
            );
        }
    }
    vertical::extract_segments_evidence(evidence.segments)
        .map_err(|error| format!("Не извлечены междетальные швы: {error:?}"))
}

fn vertical_diagnostic(verdict: VerticalVerdict, course: i64) -> Option<Diagnostic> {
    match verdict {
        VerticalVerdict::Clear => None,
        VerticalVerdict::Conflict {
            lower_at,
            upper_at,
            lower_source_ids,
            upper_source_ids,
            ..
        } => {
            let mut issue = diagnostic(DiagnosticCode::VerticalJointConflict,
                format!("Междетальные швы соседних венцов {} и {} совпадают или недостаточно смещены: ({},{}) / ({},{})",
                    course, course + 1, lower_at.x, lower_at.y, upper_at.x, upper_at.y), None, Some(course));
            issue.source_ids = lower_source_ids
                .into_iter()
                .chain(upper_source_ids)
                .collect();
            issue.source_ids.sort();
            issue.source_ids.dedup();
            Some(issue)
        }
        VerticalVerdict::MissingEvidence { reason } => Some(diagnostic(
            DiagnosticCode::NodeVerticalRuleMissing,
            reason,
            None,
            Some(course),
        )),
    }
}

fn validate_vertical_plan(
    topology: &Topology,
    blocks: &[Block],
    profile: &Profile,
) -> Vec<Diagnostic> {
    if profile.node_assembly_family.as_deref() != Some("forestbrick_legacy_observed") {
        return Vec::new();
    }
    let mut evidence = BTreeMap::new();
    let mut errors = Vec::new();
    for course in &topology.courses {
        let mut segments = Vec::new();
        for (part_index, block) in blocks
            .iter()
            .enumerate()
            .filter(|(_, b)| b.course_index == course.index)
        {
            let arms: Vec<_> = if block.arms.is_empty() {
                vec![(block.edge_id.as_str(), block.start, block.end)]
            } else {
                block
                    .arms
                    .iter()
                    .map(|a| (a.edge_id.as_str(), a.start, a.end))
                    .collect()
            };
            for (run_id, start, end) in arms {
                let Some(run) = course.runs.iter().find(|r| r.id == run_id) else {
                    errors.push(diagnostic(
                        DiagnosticCode::NodeVerticalRuleMissing,
                        format!("Нет run {run_id} для физической детали {}", block.id),
                        None,
                        Some(course.index),
                    ));
                    continue;
                };
                match vertical::material_segment(run, start, end, part_index, &block.source_ids) {
                    Ok(segment) => segments.push(segment),
                    Err(VerticalError::MissingEvidence { reason, .. }) => errors.push(diagnostic(
                        DiagnosticCode::NodeVerticalRuleMissing,
                        reason,
                        None,
                        Some(course.index),
                    )),
                }
            }
        }
        if segments.is_empty() {
            continue;
        }
        match vertical::extract_segments_evidence(segments) {
            Ok(value) => {
                evidence.insert(course.index, value);
            }
            Err(VerticalError::MissingEvidence { reason, .. }) => errors.push(diagnostic(
                DiagnosticCode::NodeVerticalRuleMissing,
                reason,
                None,
                Some(course.index),
            )),
        }
    }
    for (&index, lower) in &evidence {
        let Some(upper_index) = index.checked_add(1) else {
            continue;
        };
        let Some(upper) = evidence.get(&upper_index) else {
            continue;
        };
        let verdict = vertical::check_adjacent_plans(
            lower,
            upper,
            profile.coordinate_tolerance_centimm.max(1),
        );
        if let Some(issue) = vertical_diagnostic(verdict, index) {
            errors.push(issue);
        }
    }
    errors
}

fn select_nodes(
    topology: &Topology,
    constraints: &Constraints,
    profile: &Profile,
) -> Result<BTreeMap<(i64, Point), NodeChoice>, Vec<Diagnostic>> {
    let mut choices = BTreeMap::<String, NodeChoice>::new();
    let mut domains = BTreeMap::<(i64, Point), Vec<String>>::new();
    let mut errors = Vec::new();
    for course in &topology.courses {
        let edges: Vec<CourseEdge> = course
            .runs
            .iter()
            .map(|r| CourseEdge {
                edge_id: r.id.clone(),
                wall_id: r.id.clone(),
                start: r.start,
                end: r.end,
            })
            .collect();
        let mut incident: BTreeMap<Point, Vec<(usize, bool, (i128, i128))>> = BTreeMap::new();
        for (i, e) in edges.iter().enumerate() {
            let dx = i128::from(e.end.x) - i128::from(e.start.x);
            let dy = i128::from(e.end.y) - i128::from(e.start.y);
            incident
                .entry(e.start)
                .or_default()
                .push((i, true, (dx, dy)));
            incident
                .entry(e.end)
                .or_default()
                .push((i, false, (-dx, -dy)));
        }
        for (point, entries) in incident {
            if entries.len() < 2 {
                continue;
            }
            let dirs: Vec<_> = entries.iter().map(|e| e.2).collect();
            if is_straight(&dirs) {
                continue;
            }
            let rays: Vec<NodeRayInput> = entries
                .iter()
                .map(|(i, at_start, dir)| NodeRayInput {
                    run_id: edges[*i].edge_id.clone(),
                    direction_deg: direction_deg(*dir),
                    at_start: *at_start,
                    run_length_centimm: course
                        .runs
                        .iter()
                        .find(|r| r.id == edges[*i].edge_id)
                        .map(|r| r.length)
                        .unwrap_or(0),
                })
                .collect();
            let pre = match node_masks(topology, constraints, course, &edges, &rays, None, profile)
            {
                Ok(v) => v,
                Err(e) => {
                    errors.push(e);
                    continue;
                }
            };
            let wanted = match active_rays(course.index, &rays, &pre) {
                Ok(v) => v,
                Err(e) => {
                    errors.push(node_error(e, &edges, course.index));
                    continue;
                }
            };
            if wanted.len() <= 1
                || (wanted.len() == 2
                    && wanted
                        .iter()
                        .next()
                        .is_some_and(|a| wanted.contains(&((a + 180) % 360))))
            {
                continue;
            }
            let mut domain = Vec::new();
            let mut first_error = None;
            let mut candidate_rejections = BTreeSet::new();
            for (option_index, (kind, phase, variant, rotation, map)) in
                candidate_specs(profile, &wanted).into_iter().enumerate()
            {
                let mut arms = Vec::new();
                let mut invalid = false;
                let short_stem = if variant.as_deref() == Some("t0_short") {
                    map.keys()
                        .copied()
                        .find(|direction| !map.contains_key(&((direction + 180) % 360)))
                } else {
                    None
                };
                if variant.as_deref() == Some("t0_short")
                    && short_stem.and_then(|direction| map.get(&direction).copied()) != Some(32_000)
                {
                    candidate_rejections
                        .insert(format!("{kind}/p{phase}/r{rotation}/t0_short: нет stem320"));
                    continue;
                }
                for ray in &rays {
                    if !wanted.contains(&ray.direction_deg) {
                        continue;
                    }
                    let length = map.get(&ray.direction_deg).copied().unwrap_or(0);
                    if short_stem == Some(ray.direction_deg) && ray.run_length_centimm != 32_000 {
                        candidate_rejections.insert(format!(
                            "{kind}/p{phase}/r{rotation}/t0_short: stem run {} длина {}, требуется ровно 32000",
                            ray.run_id,ray.run_length_centimm));
                        invalid = true;
                        break;
                    }
                    let wall_distal_trim = profile.node_assembly_family.as_deref()
                        == Some("forestbrick_legacy_observed")
                        && length == 64_000
                        && ray.run_length_centimm >= 32_000;
                    if length <= 0 || (length > ray.run_length_centimm && !wall_distal_trim) {
                        candidate_rejections.insert(format!(
                            "{kind}/p{phase}/r{rotation}: луч {} длина {} вне run {} длина {}",
                            ray.direction_deg, length, ray.run_id, ray.run_length_centimm
                        ));
                        invalid = true;
                        break;
                    }
                    arms.push(NodeArmInput {
                        ray: ray.clone(),
                        nominal_length_centimm: length.min(ray.run_length_centimm),
                    });
                }
                if invalid {
                    continue;
                }
                let masks = match node_masks(
                    topology,
                    constraints,
                    course,
                    &edges,
                    &rays,
                    Some(&map),
                    profile,
                ) {
                    Ok(v) => v,
                    Err(e) => {
                        first_error.get_or_insert(e);
                        continue;
                    }
                };
                let mut geometry = match resolve_node(course.index, &arms, &masks) {
                    Ok(v) => v,
                    Err(e) => {
                        first_error.get_or_insert(node_error(e, &edges, course.index));
                        continue;
                    }
                };
                for resolved in &mut geometry.arms {
                    let Some(&nominal) = map.get(&resolved.direction_deg) else {
                        continue;
                    };
                    let Some(ray) = rays.iter().find(|ray| ray.run_id == resolved.run_id) else {
                        continue;
                    };
                    if nominal > ray.run_length_centimm {
                        resolved.nominal_length_centimm = nominal;
                        resolved.cut_at_distal_end = resolved.active_interval.is_some();
                        let source = format!("wall-boundary:{}", resolved.run_id);
                        resolved.cut_source_ids.push(source.clone());
                        geometry.source_ids.push(source);
                        geometry
                            .cuts
                            .push(format!("arm:{}:distal", resolved.run_id));
                    }
                }
                geometry.source_ids.sort();
                geometry.source_ids.dedup();
                let id = format!(
                    "{}:o{}:p{}:r{}:{kind}",
                    node_var(course.index, point),
                    option_index,
                    phase,
                    rotation
                );
                let candidate = NodeChoice {
                    kind,
                    phase,
                    variant,
                    rotation_deg: rotation,
                    geometry,
                };
                if profile.node_assembly_family.as_deref() == Some("forestbrick_legacy_observed") {
                    if let Err(reason) =
                        node_joint_evidence(&candidate, point, course, constraints, profile)
                    {
                        let mut issue = diagnostic(
                            DiagnosticCode::UnsupportedNodeCut,
                            reason,
                            None,
                            Some(course.index),
                        );
                        issue.source_ids = candidate.geometry.source_ids.clone();
                        first_error.get_or_insert(issue);
                        continue;
                    }
                }
                choices.insert(id.clone(), candidate);
                domain.push(id);
            }
            if domain.is_empty() {
                errors.push(first_error.unwrap_or_else(|| {
                    diagnostic(
                        DiagnosticCode::UnsupportedCatalog,
                        format!(
                            "Нет допустимого каталожного паттерна узла ({},{}) для лучей {:?}; причины: {:?}",
                            point.x, point.y, wanted, candidate_rejections
                        ),
                        None,
                        Some(course.index),
                    )
                }));
            } else {
                domains.insert((course.index, point), domain);
            }
        }
    }
    if !errors.is_empty() {
        return Err(aggregate_errors(errors));
    }
    let mut problem = Problem {
        variables: domains
            .iter()
            .map(|(&(course, point), ids)| Variable {
                id: node_var(course, point),
                values: ids.clone(),
            })
            .collect(),
        unary: Vec::new(),
        binary: Vec::new(),
        max_states: MAX_SEARCH_STATES,
    };
    let mut without_grid = profile.clone();
    without_grid.world_joint_policy = None;
    let mut grid_blocker = None;
    let mut unary_witness = Vec::<String>::new();
    let mut unary_short = Vec::<Option<Diagnostic>>::new();
    for course in &topology.courses {
        for run in &course.runs {
            let left = domains.get(&(course.index, run.start));
            let right = domains.get(&(course.index, run.end));
            match (left, right) {
                (Some(a), Some(b)) => {
                    let mut allowed = Vec::new();
                    for x in a {
                        for y in b {
                            let lx = choice_arm(&choices[x], &run.id);
                            let ry = choice_arm(&choices[y], &run.id);
                            if tilable_between(
                                run,
                                course.index,
                                constraints,
                                profile,
                                lx,
                                ry,
                                choices[x].phase,
                            ) {
                                allowed.push((x.clone(), y.clone()));
                            }
                        }
                    }
                    if allowed.is_empty() && profile.world_joint_policy.is_some() {
                        for x in a {
                            for y in b {
                                let lx = choice_arm(&choices[x], &run.id);
                                let ry = choice_arm(&choices[y], &run.id);
                                if tilable_between(
                                    run,
                                    course.index,
                                    constraints,
                                    &without_grid,
                                    lx,
                                    ry,
                                    choices[x].phase,
                                ) {
                                    grid_blocker.get_or_insert_with(|| {
                                        grid_issue_for_choice(
                                            run,
                                            course.index,
                                            constraints,
                                            profile,
                                            lx,
                                            ry,
                                            choices[x].phase,
                                        )
                                    });
                                    break;
                                }
                            }
                        }
                    }
                    problem.binary.push(BinaryConstraint {
                        left: node_var(course.index, run.start),
                        right: node_var(course.index, run.end),
                        allowed,
                    });
                }
                (Some(a), None) | (None, Some(a)) => {
                    let at_start = left.is_some();
                    let allowed = a
                        .iter()
                        .filter(|id| {
                            let arm = choice_arm(&choices[*id], &run.id);
                            tilable_between(
                                run,
                                course.index,
                                constraints,
                                profile,
                                if at_start { arm } else { 0 },
                                if at_start { 0 } else { arm },
                                choices[*id].phase,
                            )
                        })
                        .cloned()
                        .collect::<Vec<_>>();
                    if allowed.is_empty() && profile.world_joint_policy.is_some() {
                        for id in a {
                            let arm = choice_arm(&choices[id], &run.id);
                            let lx = if at_start { arm } else { 0 };
                            let ry = if at_start { 0 } else { arm };
                            if tilable_between(
                                run,
                                course.index,
                                constraints,
                                &without_grid,
                                lx,
                                ry,
                                choices[id].phase,
                            ) {
                                grid_blocker.get_or_insert_with(|| {
                                    grid_issue_for_choice(
                                        run,
                                        course.index,
                                        constraints,
                                        profile,
                                        lx,
                                        ry,
                                        choices[id].phase,
                                    )
                                });
                                break;
                            }
                        }
                    }
                    let witness = if allowed.is_empty() {
                        let candidates = a
                            .iter()
                            .map(|id| {
                                let arm = choice_arm(&choices[id], &run.id);
                                let lx = if at_start { arm } else { 0 };
                                let ry = if at_start { 0 } else { arm };
                                format!(
                                    "{} p{} {:?} grid={} no-grid={}",
                                    id,
                                    choices[id].phase,
                                    free_spans(run, course.index, constraints, lx, ry),
                                    tilable_between(
                                        run,
                                        course.index,
                                        constraints,
                                        profile,
                                        lx,
                                        ry,
                                        choices[id].phase
                                    ),
                                    tilable_between(
                                        run,
                                        course.index,
                                        constraints,
                                        &without_grid,
                                        lx,
                                        ry,
                                        choices[id].phase
                                    )
                                )
                            })
                            .collect::<Vec<_>>();
                        let masks = constraints
                            .masks
                            .iter()
                            .filter(|mask| {
                                mask.course_index == course.index && mask.run_id == run.id
                            })
                            .map(|mask| {
                                format!(
                                    "{} [{}, {}) {:?}",
                                    mask_source(&mask.source),
                                    mask.interval.start,
                                    mask.interval.end,
                                    mask.coverage
                                )
                            })
                            .collect::<Vec<_>>();
                        format!(
                            "run {} length={} at_start={} residue={:?} candidates={:?} masks={:?}",
                            run.id,
                            run.length,
                            at_start,
                            ordinary_residue(run, course.index, profile),
                            candidates,
                            masks
                        )
                    } else {
                        String::new()
                    };
                    unary_witness.push(witness);
                    let short = if allowed.is_empty() {
                        let tiny = a
                            .iter()
                            .map(|id| {
                                let arm = choice_arm(&choices[id], &run.id);
                                let lx = if at_start { arm } else { 0 };
                                let ry = if at_start { 0 } else { arm };
                                free_spans(run, course.index, constraints, lx, ry)
                                    .into_iter()
                                    .find(|span| span.len() < profile.minimum_cut_centimm)
                            })
                            .collect::<Vec<_>>();
                        if !tiny.is_empty() && tiny.iter().all(Option::is_some) {
                            let span = tiny[0].unwrap();
                            let edge = CourseEdge {
                                edge_id: run.id.clone(),
                                wall_id: run.id.clone(),
                                start: run.start,
                                end: run.end,
                            };
                            let mut issue=diagnostic(DiagnosticCode::MinimumOrdinarySpan,
                                format!("Свободный solid [{}, {}) длиной {} на run {} меньше минимальной детали {}",
                                    span.start,span.end,span.len(),run.id,profile.minimum_cut_centimm),
                                Some(&edge),Some(course.index));
                            issue.source_ids = run
                                .sources
                                .iter()
                                .map(|s| format!("wall:{}", s.wall_id))
                                .chain(
                                    constraints
                                        .masks
                                        .iter()
                                        .filter(|mask| {
                                            mask.course_index == course.index
                                                && mask.run_id == run.id
                                                && (mask.interval.end == span.start
                                                    || mask.interval.start == span.end)
                                        })
                                        .map(|mask| mask_source(&mask.source)),
                                )
                                .collect();
                            issue.source_ids.sort();
                            issue.source_ids.dedup();
                            Some(issue)
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    unary_short.push(short);
                    problem.unary.push(UnaryConstraint {
                        variable: node_var(
                            course.index,
                            if at_start { run.start } else { run.end },
                        ),
                        allowed,
                    });
                }
                _ => {}
            }
        }
    }
    let horizontal_binary_count = problem.binary.len();
    let horizontal_unary_count = problem.unary.len();
    // Не навязываем вертикальную фазу варианту, уже исключённому покрытием,
    // масками или горизонтальной связью. AC3 не расходует бюджет перебора.
    let horizontal_domains = choice::viable_domains(&problem).ok();
    if profile.world_joint_policy.as_deref() == Some("affine_checkerboard_v1") {
        if let Some(horizontal_domains) = &horizontal_domains {
            for (&(course_index, point), _) in &domains {
                // На каталожной полусетке фазу определяют реальные швы.
                // Некратные точки требуют обычных компенсаторов, а не сдвига сетки.
                if point.x.rem_euclid(32_000) != 0 || point.y.rem_euclid(32_000) != 0 {
                    continue;
                }
                let Some(course) = topology.courses.iter().find(|c| c.index == course_index) else {
                    continue;
                };
                let variable = node_var(course_index, point);
                let Some(ids) = horizontal_domains.get(&variable) else {
                    continue;
                };
                let allowed: Vec<_> = ids
                    .iter()
                    .filter(|id| {
                        node_joint_evidence(&choices[*id], point, course, constraints, profile)
                            .is_ok_and(|evidence| {
                                evidence.joints.iter().all(|joint| {
                                    let direction = match (joint.axis.dx, joint.axis.dy) {
                                        (1, 0) => 0,
                                        (0, 1) => 90,
                                        (1, 1) => 45,
                                        (1, -1) => 315,
                                        _ => return false,
                                    };
                                    crate::grid::is_world_joint(course_index, joint.at, direction)
                                        .is_ok_and(|on_grid| on_grid != Some(false))
                                })
                            })
                    })
                    .cloned()
                    .collect();
                // Отсутствующая каталожная фаза не запрещает физический кандидат.
                if !allowed.is_empty() && allowed.len() < ids.len() {
                    problem.unary.push(UnaryConstraint { variable, allowed });
                    unary_short.push(None);
                    unary_witness.push("Мировая фаза реальных междетальных швов узла".into());
                }
            }
        }
    }
    // После выбора доступной мировой фазы пересчитываем поддержку значений:
    // несовместимая обязательная пара не должна обнулить независимые связи.
    let horizontal_domains = choice::viable_domains(&problem).ok();
    let mut vertical_evidence_complete = true;
    // Ограничения относятся к реальным швам разных физических деталей.
    // Плечи coalesced Type6/Type7_1 не создают шов в вершине.
    for (&(course_index, point), _) in &domains {
        let Some(horizontal_domains) = &horizontal_domains else {
            break;
        };
        let Some(upper_index) = course_index.checked_add(1) else {
            continue;
        };
        let Some(lower_ids) = horizontal_domains.get(&node_var(course_index, point)) else {
            continue;
        };
        let Some(upper_ids) = horizontal_domains.get(&node_var(upper_index, point)) else {
            continue;
        };
        let Some(lower_course) = topology.courses.iter().find(|c| c.index == course_index) else {
            continue;
        };
        let Some(upper_course) = topology.courses.iter().find(|c| c.index == upper_index) else {
            continue;
        };
        let lower_evidence: Vec<_> = lower_ids
            .iter()
            .map(|id| {
                (
                    id,
                    node_joint_evidence(&choices[id], point, lower_course, constraints, profile),
                )
            })
            .collect();
        let upper_evidence: Vec<_> = upper_ids
            .iter()
            .map(|id| {
                (
                    id,
                    node_joint_evidence(&choices[id], point, upper_course, constraints, profile),
                )
            })
            .collect();
        let mut allowed = Vec::new();
        let mut first_issue = None;
        for (lower_id, below) in &lower_evidence {
            for (upper_id, above) in &upper_evidence {
                let issue = match (below, above) {
                    (Ok(below), Ok(above)) => vertical_diagnostic(
                        vertical::check_adjacent_nodes(
                            below,
                            above,
                            profile.coordinate_tolerance_centimm.max(1),
                        ),
                        course_index,
                    ),
                    (Err(reason), _) | (_, Err(reason)) => Some(diagnostic(
                        DiagnosticCode::NodeVerticalRuleMissing,
                        reason.clone(),
                        None,
                        Some(course_index),
                    )),
                };
                if let Some(mut issue) = issue {
                    if issue.code == DiagnosticCode::NodeVerticalRuleMissing {
                        vertical_evidence_complete = false;
                        issue.message = format!(
                            "{}: {} p{} {:?} / {} p{} {:?}",
                            issue.message,
                            choices[*lower_id].kind,
                            choices[*lower_id].phase,
                            choices[*lower_id].variant,
                            choices[*upper_id].kind,
                            choices[*upper_id].phase,
                            choices[*upper_id].variant
                        );
                    }
                    if issue.source_ids.is_empty() {
                        issue.source_ids = choices[*lower_id]
                            .geometry
                            .arms
                            .iter()
                            .chain(&choices[*upper_id].geometry.arms)
                            .map(|arm| arm.run_id.clone())
                            .collect();
                        issue.source_ids.sort();
                        issue.source_ids.dedup();
                    }
                    first_issue.get_or_insert(issue);
                } else {
                    allowed.push(((*lower_id).clone(), (*upper_id).clone()));
                }
            }
        }
        if allowed.is_empty() {
            if let Some(mut issue) = first_issue {
                if issue.code == DiagnosticCode::NodeVerticalRuleMissing {
                    issue.message = format!("{}; узел ({},{})", issue.message, point.x, point.y);
                    return Err(vec![issue]);
                }
            }
            // Все структурно допустимые пары имеют доказанную коллизию.
            // Остальные вертикальные связи сохраняются; финальный план
            // обязательно отдаст диагностику этого вынужденного шва.
            continue;
        }
        problem.binary.push(BinaryConstraint {
            left: node_var(course_index, point),
            right: node_var(upper_index, point),
            allowed,
        });
    }
    let constrained = choice::solve(&problem);
    let structural = match constrained {
        Err(SolveError::Unsat)
            if vertical_evidence_complete
                && (problem.binary.len() > horizontal_binary_count
                    || problem.unary.len() > horizontal_unary_count) =>
        {
            // Кандидат сохраняет тот же CSP и все ограничения покрытия/масок.
            // Только перевязка становится диагностикой полного физического плана.
            problem.binary.truncate(horizontal_binary_count);
            match choice::solve(&problem) {
                Err(SolveError::Unsat) if problem.unary.len() > horizontal_unary_count => {
                    problem.unary.truncate(horizontal_unary_count);
                    choice::solve(&problem)
                }
                result => result,
            }
        }
        result => result,
    };
    let solution = structural.map_err(|e| {
        vec![match e {
            SolveError::Invalid(message) => {
                diagnostic(DiagnosticCode::UnsupportedGeometry, message, None, None)
            }
            SolveError::Unsat => {
                let short=choice::first_unsat_component(&problem).ok().flatten()
                    .and_then(|component|component.unary_constraint_indices.iter()
                        .find_map(|&index|unary_short[index].clone()));
                short.or_else(||grid_blocker.clone()).unwrap_or_else(|| {
                let component = match choice::first_unsat_component(&problem) {
                    Ok(Some(component)) => {
                        let core=irreducible_conflict(&problem,&component);
                        let witness = core.as_ref().map(|core| core.variables.iter()
                            .filter(|v| core.unary.iter().any(|u| u.variable == v.id)
                                || core.binary.iter().any(|b| b.left == v.id || b.right == v.id))
                            .take(4).map(|v| (&v.id, v.values.iter().take(6).map(|id| {
                                let c = &choices[id];
                                format!("{} {} p{} r{} arms={:?}", id, c.kind, c.phase, c.rotation_deg,
                                    c.geometry.arms.iter().map(|a| (&a.run_id, a.active_interval)).collect::<Vec<_>>())
                            }).collect::<Vec<_>>())).collect::<Vec<_>>());
                        let empty_witness=component.unary_constraint_indices.iter()
                            .filter(|&&index|problem.unary[index].allowed.is_empty())
                            .map(|&index|unary_witness[index].as_str()).take(2).collect::<Vec<_>>();
                        format!("первая несовместимая компонента: {} узлов, {} unary, {} run-связей; неустранимое ядро: {:?}; пустые unary: {:?}; варианты ядра: {:?}",
                            component.variables.len(),component.unary_constraint_indices.len(),
                            component.binary_constraint_indices.len(),
                            core.as_ref().map(|core| (core.unary.iter().map(|u|format!("unary {} allowed={:?}",u.variable,u.allowed)).collect::<Vec<_>>(),
                                core.binary.iter().map(|b|format!("{} <-> {} allowed={:?}",b.left,b.right,b.allowed)).collect::<Vec<_>>())),
                            empty_witness, witness)
                    },
                    Ok(None) => "компонента не локализована".into(),
                    Err(error) => format!("локализация компоненты: {:?}",error),
                };
                diagnostic(DiagnosticCode::PhaseConflict,
                    format!("Нет совместимого выбора узлов и ordinary-рустов внутри курсов; {component}"),
                    None,None)
                })
            },
            SolveError::Exhausted {
                budget,
                states_examined,
            } => diagnostic(
                DiagnosticCode::SearchExhausted,
                format!("Поиск вариантов узлов исчерпан: {states_examined}/{budget}"),
                None,
                None,
            ),
        }]
    })?;
    let selected: BTreeMap<(i64, Point), NodeChoice> = domains
        .into_iter()
        .filter_map(|(key, _)| {
            let id = solution.assignment.get(&node_var(key.0, key.1))?;
            choices.get(id).cloned().map(|value| (key, value))
        })
        .collect();
    Ok(selected)
}

fn choose_pieces(length: i64, profile: &Profile, phase: i64) -> Option<Vec<(i64, bool)>> {
    fn search(
        remaining: i64,
        nominals: &[i64],
        index: usize,
        min_cut: i64,
        max_length: i64,
        states: &mut usize,
        selected: &mut Vec<i64>,
    ) -> Option<(Vec<i64>, Option<i64>)> {
        *states += 1;
        if *states > MAX_SEARCH_STATES {
            return None;
        }
        if index == nominals.len() {
            if remaining == 0 {
                return Some((selected.clone(), None));
            }
            if remaining >= min_cut && remaining <= max_length {
                return Some((selected.clone(), Some(remaining)));
            }
            return None;
        }
        let nominal = nominals[index];
        let max_count = remaining / nominal;
        if max_count as usize > MAX_BLOCKS {
            return None;
        }
        for count in (0..=max_count).rev() {
            let before = selected.len();
            selected.extend(std::iter::repeat(nominal).take(count as usize));
            if let Some(result) = search(
                remaining - count * nominal,
                nominals,
                index + 1,
                min_cut,
                max_length,
                states,
                selected,
            ) {
                return Some(result);
            }
            selected.truncate(before);
            if *states > MAX_SEARCH_STATES {
                return None;
            }
        }
        None
    }
    let mut nominals = profile.ordinary_nominal_lengths_centimm.clone();
    nominals.sort_unstable_by(|a, b| b.cmp(a));
    nominals.dedup();
    let mut states = 0;
    let (full, cut) = search(
        length,
        &nominals,
        0,
        profile.minimum_cut_centimm,
        profile.ordinary_length_centimm,
        &mut states,
        &mut Vec::new(),
    )?;
    let mut pieces: Vec<(i64, bool)> = full.into_iter().map(|v| (v, false)).collect();
    if let Some(c) = cut {
        if phase.rem_euclid(2) == 0 {
            pieces.push((c, true));
        } else {
            pieces.insert(0, (c, true));
        }
    }
    Some(pieces)
}

fn block(
    edge: &CourseEdge,
    course: i64,
    z: i64,
    span: Span,
    length: i64,
    kind: &str,
    bridge: bool,
    cuts: Vec<String>,
    sources: Vec<String>,
) -> Block {
    Block {
        id: format!(
            "c{course}:{}:{}-{}:{kind}",
            edge.edge_id, span.start, span.end
        ),
        wall_id: edge.wall_id.clone(),
        edge_id: edge.edge_id.clone(),
        course_index: course,
        z_centimm: z,
        start: point_at(edge, span.start, length),
        end: point_at(edge, span.end, length),
        length_centimm: span.len(),
        kind: kind.to_string(),
        product_key: None,
        catalog_status: "length_only".into(),
        rotation_deg: direction_deg((
            i128::from(edge.end.x) - i128::from(edge.start.x),
            i128::from(edge.end.y) - i128::from(edge.start.y),
        )),
        local_origin: None,
        local_rotation_deg: None,
        is_bridge: bridge,
        hide_spikes_left: cuts.iter().any(|v| v == "left"),
        hide_spikes_right: cuts.iter().any(|v| v == "right"),
        natural_end_left: !cuts.iter().any(|v| v == "left"),
        natural_end_right: !cuts.iter().any(|v| v == "right"),
        cuts,
        source_ids: sources,
        arms: Vec::new(),
        catalog_nominal_centimm: None,
        components: Vec::new(),
        obstacle_ends: Vec::new(),
    }
}

/// Узкий простенок включается в общую перемычку. Разрез разрешён только
/// в центре достаточно широкого простенка и вдали от шва нижнего венца.
fn lintel_groups<'a>(
    constraints: &'a Constraints,
    run: &str,
    course: i64,
) -> Vec<Vec<&'a LintelCandidate>> {
    let mut candidates: Vec<_> = constraints
        .lintel_candidates
        .iter()
        .filter(|v| v.course_index == course && v.run_id == run)
        .collect();
    candidates.sort_by_key(|v| (v.span.start, v.span.end));
    let mut groups: Vec<Vec<&LintelCandidate>> = Vec::new();
    for candidate in candidates {
        if constraints.console_axes.contains_key(&candidate.opening_id) {
            groups.push(vec![candidate]);
            continue;
        }
        if let Some(group) = groups.last_mut() {
            if !constraints.console_axes.contains_key(&group[0].opening_id)
                && candidate.span.start - group.last().unwrap().span.end <= 64_000 {
                group.push(candidate);
                continue;
            }
        }
        groups.push(vec![candidate]);
    }
    groups
}

fn lintel_segments(
    group: &[&LintelCandidate],
    bridge: Span,
    maximum: i64,
    below: &[Block],
    edge: &CourseEdge,
    course: i64,
) -> Option<Vec<Span>> {
    let mut joints = Vec::new();
    for pair in group.windows(2) {
        let center = pair[0].span.end + (pair[1].span.start - pair[0].span.end) / 2;
        let (Some(left), Some(right)) = (pair[0].right_support, pair[1].left_support) else {
            continue;
        };
        if center < left.end || center > right.start {
            continue;
        }
        let point = point_at(edge, center, edge_length(edge, 1).ok()?);
        let supported = below
            .iter()
            .filter(|b| b.course_index == course - 1 && b.edge_id == edge.edge_id)
            .any(|b| {
                let distance = |a: Point| ((a.x - point.x) as f64).hypot((a.y - point.y) as f64);
                distance(b.start) > 15_000.0
                    && distance(b.end) > 15_000.0
                    && (distance(b.start) + distance(b.end) - b.length_centimm as f64).abs() < 2.0
            });
        if supported {
            joints.push(center);
        }
    }
    let mut result = Vec::new();
    let mut start = bridge.start;
    while bridge.end - start > maximum {
        let end = joints
            .iter()
            .copied()
            .filter(|v| *v > start && *v - start <= maximum)
            .max()?;
        result.push(Span { start, end });
        start = end;
    }
    result.push(Span {
        start,
        end: bridge.end,
    });
    Some(result)
}

/// Стыки склейки допустимы только на уже существующем торце детали
/// внутри простенка с минимальным опиранием для обоих соседних проёмов.
fn glued_lintel_segments(
    selected: &[(usize, Span)],
    openings: &[(String, Span)],
    maximum: i64,
    support: i64,
) -> Option<Vec<Span>> {
    let mut start = selected.first()?.1.start;
    let end = selected.last()?.1.end;
    if maximum <= 0 {
        return None;
    }
    let joints: Vec<_> = selected.windows(2)
        .map(|pair| pair[0].1.end)
        .filter(|joint| openings.windows(2).any(|pair|
            *joint - pair[0].1.end >= support && pair[1].1.start - *joint >= support))
        .collect();
    let mut segments = Vec::new();
    while end - start > maximum {
        let joint = joints.iter().copied()
            .filter(|joint| *joint > start && *joint - start <= maximum)
            .max()?;
        segments.push(Span { start, end: joint });
        start = joint;
    }
    segments.push(Span { start, end });
    Some(segments)
}

/// Перемычка заменяет продольные детали, сохраняя врезки и поперечные
/// части узлов. Целые детали склеиваются с опиранием не меньше профильного
/// минимума (не менее 150 мм); из двух венцов выбирается короткий вариант.
fn merge_t_lintels(
    blocks: &mut Vec<Block>,
    constraints: &Constraints,
    profile: &Profile,
) -> BTreeSet<(i64, String)> {
    let mut completed = BTreeSet::new();
    let mut visited = BTreeSet::new();
    let maximum = profile
        .bridge_nominal_lengths_centimm
        .iter()
        .copied()
        .max()
        .unwrap_or(0);
    'candidate: for candidate in &constraints.lintel_candidates {
        if !visited.insert((candidate.course_index, candidate.opening_id.clone())) {
            continue;
        }
        let Some((a, b)) = constraints.opening_axis(&candidate.opening_id) else {
            continue;
        };
        let dx = b.x_mm - a.x_mm;
        let dy = b.y_mm - a.y_mm;
        let width = dx.hypot(dy);
        if width <= 0.0 {
            continue;
        }
        let (ux, uy) = (dx / width, dy / width);
        let project = |p: Point| {
            ((p.x as f64 - a.x_mm * 100.0) * ux + (p.y as f64 - a.y_mm * 100.0) * uy).round() as i64
        };
        let on_axis = |p: Point| {
            ((p.x as f64 - a.x_mm * 100.0) * uy - (p.y as f64 - a.y_mm * 100.0) * ux).abs() <= 1.0
        };
        let console = constraints.console_axes.contains_key(&candidate.opening_id);
        if console && !blocks.iter().any(|part| {
            part.course_index == candidate.course_index
                && part.product_key.as_deref() == Some("Type6")
                && part.arms.iter().all(|arm| on_axis(arm.start) && on_axis(arm.end))
                && part.arms.iter().flat_map(|arm| [arm.start, arm.end]).map(project).min().is_some_and(|u| u <= 0)
                && part.arms.iter().flat_map(|arm| [arm.start, arm.end]).map(project).max().is_some_and(|u| u >= 0)
        }) { continue; }
        let group = lintel_groups(constraints, &candidate.run_id, candidate.course_index)
            .into_iter()
            .find(|group| group.iter().any(|item| item.opening_id == candidate.opening_id))
            .unwrap_or_else(|| vec![candidate]);
        let support = (profile.lintel_support_mm.max(150.0) * 100.0).round() as i64;
        let mut openings = Vec::new();
        for item in &group {
            let Some((left, right)) = constraints.opening_axis(&item.opening_id) else {
                continue 'candidate;
            };
            let position = |point: crate::domain::RawPoint| project(Point {
                x: (point.x_mm * 100.0).round() as i64,
                y: (point.y_mm * 100.0).round() as i64,
            });
            let (left, right) = (position(left), position(right));
            openings.push((item.opening_id.clone(), Span { start: left.min(right), end: left.max(right) }));
        }
        openings.sort_by_key(|(_, span)| span.start);
        let required = if console { Span {
            start: -((width * 100.0).round() as i64).max(support) - 1,
            end: (width * 100.0).round() as i64,
        }} else { Span {
            start: openings.first().map(|(_, span)| span.start).unwrap_or(0) - support,
            end: openings.last().map(|(_, span)| span.end).unwrap_or(0) + support,
        }};
        if maximum <= 0
            || blocks.iter().any(|part| {
                part.course_index == candidate.course_index
                    && part.is_bridge
                    && part.source_ids.contains(&candidate.opening_id)
            })
        {
            continue;
        }
        let mut placement: Option<(i64, Vec<(usize, Span)>, Span, Vec<Span>)> = None;
        for offset in 0..=if console { 0 } else { 1 } {
            let course = candidate.course_index + offset;
            let mut coverage = Vec::new();
            for (index, part) in blocks
                .iter()
                .enumerate()
                .filter(|(_, p)| p.course_index == course)
            {
                let points: Vec<_> = if part.arms.is_empty() {
                    vec![part.start, part.end]
                } else {
                    part.arms
                        .iter()
                        .flat_map(|arm| [arm.start, arm.end])
                        .collect()
                };
                if points.iter().all(|p| on_axis(*p)) {
                    let start = points.iter().map(|p| project(*p)).min().unwrap_or(0);
                    let end = points.iter().map(|p| project(*p)).max().unwrap_or(0);
                    if end > start {
                        coverage.push((index, Span { start, end }));
                    }
                }
            }
            let mut selected: Vec<_> = coverage
                .into_iter()
                .filter(|(_, part_span)| part_span.intersects(required))
                .collect();
            selected.sort_by_key(|(_, part_span)| part_span.start);
            let (Some((_, first)), Some((_, last))) = (selected.first(), selected.last()) else {
                continue;
            };
            let span = Span { start: first.start, end: last.end };
            if span.start > required.start || span.end < required.end {
                continue;
            }
            let transverse_blocked = blocks
                .iter()
                .filter(|part| part.course_index == course && part.kind == "node_T")
                .any(|part| {
                    part.arms
                        .iter()
                        .any(|arm| !on_axis(arm.start) || !on_axis(arm.end))
                        && part
                            .arms
                            .iter()
                            .flat_map(|arm| [arm.start, arm.end])
                            .any(|p| on_axis(p) && project(p) > span.start && project(p) < span.end)
                        && part.product_key.as_deref() != Some("Type5_1")
                });
            if transverse_blocked {
                continue;
            }
            if selected
                    .windows(2)
                    .any(|pair| pair[0].1.end != pair[1].1.start)
                || selected.iter().any(|(i, _)| blocks[*i].is_bridge)
            {
                continue;
            }
            let segments = if console {
                (span.len() <= maximum).then_some(vec![span])
            } else { glued_lintel_segments(&selected, &openings, maximum, support) };
            let Some(segments) = segments else {
                continue;
            };
            if placement.as_ref().is_none_or(|(_, _, best, _)| span.len() < best.len()) {
                placement = Some((course, selected, span, segments));
            }
        }
        let Some((course, selected, _, segments)) = placement else {
            continue;
        };
        let point = |u: i64| Point {
            x: (a.x_mm * 100.0 + u as f64 * ux).round() as i64,
            y: (a.y_mm * 100.0 + u as f64 * uy).round() as i64,
        };
        let mut bridges = Vec::new();
        for span in segments {
            let selected: Vec<_> = selected.iter().copied()
                .filter(|(_, part)| part.intersects(span)).collect();
            let first_block = &blocks[selected[0].0];
            let mut merged = block(
                &CourseEdge {
                    edge_id: first_block.edge_id.clone(),
                    wall_id: first_block.wall_id.clone(),
                    start: point(span.start),
                    end: point(span.end),
                },
                course,
                first_block.z_centimm,
                Span {
                    start: 0,
                    end: span.len(),
                },
                span.len(),
                "bridge",
                true,
                Vec::new(),
                openings.iter().filter(|(_, opening)| opening.intersects(span))
                    .map(|(id, _)| id.clone()).collect(),
            );
            merged.id = format!(
                "c{course}:lintel:{}:{}-{}",
                candidate.opening_id, span.start, span.end
            );
            let mut cuts_valid = true;
            for (i, used) in &selected {
                let part = &blocks[*i];
                merged.source_ids.extend(part.source_ids.iter().cloned());
                for cut in &part.cuts {
                    if !cut.starts_with("Type") {
                        if cut.starts_with("opening_volume:") || cut.starts_with("beam_volume:") {
                            merged.cuts.push(cut.clone());
                        }
                        continue;
                    }
                    let fields: Vec<_> = cut.splitn(4, ':').collect();
                    let Some(local_position) = fields.get(1).and_then(|s| {
                        s.strip_prefix('x')
                            .and_then(|s| s.parse::<i64>().ok())
                            .map(|x| (x - 1) * 32_000)
                            .or_else(|| s.strip_prefix('p').and_then(|s| s.parse::<i64>().ok()))
                    }) else {
                        cuts_valid = false;
                        break;
                    };
                    let rotation = f64::from(part.rotation_deg).to_radians();
                    let cut_point = Point {
                        x: (part.start.x as f64 + local_position as f64 * rotation.cos()).round()
                            as i64,
                        y: (part.start.y as f64 + local_position as f64 * rotation.sin()).round()
                            as i64,
                    };
                    if project(cut_point) < used.start || project(cut_point) > used.end {
                        continue;
                    }
                    let position = project(cut_point) - span.start;
                    if fields.len() != 4 || position < 0 || position > span.len() {
                        cuts_valid = false;
                        break;
                    }
                    let coordinate = if position % 32_000 == 0 {
                        format!("x{}", position / 32_000 + 1)
                    } else {
                        format!("p{position}")
                    };
                    let Some(mut face) = fields[2]
                        .strip_prefix('y')
                        .and_then(|v| v.parse::<u8>().ok())
                        .filter(|v| (1..=3).contains(v))
                    else {
                        cuts_valid = false;
                        break;
                    };
                    // Разворот оси детали зеркалит и координату, и грань врезки.
                    if rotation.cos() * ux + rotation.sin() * uy < 0.0 {
                        face = 4 - face;
                    }
                    merged.cuts.push(format!(
                        "{}:{}:y{}:{}",
                        fields[0], coordinate, face, fields[3]
                    ));
                }
            }
            if !cuts_valid {
                continue 'candidate;
            }
            merged.cuts.sort();
            merged.cuts.dedup();
            merged.source_ids.sort();
            merged.source_ids.dedup();
            merged.catalog_nominal_centimm = profile
                .bridge_nominal_lengths_centimm
                .iter()
                .copied()
                .filter(|n| *n >= span.len())
                .min();
            bridges.push(merged);
        }
        let removed: BTreeSet<_> = selected.iter().map(|(index, _)| *index).collect();
        let mut index = 0;
        blocks.retain(|_| {
            let keep = !removed.contains(&index);
            index += 1;
            keep
        });
        blocks.extend(bridges);
        for item in group {
            completed.insert((item.course_index, item.opening_id.clone()));
            visited.insert((item.course_index, item.opening_id.clone()));
        }
    }
    completed
}

/// Производственный расчёт: любая диагностика кандидата запрещает выпуск.
pub fn calculate(
    topology: &Topology,
    constraints: &Constraints,
    profile: &Profile,
) -> Result<Layout, Vec<Diagnostic>> {
    let candidate = calculate_candidate(topology, constraints, profile)?;
    if candidate.diagnostics.is_empty() {
        Ok(candidate.layout)
    } else {
        Err(candidate.diagnostics)
    }
}

/// Возвращает полный физический кандидат. Неподдержанная геометрия и непокрытый solid — ошибка;
/// подтверждённые нарушения перевязки сохраняются отдельно от геометрии.
pub fn calculate_candidate(
    topology: &Topology,
    constraints: &Constraints,
    profile: &Profile,
) -> Result<CandidateCalculation, Vec<Diagnostic>> {
    let mut errors = Vec::new();
    if profile.profile_id.trim().is_empty()
        || profile.revision.trim().is_empty()
        || profile.catalog_version.trim().is_empty()
        || profile.phase_count <= 0
        || !profile.lintel_support_mm.is_finite()
        || profile.lintel_support_mm <= 0.0
        || !profile.assembly_clearance_mm.is_finite()
        || profile.assembly_clearance_mm < 0.0
        || profile.ordinary_length_centimm <= 0
        || profile
            .maximum_ordinary_blank_centimm
            .is_some_and(|v| v < profile.ordinary_length_centimm)
        || profile.maximum_special_blank_centimm.is_some_and(|v| {
            v < profile.ordinary_length_centimm || v > 10 * profile.ordinary_length_centimm
        })
        || !(1..=2).contains(&profile.max_ordinary_cuts_per_free_span)
        || profile.minimum_cut_centimm <= 0
        || profile.minimum_cut_centimm > profile.ordinary_length_centimm
        || profile.index_centimm != 6300
        || profile.coordinate_tolerance_centimm < 0
        || profile
            .node_assembly_family
            .as_deref()
            .is_some_and(|v| v != "forestbrick_legacy_observed")
        || profile
            .world_joint_policy
            .as_deref()
            .is_some_and(|v| v != "affine_checkerboard_v1")
    {
        return Err(vec![diagnostic(DiagnosticCode::UnsupportedCatalog,
            "Профиль требует версии каталога, длины ordinary, допустимого реза и общего индекса 63 мм", None, None)]);
    }
    if profile.world_joint_policy.is_some() {
        for course in &topology.courses {
            for run in &course.runs {
                if let Err(error) = joint_residue(course.index, run) {
                    return Err(vec![diagnostic(
                        DiagnosticCode::UnsupportedGeometry,
                        format!("Мировой руст для run {}: {:?}", run.id, error),
                        None,
                        Some(course.index),
                    )]);
                }
            }
        }
    }
    if profile
        .bridge_nominal_lengths_centimm
        .iter()
        .any(|v| *v <= 0)
        || profile.ordinary_nominal_lengths_centimm.is_empty()
        || profile.ordinary_nominal_lengths_centimm.len() > 16
        || profile
            .ordinary_nominal_lengths_centimm
            .iter()
            .any(|v| *v <= 0 || *v > profile.ordinary_length_centimm)
        || !profile
            .ordinary_nominal_lengths_centimm
            .contains(&profile.ordinary_length_centimm)
        || profile.nodes.iter().any(|n| {
            n.kind.is_empty()
                || n.phase < 0
                || n.phase >= profile.phase_count
                || n.variant.as_deref().is_some_and(|v| v != "t0_short")
                || (n.variant.as_deref() == Some("t0_short")
                    && (n.kind != "T" || n.phase != 0 || n.rays.len() != 3))
                || n.rays.is_empty()
                || n.rays.iter().any(|r| {
                    r.length_centimm <= 0 || r.direction_deg > 315 || r.direction_deg % 45 != 0
                })
                || n.rays
                    .iter()
                    .map(|r| r.direction_deg)
                    .collect::<BTreeSet<_>>()
                    .len()
                    != n.rays.len()
        })
    {
        return Err(vec![diagnostic(
            DiagnosticCode::UnsupportedCatalog,
            "Некорректные размеры, фазы или направления каталожного профиля",
            None,
            None,
        )]);
    }
    let selected_nodes = select_nodes(topology, constraints, profile)?;
    let mut blocks = Vec::new();
    let mut courses = topology.courses.clone();
    courses.sort_by_key(|c| (c.index, c.z));
    for course in &courses {
        let mut edges: Vec<CourseEdge> = course
            .runs
            .iter()
            .map(|r| CourseEdge {
                edge_id: r.id.clone(),
                wall_id: r.id.clone(),
                start: r.start,
                end: r.end,
            })
            .collect();
        let run_lengths: BTreeMap<_, _> = course
            .runs
            .iter()
            .map(|r| (r.id.clone(), r.length))
            .collect();
        let source_map: BTreeMap<_, _> = course
            .runs
            .iter()
            .map(|r| {
                let mut ids = Vec::new();
                for s in &r.sources {
                    ids.push(format!("wall:{}", s.wall_id));
                    ids.push(format!("edge:{}", s.edge_id));
                }
                ids.sort();
                ids.dedup();
                (r.id.clone(), ids)
            })
            .collect();
        let assembly_sources: BTreeMap<_, _> = course
            .runs
            .iter()
            .map(|r| {
                let mut ids: Vec<String> = r.sources.iter().map(|s| s.wall_id.clone()).collect();
                ids.sort();
                ids.dedup();
                (r.id.clone(), ids)
            })
            .collect();
        edges.sort_by(|a, b| {
            (&a.wall_id, &a.edge_id, a.start, a.end).cmp(&(&b.wall_id, &b.edge_id, b.start, b.end))
        });
        let mut incident: BTreeMap<Point, Vec<(usize, bool, (i128, i128))>> = BTreeMap::new();
        let mut lengths = Vec::new();
        for (i, edge) in edges.iter().enumerate() {
            match edge_length(edge, profile.coordinate_tolerance_centimm) {
                Ok(calculated)
                    if run_lengths.get(&edge.edge_id).is_some_and(|v| {
                        (v - calculated).abs() <= profile.coordinate_tolerance_centimm.max(1)
                    }) =>
                {
                    lengths.push(run_lengths[&edge.edge_id])
                }
                Ok(_) => {
                    errors.push(diagnostic(
                        DiagnosticCode::UnsupportedGeometry,
                        "Длина run не совпадает с канонической геометрией",
                        Some(edge),
                        Some(course.index),
                    ));
                    lengths.push(0);
                    continue;
                }
                Err(mut err) => {
                    err.course_index = Some(course.index);
                    errors.push(err);
                    lengths.push(0);
                    continue;
                }
            }
            let dx = i128::from(edge.end.x) - i128::from(edge.start.x);
            let dy = i128::from(edge.end.y) - i128::from(edge.start.y);
            incident
                .entry(edge.start)
                .or_default()
                .push((i, true, (dx, dy)));
            incident
                .entry(edge.end)
                .or_default()
                .push((i, false, (-dx, -dy)));
        }
        let mut arms = vec![(0_i64, 0_i64); edges.len()];
        let mut node_blocks = Vec::new();
        for (point, entries) in &incident {
            if entries.len() < 2 {
                continue;
            }
            let dirs: Vec<_> = entries.iter().map(|e| e.2).collect();
            if is_straight(&dirs) {
                continue;
            }
            let Some(choice) = selected_nodes.get(&(course.index, *point)) else {
                continue;
            };
            let kind = &choice.kind;
            let geometry = &choice.geometry;
            let mut node_arms = Vec::new();
            let mut hide_left = false;
            let mut hide_right = false;
            for resolved in &geometry.arms {
                let Some(interval) = resolved.active_interval else {
                    continue;
                };
                let Some(idx) = edges.iter().position(|e| e.edge_id == resolved.run_id) else {
                    continue;
                };
                let span = Span {
                    start: interval.start,
                    end: interval.end,
                };
                let slot = &mut arms[idx];
                if resolved.at_start {
                    slot.0 = span.len();
                } else {
                    slot.1 = span.len();
                }
                if resolved.cut_at_distal_end {
                    if resolved.at_start {
                        hide_right = true;
                    } else {
                        hide_left = true;
                    }
                }
                node_arms.push(BlockArm {
                    wall_id: edges[idx].wall_id.clone(),
                    edge_id: edges[idx].edge_id.clone(),
                    start: point_at(&edges[idx], span.start, lengths[idx]),
                    end: point_at(&edges[idx], span.end, lengths[idx]),
                    length_centimm: span.len(),
                });
            }
            node_arms.sort_by(|a, b| (&a.wall_id, &a.edge_id).cmp(&(&b.wall_id, &b.edge_id)));
            if let Some(first) = node_arms.first() {
                if profile.node_assembly_family.as_deref() == Some("forestbrick_legacy_observed") {
                    let node_kind = match kind.as_str() {
                        "L" => Some(NodeKind::L),
                        "T" => Some(NodeKind::T),
                        "X" => Some(NodeKind::X),
                        "Y" => Some(NodeKind::Y),
                        _ => None,
                    };
                    let Some(node_kind) = node_kind else {
                        errors.push(diagnostic(
                            DiagnosticCode::UnsupportedCatalog,
                            format!("Нет физического типа узла {kind}"),
                            None,
                            Some(course.index),
                        ));
                        continue;
                    };
                    let Ok(phase) = u8::try_from(choice.phase) else {
                        errors.push(diagnostic(
                            DiagnosticCode::UnsupportedCatalog,
                            "Фаза физического узла вне диапазона",
                            None,
                            Some(course.index),
                        ));
                        continue;
                    };
                    let parts = match assemble_node(
                        node_kind,
                        phase,
                        choice.rotation_deg,
                        *point,
                        &geometry.arms,
                        &assembly_sources,
                    ) {
                        Ok(v) => v,
                        Err(e) => {
                            errors.push(diagnostic(
                                DiagnosticCode::UnsupportedCatalog,
                                format!(
                                    "Не собран физический узел ({},{}): {e:?}",
                                    point.x, point.y
                                ),
                                None,
                                Some(course.index),
                            ));
                            continue;
                        }
                    };
                    let mut expected: Vec<_> = geometry
                        .arms
                        .iter()
                        .filter_map(|a| {
                            a.active_interval
                                .map(|v| (a.run_id.clone(), v.start, v.end))
                        })
                        .collect();
                    let mut actual: Vec<_> = parts
                        .iter()
                        .flat_map(|p| {
                            p.covered_arms
                                .iter()
                                .map(|a| (a.run_id.clone(), a.interval.start, a.interval.end))
                        })
                        .collect();
                    expected.sort();
                    actual.sort();
                    if expected != actual {
                        errors.push(diagnostic(
                            DiagnosticCode::UncoveredSolid,
                            "Физические детали узла не покрывают ровно выбранные плечи",
                            None,
                            Some(course.index),
                        ));
                        continue;
                    }
                    for (part_index, part) in parts.into_iter().enumerate() {
                        let angle = f64::from(part.world_rotation_deg).to_radians();
                        let (axis_x, axis_y) = (angle.cos(), angle.sin());
                        let mut part_hide_left = false;
                        let mut part_hide_right = false;
                        let mut oriented_cut = false;
                        for resolved in geometry.arms.iter().filter(|arm| arm.cut_at_distal_end) {
                            if !part
                                .covered_arms
                                .iter()
                                .any(|arm| arm.run_id == resolved.run_id)
                            {
                                continue;
                            }
                            let Some(idx) = edges
                                .iter()
                                .position(|edge| edge.edge_id == resolved.run_id)
                            else {
                                continue;
                            };
                            let edge = &edges[idx];
                            let direction = if resolved.at_start { 1.0 } else { -1.0 };
                            let dot = direction
                                * ((edge.end.x - edge.start.x) as f64 * axis_x
                                    + (edge.end.y - edge.start.y) as f64 * axis_y);
                            if dot.abs() > 1.0 {
                                oriented_cut = true;
                                part_hide_left |= dot < 0.0;
                                part_hide_right |= dot > 0.0;
                            }
                        }
                        if !oriented_cut {
                            part_hide_left = hide_left;
                            part_hide_right = hide_right;
                        }
                        let mut part_arms = Vec::new();
                        for arm in &part.covered_arms {
                            let Some(idx) = edges.iter().position(|e| e.edge_id == arm.run_id)
                            else {
                                continue;
                            };
                            part_arms.push(BlockArm {
                                wall_id: edges[idx].wall_id.clone(),
                                edge_id: arm.run_id.clone(),
                                start: point_at(&edges[idx], arm.interval.start, lengths[idx]),
                                end: point_at(&edges[idx], arm.interval.end, lengths[idx]),
                                length_centimm: arm.interval.end - arm.interval.start,
                            });
                        }
                        let mut sources: Vec<String> = part
                            .source_ids
                            .iter()
                            .map(|id| {
                                if id.starts_with("beam:")
                                    || id.starts_with("opening:")
                                    || id.starts_with("wall-boundary:")
                                {
                                    id.clone()
                                } else {
                                    format!("wall:{id}")
                                }
                            })
                            .collect();
                        sources.extend(geometry.source_ids.iter().cloned());
                        sources.sort();
                        sources.dedup();
                        let cuts = part
                            .cuts
                            .iter()
                            .map(|c| {
                                format!(
                                    "{:?}:x{}:y{}:{}",
                                    c.cut.product,
                                    c.cut.x,
                                    c.cut.y,
                                    c.source_run_ids.join(",")
                                )
                            })
                            .chain(geometry.cuts.iter().cloned())
                            .collect();
                        node_blocks.push(Block {
                            id: format!(
                                "c{}:node:{}:{}:{kind}:part{part_index}",
                                course.index, point.x, point.y
                            ),
                            wall_id: part_arms
                                .first()
                                .map(|a| a.wall_id.clone())
                                .unwrap_or_default(),
                            edge_id: part_arms
                                .first()
                                .map(|a| a.edge_id.clone())
                                .unwrap_or_default(),
                            course_index: course.index,
                            z_centimm: course.z,
                            start: part.world_origin,
                            end: part.world_origin,
                            length_centimm: part.nominal_length_centimm,
                            kind: format!("node_{kind}"),
                            product_key: Some(format!("{:?}", part.product)),
                            catalog_status: format!("{:?}", part.catalog_status),
                            rotation_deg: part.world_rotation_deg,
                            local_origin: Some(part.local_origin),
                            local_rotation_deg: Some(part.local_rotation_deg),
                            is_bridge: false,
                            hide_spikes_left: part_hide_left,
                            hide_spikes_right: part_hide_right,
                            natural_end_left: true,
                            natural_end_right: true,
                            cuts,
                            source_ids: sources,
                            arms: part_arms,
                            catalog_nominal_centimm: Some(part.nominal_length_centimm),
                            components: Vec::new(),
                            obstacle_ends: Vec::new(),
                        });
                    }
                } else {
                    node_blocks.push(Block {
                        id: format!("c{}:node:{}:{}:{kind}", course.index, point.x, point.y),
                        wall_id: first.wall_id.clone(),
                        edge_id: first.edge_id.clone(),
                        course_index: course.index,
                        z_centimm: course.z,
                        start: *point,
                        end: *point,
                        length_centimm: geometry
                            .arms
                            .iter()
                            .map(|a| a.nominal_length_centimm)
                            .max()
                            .unwrap_or(0),
                        kind: format!("node_{kind}"),
                        product_key: None,
                        catalog_status: "unverified".into(),
                        rotation_deg: choice.rotation_deg,
                        local_origin: None,
                        local_rotation_deg: None,
                        is_bridge: false,
                        hide_spikes_left: hide_left,
                        hide_spikes_right: hide_right,
                        natural_end_left: true,
                        natural_end_right: true,
                        cuts: geometry.cuts.clone(),
                        source_ids: geometry.source_ids.clone(),
                        arms: node_arms,
                        catalog_nominal_centimm: None,
                        components: Vec::new(),
                        obstacle_ends: Vec::new(),
                    });
                }
            }
        }
        for (i, edge) in edges.iter().enumerate() {
            let length = lengths[i];
            if length == 0 {
                continue;
            }
            let (start_arm, end_arm) = arms[i];
            if start_arm + end_arm > length {
                errors.push(diagnostic(
                    DiagnosticCode::NodeCollision,
                    "Фиксированные узлы перекрываются",
                    Some(edge),
                    Some(course.index),
                ));
                continue;
            }
            let mut voids = Vec::new();
            for mask in constraints
                .masks
                .iter()
                .filter(|m| m.course_index == course.index && m.run_id == edge.edge_id)
            {
                let span = Span {
                    start: mask.interval.start,
                    end: mask.interval.end,
                };
                if span.start < 0 || span.end > length || span.len() <= 0 {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidMask,
                        format!(
                            "Маска {} [{}, {}) выходит за run {} [0, {})",
                            mask_source(&mask.source),
                            span.start,
                            span.end,
                            edge.edge_id,
                            length
                        ),
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                }
                match mask.coverage {
                    MaskCoverage::FullSectionVoid => voids.push(span),
                    MaskCoverage::PartialDepth => {
                        if constraints.evidence.is_empty() {
                            errors.push(diagnostic(DiagnosticCode::UnsupportedGeometry,
                                format!("Частичное 3D-пересечение {} на run {} [{}, {}) без точной геометрии",
                                    mask_source(&mask.source),edge.edge_id,span.start,span.end),
                                Some(edge),Some(course.index)));
                        }
                    }
                }
            }
            voids.sort_by_key(|v| (v.start, v.end));
            let mut solid = vec![Span {
                start: 0,
                end: length,
            }];
            for v in &voids {
                solid = subtract(solid, *v);
            }
            let mut expected_solid = solid.clone();
            let mut placed = Vec::new();
            for group in lintel_groups(constraints, &edge.edge_id, course.index) {
                if constraints.console_axes.contains_key(&group[0].opening_id) {
                    continue;
                }
                if group.iter().all(|candidate| constraints.opening_axis(&candidate.opening_id).is_some()) {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Не найдено склеиваемых деталей с достаточным опиранием",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                }
                let first = group[0];
                let last = group[group.len() - 1];
                let Some(left) = first.left_support else {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Нет левой опоры перемычки",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                };
                let Some(right) = last.right_support else {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Нет правой опоры перемычки",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                };
                let minimum_support = (profile.lintel_support_mm.max(150.0) * 100.0).round() as i64;
                let bridge = Span {
                    start: left.start.min(first.span.start - minimum_support),
                    end: right.end.max(last.span.end + minimum_support),
                };
                if left.end != first.span.start
                    || right.start != last.span.end
                    || bridge.start < start_arm
                    || bridge.end > length - end_arm
                    || bridge.len() <= 0
                    || voids.iter().any(|v| v.intersects(bridge))
                {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Опоры перемычки должны быть сплошными и вне фиксированных узлов",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                }
                let maximum = profile
                    .bridge_nominal_lengths_centimm
                    .iter()
                    .copied()
                    .max()
                    .unwrap_or(0);
                let Some(segments) = (maximum > 0)
                    .then(|| lintel_segments(&group, bridge, maximum, &blocks, edge, course.index))
                    .flatten()
                else {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Нет допустимых стыков на простенках в пределах максимальной длины перемычки",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                };
                if !solid
                    .iter()
                    .any(|s| s.start <= bridge.start && s.end >= bridge.end)
                {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Перемычка пересекает уже занятый участок",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                }
                match classify_candidate(topology, constraints, course, edge, bridge, profile) {
                    Ok(Exclusion::None) => {}
                    Ok(Exclusion::Partial { .. }) => {}
                    Ok(Exclusion::FullVoid { source_ids }) => {
                        errors.push(diagnostic(
                            DiagnosticCode::InvalidSupport,
                            format!("Перемычка пересекает 3D-пустоту {:?}", source_ids),
                            Some(edge),
                            Some(course.index),
                        ));
                        continue;
                    }
                    Err(error) => {
                        errors.push(error);
                        continue;
                    }
                }
                solid = subtract(solid, bridge);
                placed.push(bridge);
                for bridge in segments {
                    let nominal = profile
                        .bridge_nominal_lengths_centimm
                        .iter()
                        .copied()
                        .filter(|v| *v >= bridge.len())
                        .min()
                        .unwrap_or(maximum);
                    let cuts = if nominal > bridge.len() {
                        vec!["right".into()]
                    } else {
                        Vec::new()
                    };
                    let mut item = block(
                        edge,
                        course.index,
                        course.z,
                        bridge,
                        length,
                        "bridge",
                        true,
                        cuts,
                        group.iter().map(|v| v.opening_id.clone()).collect(),
                    );
                    item.catalog_nominal_centimm = Some(nominal);
                    blocks.push(item);
                }
            }
            for node_span in [
                Span {
                    start: 0,
                    end: start_arm,
                },
                Span {
                    start: length - end_arm,
                    end: length,
                },
            ] {
                if node_span.len() == 0 {
                    continue;
                }
                if let Some(mask) = constraints.masks.iter().find(|m| {
                    m.course_index == course.index
                        && m.run_id == edge.edge_id
                        && m.coverage == MaskCoverage::FullSectionVoid
                        && node_span.intersects(Span {
                            start: m.interval.start,
                            end: m.interval.end,
                        })
                }) {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidMask,
                        format!("Маска {} [{}, {}) пересекает фиксированный узел у {} на run {} [{}, {})",
                            mask_source(&mask.source),mask.interval.start,mask.interval.end,
                            if node_span.start==0 {format!("({},{})",edge.start.x,edge.start.y)}
                            else {format!("({},{})",edge.end.x,edge.end.y)},
                            edge.edge_id,node_span.start,node_span.end),
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                }
                placed.push(node_span);
            }
            for span in solid {
                let run = Span {
                    start: span.start.max(start_arm),
                    end: span.end.min(length - end_arm),
                };
                if run.len() <= 0 {
                    continue;
                }
                let phase = selected_nodes
                    .get(&(course.index, edge.start))
                    .or_else(|| selected_nodes.get(&(course.index, edge.end)))
                    .map(|n| n.phase)
                    .unwrap_or(0);
                let run_geometry = course.runs.iter().find(|item| item.id == edge.edge_id);
                let residue =
                    run_geometry.and_then(|item| ordinary_residue(item, course.index, profile));
                let roof_start = run_geometry.is_some_and(|geometry| {
                    roof_clips_end(
                        topology,
                        course,
                        geometry,
                        point_at(edge, run.start, length),
                        profile,
                    )
                });
                let roof_end = run_geometry.is_some_and(|geometry| {
                    roof_clips_end(
                        topology,
                        course,
                        geometry,
                        point_at(edge, run.end, length),
                        profile,
                    )
                });
                let Some(pieces) = choose_pieces_with_roof_ends(
                    run.start, run.end, profile, phase, residue, roof_start, roof_end,
                ) else {
                    let loose = choose_pieces(run.len(), profile, phase);
                    let conflicting_joint = residue.and_then(|value| {
                        loose
                            .as_ref()
                            .and_then(|parts| first_offgrid_joint(run.start, parts, value))
                    });
                    let code = if residue.is_some() && loose.is_some() {
                        DiagnosticCode::WorldGridIncompatible
                    } else {
                        DiagnosticCode::SearchExhausted
                    };
                    errors.push(diagnostic(
                        code,
                        format!(
                            "Нет раскладки ordinary с двумя краевыми компенсаторами для run {} [{}, {}), мировой residue {:?}, пример недопустимого руста {:?}",
                            edge.edge_id, run.start, run.end, residue, conflicting_joint
                        ),
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                };
                if pieces.len() > MAX_BLOCKS.saturating_sub(blocks.len()) {
                    errors.push(diagnostic(
                        DiagnosticCode::SearchExhausted,
                        "Превышен предел количества блоков раскладки",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                }
                let mut cursor = run.start;
                let piece_count = pieces.len();
                for (piece_index, (piece, is_cut)) in pieces.into_iter().enumerate() {
                    let current = Span {
                        start: cursor,
                        end: cursor + piece,
                    };
                    if piece < 2_000
                        && ((roof_start && piece_index == 0)
                            || (roof_end && piece_index + 1 == piece_count))
                    {
                        expected_solid = subtract(expected_solid, current);
                        cursor += piece;
                        continue;
                    }
                    let mut volume_sources = Vec::new();
                    match classify_candidate(topology, constraints, course, edge, current, profile)
                    {
                        Ok(Exclusion::None) => {}
                        Ok(Exclusion::Partial { source_ids }) => volume_sources = source_ids,
                        Ok(Exclusion::FullVoid { .. }) => {
                            // Точная объёмная проверка доказала, что материал
                            // полностью занят препятствием. Он исключается и
                            // из эталона покрытия, а не становится пустой деталью.
                            expected_solid = subtract(expected_solid, current);
                            cursor += piece;
                            continue;
                        }
                        Err(error) => {
                            errors.push(error);
                            cursor += piece;
                            continue;
                        }
                    }
                    let cuts = if is_cut {
                        vec![if piece_index == 0 && piece_count > 1 {
                            "left".to_string()
                        } else {
                            "right".to_string()
                        }]
                    } else {
                        Vec::new()
                    };
                    let mut item = block(
                        edge,
                        course.index,
                        course.z,
                        current,
                        length,
                        "ordinary",
                        false,
                        cuts,
                        Vec::new(),
                    );
                    for source in volume_sources {
                        if let Some(id) = source.strip_prefix("opening:") {
                            item.cuts.push(format!("opening_volume:{id}"));
                            item.source_ids.push(source);
                        } else if let Some(id) = source.strip_prefix("beam:") {
                            item.cuts.push(format!("beam_volume:{id}"));
                            item.source_ids.push(source);
                        }
                    }
                    // Даже каталожный блок у точной грани проёма имеет
                    // спиленные шипы: наличие продольной подрезки не требуется.
                    for mask in constraints
                        .masks
                        .iter()
                        .filter(|m| m.course_index == course.index && m.run_id == edge.edge_id)
                    {
                        if current.start == mask.interval.end {
                            item.hide_spikes_left = true;
                            item.source_ids.push(mask_source(&mask.source));
                        }
                        if current.end == mask.interval.start {
                            item.hide_spikes_right = true;
                            item.source_ids.push(mask_source(&mask.source));
                        }
                    }
                    if roof_start && piece_index == 0 {
                        item.hide_spikes_left = false;
                    }
                    if roof_end && piece_index + 1 == piece_count {
                        item.hide_spikes_right = false;
                    }
                    if profile.world_joint_policy.is_none() {
                        item.catalog_status = "length_only_grid_unverified".into();
                    }
                    item.catalog_nominal_centimm = if piece > profile.ordinary_length_centimm
                        && profile.maximum_special_blank_centimm.is_some()
                    {
                        item.kind = "special_ordinary".into();
                        item.catalog_status = "special_product_required".into();
                        item.product_key = Some(format!("SpecialP{}", piece as f64 / 100.0));
                        Some(piece)
                    } else if is_cut {
                        let selected = profile
                            .ordinary_nominal_lengths_centimm
                            .iter()
                            .copied()
                            .filter(|v| *v >= piece)
                            .min();
                        selected.or_else(|| {
                            profile
                                .ordinary_nominal_lengths_centimm
                                .iter()
                                .copied()
                                .max()
                        })
                    } else {
                        Some(piece)
                    };
                    if profile.world_joint_policy.as_deref() == Some("affine_checkerboard_v1")
                        && matches!(item.rotation_deg, 0 | 90 | 180 | 270) {
                        // Происхождение торца определяется исходной мировой сеткой
                        // заготовок, а не длиной остатка или названием первого запила.
                        let factory = |point: Point| (i128::from(point.x) + i128::from(point.y)).rem_euclid(32_000) == 0;
                        item.natural_end_left = factory(item.start);
                        item.natural_end_right = factory(item.end);
                        item.hide_spikes_left = !item.natural_end_left;
                        item.hide_spikes_right = !item.natural_end_right;
                    }
                    blocks.push(item);
                    placed.push(current);
                    cursor += piece;
                }
            }
            placed.sort_by_key(|v| (v.start, v.end));
            let mut merged = Vec::<Span>::new();
            for span in placed {
                if let Some(last) = merged.last_mut() {
                    if last.end == span.start {
                        last.end = span.end;
                        continue;
                    }
                }
                merged.push(span);
            }
            if merged != expected_solid {
                errors.push(diagnostic(
                    DiagnosticCode::UncoveredSolid,
                    "Блоки не дают однократного полного покрытия solid",
                    Some(edge),
                    Some(course.index),
                ));
            }
        }
        blocks.extend(node_blocks);
        for item in blocks.iter_mut().filter(|b| b.course_index == course.index) {
            if let Some(ids) = source_map.get(&item.edge_id) {
                item.source_ids.extend(ids.iter().cloned());
            }
            for arm in &item.arms {
                if let Some(ids) = source_map.get(&arm.edge_id) {
                    item.source_ids.extend(ids.iter().cloned());
                }
            }
            item.source_ids.sort();
            item.source_ids.dedup();
            if !item.arms.is_empty() {
                for source in &item.source_ids {
                    if let Some(id) = source.strip_prefix("beam:") {
                        let cut = format!("beam_volume:{id}");
                        if !item.cuts.contains(&cut) {
                            item.cuts.push(cut);
                        }
                    }
                }
            }
        }
    }
    let completed_t = merge_t_lintels(&mut blocks, constraints, profile);
    let mut support_warnings = Vec::new();
    errors.retain(|issue| {
        if issue.code==DiagnosticCode::InvalidSupport {
            let affected: Vec<_> = constraints.lintel_candidates.iter().filter(|l|
                Some(l.course_index) == issue.course_index && Some(&l.run_id) == issue.edge_id.as_ref()).collect();
            if !affected.is_empty() && affected.iter().all(|l| completed_t.contains(&(l.course_index, l.opening_id.clone()))) {
                return false;
            }
            let mut warning=issue.clone();
            warning.source_ids.extend(affected.iter().map(|l| format!("opening:{}", l.opening_id)));
            if let Some(run) = topology.courses.iter().find(|c| Some(c.index) == issue.course_index)
                .and_then(|c| c.runs.iter().find(|r| Some(&r.id) == issue.edge_id.as_ref())) {
                warning.source_ids.push(run.id.clone());
                warning.source_ids.extend(run.sources.iter().map(|s| format!("wall:{}", s.wall_id)));
            }
            warning.source_ids.sort(); warning.source_ids.dedup();
            warning.message=format!("Перемычка требует уточнения: {}. В просмотре сохранён материал ряда обычными деталями",warning.message);
            support_warnings.push(warning);false
        } else {true}
    });
    if !errors.is_empty() {
        let mut provenance: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for course in &topology.courses {
            for run in &course.runs {
                let ids = provenance.entry(run.id.clone()).or_default();
                for source in &run.sources {
                    ids.push(format!("wall:{}", source.wall_id));
                    ids.push(format!("edge:{}", source.edge_id));
                }
                ids.sort();
                ids.dedup();
            }
        }
        for error in &mut errors {
            if let Some(run) = error.edge_id.as_ref().and_then(|id| provenance.get(id)) {
                error.source_ids.extend(run.iter().cloned());
            }
        }
        return Err(aggregate_errors(errors));
    }
    let mut vertical_errors = validate_vertical_plan(topology, &blocks, profile);
    if vertical_errors
        .iter()
        .any(|issue| issue.code != DiagnosticCode::VerticalJointConflict)
    {
        return Err(aggregate_errors(vertical_errors));
    }
    blocks.sort_by(|a, b| {
        (&a.course_index, &a.wall_id, &a.edge_id, &a.id).cmp(&(
            &b.course_index,
            &b.wall_id,
            &b.edge_id,
            &b.id,
        ))
    });
    let mut ids = BTreeSet::new();
    if blocks.iter().any(|b| !ids.insert(b.id.clone())) {
        return Err(vec![diagnostic(
            DiagnosticCode::UncoveredSolid,
            "Дублированный ID блока",
            None,
            None,
        )]);
    }
    vertical_errors.extend(support_warnings);
    for (&(course, _), node) in &selected_nodes {
        for arm in &node.geometry.arms {
            if !arm
                .cut_source_ids
                .iter()
                .any(|source| source.starts_with("wall-boundary:"))
            {
                continue;
            }
            let actual = arm
                .active_interval
                .map(|interval| interval.end - interval.start)
                .unwrap_or(0);
            let mut warning = diagnostic(DiagnosticCode::NodeDistalWallTrim,
                format!("Плечо узла подрезано реальной границей стены на {}: материал {}, каталожная заготовка {} сохранена вместе с фиксированным стыком",
                    arm.nominal_length_centimm - actual, actual, arm.nominal_length_centimm), None, Some(course));
            warning.edge_id = Some(arm.run_id.clone());
            warning.source_ids = arm.cut_source_ids.clone();
            vertical_errors.push(warning);
        }
    }
    Ok(CandidateCalculation {
        layout: Layout { blocks },
        diagnostics: aggregate_errors(vertical_errors),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraints::{Interval, LintelCandidate, Mask};
    use crate::domain::{Course, RunSource, WallRun};
    fn calculate(t: &Topology, c: &Constraints, p: &Profile) -> Result<Layout, Vec<Diagnostic>> {
        super::calculate(t, c, p)
    }

    fn profile() -> Profile {
        Profile {
            profile_id: "test".into(),
            revision: "1".into(),
            catalog_version: "test".into(),
            node_assembly_family: None,
            world_joint_policy: None,
            lintel_support_mm: 50.0,
            longitudinal_full_wall_thickness: false,
            full_course_clearance: false,
            assembly_clearance_mm: 0.0,
            ordinary_length_centimm: 64_000,
            maximum_ordinary_blank_centimm: None,
            maximum_special_blank_centimm: None,
            ordinary_nominal_lengths_centimm: vec![32_000, 64_000],
            max_ordinary_cuts_per_free_span: 1,
            minimum_cut_centimm: 1_000,
            index_centimm: 6_300,
            coordinate_tolerance_centimm: 0,
            phase_count: 1,
            nodes: vec![NodeSpec {
                kind: "T".into(),
                phase: 0,
                variant: None,
                rays: vec![
                    RayLength {
                        direction_deg: 0,
                        length_centimm: 5_000,
                    },
                    RayLength {
                        direction_deg: 90,
                        length_centimm: 5_000,
                    },
                    RayLength {
                        direction_deg: 180,
                        length_centimm: 5_000,
                    },
                ],
            }],
            bridge_nominal_lengths_centimm: vec![30_000],
        }
    }
    fn edge(id: &str, wall: &str, a: (i64, i64), b: (i64, i64)) -> CourseEdge {
        CourseEdge {
            edge_id: id.into(),
            wall_id: wall.into(),
            start: Point { x: a.0, y: a.1 },
            end: Point { x: b.0, y: b.1 },
        }
    }
    fn topology(edges: Vec<CourseEdge>, index: i64) -> Topology {
        let runs = edges
            .iter()
            .map(|e| WallRun {
                id: e.edge_id.clone(),
                start: e.start,
                end: e.end,
                length: edge_length(e, 0).unwrap(),
                sources: vec![RunSource {
                    wall_id: e.wall_id.clone(),
                    edge_id: e.edge_id.clone(),
                    start: e.start,
                    end: e.end,
                    start_offset: 0,
                    end_offset: edge_length(e, 0).unwrap(),
                }],
            })
            .collect();
        Topology {
            z0: 0,
            walls: Vec::new(),
            vertices: Vec::new(),
            edges: Vec::new(),
            courses: vec![Course {
                index,
                z: index * 6300,
                edges,
                runs,
                vertices: Vec::new(),
            }],
            vertical_links: Vec::new(),
        }
    }
    fn local_topology(mut edges: Vec<CourseEdge>, index: i64) -> Topology {
        for edge in &mut edges {
            edge.start.x += 1;
            edge.start.y += 1;
            edge.end.x += 1;
            edge.end.y += 1;
        }
        topology(edges, index)
    }
    fn covered(blocks: &[Block], edge_id: &str) -> i64 {
        blocks
            .iter()
            .map(|b| {
                if b.arms.is_empty() {
                    if b.edge_id == edge_id {
                        b.length_centimm
                    } else {
                        0
                    }
                } else {
                    b.arms
                        .iter()
                        .filter(|a| a.edge_id == edge_id)
                        .map(|a| a.length_centimm)
                        .sum()
                }
            })
            .sum()
    }
    #[test]
    fn upper_course_is_placed_at_global_height() {
        let t = topology(vec![edge("e", "upper", (0, 0), (130_000, 0))], 42);
        let result = calculate(&t, &Constraints::default(), &profile()).unwrap();
        assert_eq!(covered(&result.blocks, "e"), 130_000);
        assert!(result
            .blocks
            .iter()
            .all(|b| b.course_index == 42 && b.z_centimm == 264_600));
    }
    #[test]
    fn order_and_reversal_do_not_change_output() {
        let a = edge("a", "a", (0, 0), (100_000, 0));
        let b = edge("b", "b", (0, 20_000), (100_000, 20_000));
        let one = calculate(
            &topology(vec![a.clone(), b.clone()], 0),
            &Constraints::default(),
            &profile(),
        )
        .unwrap();
        let two = calculate(
            &topology(vec![b, a], 0),
            &Constraints::default(),
            &profile(),
        )
        .unwrap();
        assert_eq!(one, two);
    }
    #[test]
    fn diagonal_uses_local_length_and_exact_terminal_point() {
        let t = topology(vec![edge("d", "diagonal", (0, 0), (45_255, 45_255))], 0);
        let result = calculate(&t, &Constraints::default(), &profile()).unwrap();
        assert_eq!(result.blocks.len(), 1);
        assert_eq!(
            result.blocks[0].end,
            Point {
                x: 45_255,
                y: 45_255
            }
        );
    }
    #[test]
    fn t_node_uses_fixed_arms_once_per_edge() {
        let t = local_topology(
            vec![
                edge("a", "a", (-50_000, 0), (0, 0)),
                edge("b", "b", (0, 0), (50_000, 0)),
                edge("c", "c", (0, 0), (0, 50_000)),
            ],
            0,
        );
        let result = calculate(&t, &Constraints::default(), &profile()).unwrap();
        assert_eq!(
            result.blocks.iter().filter(|b| b.kind == "node_T").count(),
            1
        );
        for id in ["a", "b", "c"] {
            assert_eq!(covered(&result.blocks, id), 50_000);
        }
    }
    #[test]
    fn full_void_cuts_node_arm_and_keeps_nominal() {
        let t = local_topology(
            vec![
                edge("w", "w", (-50_000, 0), (0, 0)),
                edge("e", "e", (0, 0), (50_000, 0)),
                edge("n", "n", (0, 0), (0, 50_000)),
            ],
            0,
        );
        let mask = Mask {
            run_id: "e".into(),
            course_index: 0,
            interval: Interval {
                start: 3_000,
                end: 6_000,
            },
            source: MaskSource::Beam("beam".into()),
            coverage: MaskCoverage::FullSectionVoid,
        };
        let result = calculate(
            &t,
            &Constraints {
                masks: vec![mask],
                ..Default::default()
            },
            &profile(),
        )
        .unwrap();
        let node = result.blocks.iter().find(|b| b.kind == "node_T").unwrap();
        assert_eq!(node.length_centimm, 5_000);
        assert!(node
            .arms
            .iter()
            .any(|a| a.edge_id == "e" && a.length_centimm == 3_000));
        assert!(node.cuts.iter().any(|v| v.contains("distal")));
        assert!(node.source_ids.iter().any(|v| v == "beam:beam"));
        assert_eq!(covered(&result.blocks, "e"), 47_000);
    }
    #[test]
    fn partial_depth_beam_never_becomes_full_void() {
        let t = local_topology(
            vec![
                edge("w", "w", (-50_000, 0), (0, 0)),
                edge("e", "e", (0, 0), (50_000, 0)),
                edge("n", "n", (0, 0), (0, 50_000)),
            ],
            0,
        );
        let mask = Mask {
            run_id: "e".into(),
            course_index: 0,
            interval: Interval {
                start: 3_000,
                end: 6_000,
            },
            source: MaskSource::Beam("beam".into()),
            coverage: MaskCoverage::PartialDepth,
        };
        let errors = calculate(
            &t,
            &Constraints {
                masks: vec![mask],
                ..Default::default()
            },
            &profile(),
        )
        .unwrap_err();
        assert!(errors
            .iter()
            .any(|d| d.code == DiagnosticCode::UnsupportedGeometry
                && d.message.contains("beam:beam")));
    }
    #[test]
    fn non_640_length_is_covered_by_cut_ordinary() {
        let t = topology(vec![edge("e", "wall", (0, 0), (70_000, 0))], 0);
        let result = calculate(&t, &Constraints::default(), &profile()).unwrap();
        assert_eq!(covered(&result.blocks, "e"), 70_000);
        assert!(result
            .blocks
            .iter()
            .any(|b| b.hide_spikes_right && !b.hide_spikes_left));
    }
    #[test]
    fn straight_split_is_one_physical_run_without_i_node() {
        let a = edge("a", "wall", (0, 0), (32_000, 0));
        let b = edge("b", "wall", (32_000, 0), (64_000, 0));
        let mut t = topology(vec![a.clone(), b.clone()], 0);
        t.courses[0].runs = vec![WallRun {
            id: "run".into(),
            start: a.start,
            end: b.end,
            length: 64_000,
            sources: vec![
                RunSource {
                    wall_id: "wall".into(),
                    edge_id: "a".into(),
                    start: a.start,
                    end: a.end,
                    start_offset: 0,
                    end_offset: 32_000,
                },
                RunSource {
                    wall_id: "wall".into(),
                    edge_id: "b".into(),
                    start: b.start,
                    end: b.end,
                    start_offset: 32_000,
                    end_offset: 64_000,
                },
            ],
        }];
        let result = calculate(&t, &Constraints::default(), &profile()).unwrap();
        assert_eq!(result.blocks.len(), 1);
        assert_eq!(result.blocks[0].kind, "ordinary");
        assert_eq!(result.blocks[0].start, Point { x: 0, y: 0 });
        assert_eq!(result.blocks[0].end, Point { x: 64_000, y: 0 });
        assert!(result.blocks[0].source_ids.contains(&"edge:a".into()));
        assert!(result.blocks[0].source_ids.contains(&"edge:b".into()));
    }
    #[test]
    fn short_tail_uses_one_catalog_bounded_compensator() {
        let p = profile();
        assert_eq!(
            choose_pieces(64_500, &p, 0),
            Some(vec![(32_000, false), (32_500, true)])
        );
        assert_eq!(
            choose_pieces(64_500, &p, 1),
            Some(vec![(32_500, true), (32_000, false)])
        );
    }
    #[test]
    fn equal_vertex_phase_can_still_collide_across_a_run() {
        let mut p = profile();
        p.phase_count = 2;
        p.nodes = vec![NodeSpec {
            kind: "L".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 32_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 64_000,
                },
            ],
        }];
        let t = local_topology(
            vec![
                edge("main", "main", (0, 0), (64_000, 0)),
                edge("left", "left", (0, 0), (0, 100_000)),
                edge("right", "right", (64_000, 0), (64_000, 100_000)),
            ],
            0,
        );
        let errors = calculate(&t, &Constraints::default(), &p).unwrap_err();
        assert!(errors
            .iter()
            .any(|d| d.code == DiagnosticCode::PhaseConflict));
        p.nodes.push(NodeSpec {
            kind: "L".into(),
            phase: 1,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 64_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 32_000,
                },
            ],
        });
        let resolved = calculate(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(covered(&resolved.blocks, "main"), 64_000);
    }
    #[test]
    fn world_grid_checks_ordinary_joints_without_changing_catalog_arms() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        p.phase_count = 1;
        p.nodes = vec![NodeSpec {
            kind: "L".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 32_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 64_000,
                },
            ],
        }];
        for (x, k) in [(0, 0), (32_000, 0), (0, 41)] {
            let t = topology(
                vec![
                    edge("east", "east", (x, 0), (x + 100_000, 0)),
                    edge("north", "north", (x, 0), (x, 100_000)),
                ],
                k,
            );
            let selected = select_nodes(&t, &Constraints::default(), &p).unwrap();
            let node = &selected[&(k, Point { x, y: 0 })];
            let length = |direction| {
                node.geometry
                    .arms
                    .iter()
                    .find(|arm| arm.direction_deg == direction)
                    .unwrap()
                    .nominal_length_centimm
            };
            assert_eq!(length(0), 32_000);
            assert_eq!(length(90), 64_000);
        }
        assert_eq!(
            choose_pieces_on_span(0, 64_500, &p, 0, Some(32_000)),
            Some(vec![(32_000, false), (32_500, true)])
        );
        assert_eq!(choose_pieces_on_span(0, 64_500, &p, 0, Some(0)), None);
    }
    #[test]
    fn world_grid_reports_real_incompatible_ordinary_joint() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        let t = topology(vec![edge("run", "wall", (0, 0), (64_500, 0))], 1);
        let errors = calculate(&t, &Constraints::default(), &p).unwrap_err();
        assert!(errors
            .iter()
            .any(|d| d.code == DiagnosticCode::WorldGridIncompatible
                && d.message.contains("32000")
                && d.edge_id.as_deref() == Some("run")));
    }
    #[test]
    fn two_edge_compensators_keep_all_internal_joints_on_world_grid() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        p.max_ordinary_cuts_per_free_span = 2;
        assert_eq!(
            choose_pieces_on_span(212_000, 414_600, &p, 0, Some(32_000)),
            Some(vec![
                (12_000, true),
                (64_000, false),
                (64_000, false),
                (62_600, true)
            ])
        );
        let t = topology(vec![edge("run", "wall", (0, 0), (500_000, 0))], 0);
        let constraints = Constraints {
            masks: vec![
                Mask {
                    run_id: "run".into(),
                    course_index: 0,
                    interval: Interval {
                        start: 0,
                        end: 212_000,
                    },
                    source: MaskSource::Opening("left".into()),
                    coverage: MaskCoverage::FullSectionVoid,
                },
                Mask {
                    run_id: "run".into(),
                    course_index: 0,
                    interval: Interval {
                        start: 414_600,
                        end: 500_000,
                    },
                    source: MaskSource::Opening("right".into()),
                    coverage: MaskCoverage::FullSectionVoid,
                },
            ],
            ..Default::default()
        };
        let layout = calculate(&t, &constraints, &p).unwrap();
        let ordinary: Vec<_> = layout
            .blocks
            .iter()
            .filter(|b| b.kind == "ordinary")
            .collect();
        assert_eq!(ordinary.len(), 4);
        assert!(ordinary[0].hide_spikes_left && !ordinary[0].hide_spikes_right);
        assert!(ordinary[3].hide_spikes_right && !ordinary[3].hide_spikes_left);
        assert_eq!(
            ordinary.iter().map(|b| b.length_centimm).sum::<i64>(),
            202_600
        );
    }
    #[test]
    fn roof_tail_discards_tiny_piece_while_lower_courses_keep_rust() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        p.maximum_special_blank_centimm = Some(128_000);
        p.max_ordinary_cuts_per_free_span = 2;
        let mut t = topology(vec![edge("run", "wall", (0, 0), (192_015, 0))], 1);
        t.walls.push(crate::domain::NormalizedWall {
            id: "wall".into(),
            start: Point { x: 0, y: 0 },
            end: Point { x: 256_000, y: 0 },
            bottom_start: -100_000,
            bottom_end: -100_000,
            top_start: 198_315,
            top_end: -57_685,
            thickness: 20_000,
        });
        let mut roof = calculate(&t, &Constraints::default(), &p).unwrap();
        roof.blocks.sort_by_key(|b| b.start.x);
        assert_eq!(
            roof.blocks
                .iter()
                .map(|b| b.length_centimm)
                .collect::<Vec<_>>(),
            vec![64_000, 64_000, 64_000]
        );

        for (tail, expected_count) in [(1_999, 3), (2_000, 4)] {
            let mut boundary = topology(vec![edge("run", "wall", (0, 0), (192_000 + tail, 0))], 1);
            let mut wall = t.walls[0].clone();
            wall.top_start = 192_000 + tail + 6_300;
            wall.top_end = wall.top_start - 256_000;
            boundary.walls.push(wall);
            let output = calculate(&boundary, &Constraints::default(), &p).unwrap();
            assert_eq!(output.blocks.len(), expected_count);
            if tail == 2_000 {
                assert!(output.blocks.iter().any(|block| block.length_centimm == 2_000));
            }
        }

        // Та же стена в нижнем венце: верх не подрезает текущий материал.
        t.courses[0].z = 0;
        let mut lower = calculate(&t, &Constraints::default(), &p).unwrap();
        lower.blocks.sort_by_key(|b| b.start.x);
        assert_eq!(
            lower
                .blocks
                .iter()
                .map(|b| b.length_centimm)
                .collect::<Vec<_>>(),
            vec![64_000, 64_000, 64_000, 15]
        );
        assert!(lower.blocks.last().unwrap().hide_spikes_right);

        // Независимая короткая торцевая деталь продолжает снимать правый шип.
        let plain = topology(vec![edge("run", "wall", (0, 0), (15, 0))], 1);
        let ordinary = calculate(&plain, &Constraints::default(), &p).unwrap();
        assert_eq!(ordinary.blocks[0].length_centimm, 15);
        assert!(ordinary.blocks[0].hide_spikes_right);
    }

    #[test]
    fn roof_start_discards_short_fragment_without_absorbing_next_module() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        p.maximum_special_blank_centimm = Some(128_000);
        p.max_ordinary_cuts_per_free_span = 2;
        let mut t = topology(vec![edge("run", "wall", (0, 31_985), (192_015, 31_985))], 0);
        t.walls.push(crate::domain::NormalizedWall {
            id: "wall".into(),
            start: Point {
                x: -64_000,
                y: 31_985,
            },
            end: Point {
                x: 256_000,
                y: 31_985,
            },
            bottom_start: -100_000,
            bottom_end: -100_000,
            top_start: -64_000,
            top_end: 256_000,
            thickness: 20_000,
        });
        let roof = calculate(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(
            roof.blocks
                .iter()
                .map(|b| b.length_centimm)
                .collect::<Vec<_>>(),
            vec![64_000, 64_000, 64_000]
        );
        assert!(!roof.blocks[0].hide_spikes_left && !roof.blocks[0].hide_spikes_right);
        t.walls[0].top_start = 300_000;
        t.walls[0].top_end = 300_000;
        let flat = calculate(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(
            flat.blocks
                .iter()
                .map(|b| b.length_centimm)
                .collect::<Vec<_>>(),
            vec![15, 64_000, 64_000, 64_000]
        );
        assert!(flat.blocks[0].hide_spikes_left);
    }

    #[test]
    fn catalog_blank_absorbs_sub_minimum_grid_residuals() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        p.maximum_ordinary_blank_centimm = Some(65_000);
        assert_eq!(
            choose_pieces_on_span(0, 192_015, &p, 0, Some(15)),
            Some(vec![(64_015, true), (64_000, false), (64_000, false)])
        );
        assert_eq!(
            choose_pieces_on_span(32_000, 192_273, &p, 0, Some(0)),
            Some(vec![(32_000, false), (64_000, false), (64_273, true)])
        );
        let t = topology(vec![edge("run", "wall", (0, 0), (64_273, 0))], 1);
        let output = calculate(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(output.blocks.len(), 1);
        assert_eq!(output.blocks[0].catalog_nominal_centimm, Some(64_000));
        assert!(output.blocks[0].hide_spikes_right);
    }
    #[test]
    fn repeated_node_without_verified_vertical_rule_is_typed_refusal() {
        let mut p = profile();
        p.nodes = vec![NodeSpec {
            kind: "L".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 5_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 7_000,
                },
            ],
        }];
        let edges = vec![
            edge("east", "east", (1, 1), (30_001, 1)),
            edge("north", "north", (1, 1), (1, 30_001)),
        ];
        let mut t = topology(edges.clone(), 0);
        t.courses.extend(topology(edges, 1).courses);
        let errors = calculate(&t, &Constraints::default(), &p).unwrap_err();
        assert!(errors
            .iter()
            .any(|d| d.code == DiagnosticCode::NodeVerticalRuleMissing
                && d.course_index == Some(0)
                && d.source_ids.iter().any(|id| id == "east")));
        assert!(calculate_candidate(&t, &Constraints::default(), &p).is_err());
    }

    #[test]
    fn full_physical_plan_reports_coincident_ordinary_and_node_boundaries() {
        let mut p = profile();
        p.node_assembly_family = Some("forestbrick_legacy_observed".into());
        let e = edge("run", "wall", (0, 0), (100_000, 0));
        let mut t = topology(vec![e.clone()], 0);
        t.courses.extend(topology(vec![e.clone()], 1).courses);
        let blocks: Vec<_> = (0..2)
            .flat_map(|course| {
                [
                    block(
                        &e,
                        course,
                        course * 6300,
                        Span {
                            start: 0,
                            end: 32_000,
                        },
                        100_000,
                        "node_L",
                        false,
                        vec![],
                        vec!["wall:wall".into()],
                    ),
                    block(
                        &e,
                        course,
                        course * 6300,
                        Span {
                            start: 32_000,
                            end: 100_000,
                        },
                        100_000,
                        "ordinary",
                        false,
                        vec![],
                        vec!["wall:wall".into()],
                    ),
                ]
            })
            .collect();
        let errors = validate_vertical_plan(&t, &blocks, &p);
        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0].code, DiagnosticCode::VerticalJointConflict);
        assert_eq!(errors[0].course_index, Some(0));
        assert_eq!(errors[0].source_ids, vec!["wall:wall"]);
    }

    #[test]
    fn physical_l_candidates_alternate_real_distal_joints() {
        let mut p = profile();
        p.node_assembly_family = Some("forestbrick_legacy_observed".into());
        p.nodes = (0..2)
            .map(|phase| NodeSpec {
                kind: "L".into(),
                phase,
                variant: None,
                rays: vec![
                    RayLength {
                        direction_deg: 0,
                        length_centimm: if phase == 0 { 32_000 } else { 64_000 },
                    },
                    RayLength {
                        direction_deg: 90,
                        length_centimm: if phase == 0 { 64_000 } else { 32_000 },
                    },
                ],
            })
            .collect();
        let edges = vec![
            edge("east", "east", (0, 0), (100_000, 0)),
            edge("north", "north", (0, 0), (0, 100_000)),
        ];
        let mut t = topology(edges.clone(), 0);
        t.courses.extend(topology(edges, 1).courses);
        let selected = select_nodes(&t, &Constraints::default(), &p).unwrap();
        let point = Point { x: 0, y: 0 };
        let below = node_joint_evidence(
            &selected[&(0, point)],
            point,
            &t.courses[0],
            &Constraints::default(),
            &p,
        )
        .unwrap();
        let above = node_joint_evidence(
            &selected[&(1, point)],
            point,
            &t.courses[1],
            &Constraints::default(),
            &p,
        )
        .unwrap();
        assert_eq!(below.joints.len(), 2);
        assert_eq!(above.joints.len(), 2);
        assert_eq!(
            vertical::check_adjacent_nodes(&below, &above, 1),
            VerticalVerdict::Clear
        );
        assert_ne!(selected[&(0, point)].phase, selected[&(1, point)].phase);
    }

    #[test]
    fn candidate_keeps_complete_geometry_with_real_warning_while_strict_refuses() {
        let mut p = profile();
        p.node_assembly_family = Some("forestbrick_legacy_observed".into());
        p.ordinary_length_centimm = 64_000;
        p.ordinary_nominal_lengths_centimm = vec![64_000];
        let edges = vec![edge("straight", "wall", (0, 0), (128_000, 0))];
        let mut t = topology(edges.clone(), 0);
        t.courses.extend(topology(edges, 1).courses);
        let candidate = calculate_candidate(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(candidate.layout.blocks.len(), 4);
        assert_eq!(candidate.diagnostics.len(), 1);
        assert_eq!(
            candidate.diagnostics[0].code,
            DiagnosticCode::VerticalJointConflict
        );
        assert_eq!(candidate.diagnostics[0].course_index, Some(0));
        assert!(candidate.diagnostics[0]
            .message
            .contains("(64000,0) / (64000,0)"));
        assert_eq!(
            calculate(&t, &Constraints::default(), &p).unwrap_err(),
            candidate.diagnostics
        );
    }

    #[test]
    fn candidate_relaxes_only_vertical_constraints_and_keeps_node_geometry() {
        let mut p = profile();
        p.node_assembly_family = Some("forestbrick_legacy_observed".into());
        p.nodes = vec![NodeSpec {
            kind: "L".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 32_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 64_000,
                },
            ],
        }];
        let edges = vec![
            edge("east", "east", (0, 0), (100_000, 0)),
            edge("north", "north", (0, 0), (0, 100_000)),
        ];
        let mut t = topology(edges.clone(), 0);
        t.courses.extend(topology(edges, 1).courses);
        let candidate = calculate_candidate(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(
            candidate
                .layout
                .blocks
                .iter()
                .filter(|block| block.kind == "node_L")
                .count(),
            4
        );
        assert!(!candidate.diagnostics.is_empty());
        assert!(candidate
            .diagnostics
            .iter()
            .all(|issue| issue.code == DiagnosticCode::VerticalJointConflict));
        assert!(calculate(&t, &Constraints::default(), &p).is_err());
    }

    #[test]
    fn forced_t_joint_does_not_discard_independent_l_alternation() {
        let mut p = profile();
        p.phase_count = 2;
        p.ordinary_length_centimm = 64_000;
        p.ordinary_nominal_lengths_centimm = vec![32_000, 64_000];
        p.minimum_cut_centimm = 20_000;
        p.node_assembly_family = Some("forestbrick_legacy_observed".into());
        p.nodes = (0..2)
            .map(|phase| NodeSpec {
                kind: "L".into(),
                phase,
                variant: None,
                rays: vec![
                    RayLength {
                        direction_deg: 0,
                        length_centimm: if phase == 0 { 32_000 } else { 64_000 },
                    },
                    RayLength {
                        direction_deg: 90,
                        length_centimm: if phase == 0 { 64_000 } else { 32_000 },
                    },
                ],
            })
            .collect();
        p.nodes.push(NodeSpec {
            kind: "T".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 32_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 64_000,
                },
                RayLength {
                    direction_deg: 180,
                    length_centimm: 32_000,
                },
            ],
        });
        p.nodes.push(NodeSpec {
            kind: "T".into(),
            phase: 1,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 64_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 32_000,
                },
                RayLength {
                    direction_deg: 180,
                    length_centimm: 64_000,
                },
            ],
        });
        let edges = vec![
            edge("l-east", "l-east", (0, 0), (100_000, 0)),
            edge("l-north", "l-north", (0, 0), (0, 100_000)),
            edge("t-west", "t-west", (100_000, 200_000), (200_000, 200_000)),
            edge("t-east", "t-east", (200_000, 200_000), (300_000, 200_000)),
            edge("t-stem", "t-stem", (200_000, 200_000), (200_000, 280_000)),
        ];
        let mut t = topology(edges.clone(), 0);
        t.courses.extend(topology(edges, 1).courses);
        let selected = select_nodes(&t, &Constraints::default(), &p).unwrap();
        let l_point = Point { x: 0, y: 0 };
        assert_ne!(selected[&(0, l_point)].phase, selected[&(1, l_point)].phase);
        let t_point = Point {
            x: 200_000,
            y: 200_000,
        };
        assert_eq!(selected[&(0, t_point)].phase, 1);
        assert_eq!(selected[&(1, t_point)].phase, 1);
        let candidate = calculate_candidate(&t, &Constraints::default(), &p).unwrap();
        assert!(!candidate.diagnostics.is_empty());
        assert!(candidate.diagnostics.iter().all(|issue| issue
            .source_ids
            .iter()
            .all(|id| !id.contains("l-east") && !id.contains("l-north"))));
    }

    #[test]
    fn t_distal_joint_respects_world_phase_and_does_not_repeat_ordinary_joint_above() {
        let mut p = profile();
        p.phase_count = 2;
        p.node_assembly_family = Some("forestbrick_legacy_observed".into());
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        p.ordinary_length_centimm = 64_000;
        p.ordinary_nominal_lengths_centimm = vec![32_000, 64_000];
        p.nodes = vec![
            NodeSpec {
                kind: "T".into(),
                phase: 0,
                variant: None,
                rays: vec![
                    RayLength {
                        direction_deg: 0,
                        length_centimm: 32_000,
                    },
                    RayLength {
                        direction_deg: 90,
                        length_centimm: 64_000,
                    },
                    RayLength {
                        direction_deg: 180,
                        length_centimm: 32_000,
                    },
                ],
            },
            NodeSpec {
                kind: "T".into(),
                phase: 1,
                variant: None,
                rays: vec![
                    RayLength {
                        direction_deg: 0,
                        length_centimm: 64_000,
                    },
                    RayLength {
                        direction_deg: 90,
                        length_centimm: 32_000,
                    },
                    RayLength {
                        direction_deg: 180,
                        length_centimm: 64_000,
                    },
                ],
            },
        ];
        let edges = vec![
            edge("west", "west", (224_000, 672_000), (320_000, 672_000)),
            edge("east", "east", (320_000, 672_000), (416_000, 672_000)),
            edge("stem", "stem", (320_000, 672_000), (320_000, 768_000)),
        ];
        let mut t = topology(edges.clone(), 0);
        t.courses.extend(topology(edges, 1).courses);
        let candidate = calculate_candidate(&t, &Constraints::default(), &p).unwrap();
        assert!(
            candidate.diagnostics.is_empty(),
            "{:?}",
            candidate.diagnostics
        );
        let node = Point {
            x: 320_000,
            y: 672_000,
        };
        let selected = select_nodes(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(selected[&(0, node)].phase, 1);
        assert_eq!(selected[&(1, node)].phase, 0);
        assert!(calculate(&t, &Constraints::default(), &p).is_ok());

        // Скос верхней стены режет только distal-конец 640-мм заготовки.
        // Номинал и фиксированный стык остаются теми же; фаза не блокируется.
        let upper_edges = vec![
            edge("west", "west", (224_000, 672_000), (320_000, 672_000)),
            edge("east", "east", (320_000, 672_000), (416_000, 672_000)),
            edge("stem", "stem", (320_000, 672_000), (320_000, 732_149)),
        ];
        t.courses[1] = topology(upper_edges, 1).courses.remove(0);
        let clipped = calculate_candidate(&t, &Constraints::default(), &p).unwrap();
        assert!(clipped
            .diagnostics
            .iter()
            .any(|issue| issue.code == DiagnosticCode::NodeDistalWallTrim));
        assert!(clipped
            .diagnostics
            .iter()
            .all(|issue| issue.code != DiagnosticCode::VerticalJointConflict));
        let stem = clipped
            .layout
            .blocks
            .iter()
            .find(|block| {
                block.course_index == 1 && block.product_key.as_deref() == Some("Type5_1")
            })
            .unwrap();
        assert_eq!(stem.catalog_nominal_centimm, Some(64_000));
        assert_eq!(stem.arms[0].length_centimm, 60_149);
        assert_eq!(stem.product_key.as_deref(), Some("Type5_1"));
        assert!(stem.cuts.iter().any(|cut| cut.starts_with("Type5_1:")));
        let selected = select_nodes(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(selected[&(0, node)].phase, 1);
        assert_eq!(selected[&(1, node)].phase, 0);
    }
    #[test]
    fn t0_short_stem_requires_original_320_run() {
        let mut p = profile();
        p.nodes = vec![
            NodeSpec {
                kind: "T".into(),
                phase: 0,
                variant: None,
                rays: vec![
                    RayLength {
                        direction_deg: 0,
                        length_centimm: 32_000,
                    },
                    RayLength {
                        direction_deg: 90,
                        length_centimm: 64_000,
                    },
                    RayLength {
                        direction_deg: 180,
                        length_centimm: 32_000,
                    },
                ],
            },
            NodeSpec {
                kind: "T".into(),
                phase: 0,
                variant: Some("t0_short".into()),
                rays: vec![
                    RayLength {
                        direction_deg: 0,
                        length_centimm: 32_000,
                    },
                    RayLength {
                        direction_deg: 90,
                        length_centimm: 32_000,
                    },
                    RayLength {
                        direction_deg: 180,
                        length_centimm: 32_000,
                    },
                ],
            },
        ];
        let make = |stem: i64| {
            local_topology(
                vec![
                    edge("west", "west", (-100_000, 0), (0, 0)),
                    edge("east", "east", (0, 0), (100_000, 0)),
                    edge("stem", "stem", (0, 0), (0, stem)),
                ],
                0,
            )
        };
        let t = make(32_000);
        let selected = select_nodes(&t, &Constraints::default(), &p).unwrap();
        assert_eq!(
            selected[&(0, Point { x: 1, y: 1 })].variant.as_deref(),
            Some("t0_short")
        );
        let errors = match select_nodes(&make(33_000), &Constraints::default(), &p) {
            Err(errors) => errors,
            Ok(_) => panic!("short stem was accepted off 320"),
        };
        assert!(errors
            .iter()
            .any(|d| d.code == DiagnosticCode::UnsupportedCatalog
                && d.message.contains("требуется ровно 32000")));
    }
    #[test]
    fn overlapping_opening_and_beam_are_subtracted_once() {
        let t = topology(vec![edge("e", "wall", (0, 0), (100_000, 0))], 0);
        let masks = vec![
            Mask {
                run_id: "e".into(),
                course_index: 0,
                interval: Interval {
                    start: 20_000,
                    end: 40_000,
                },
                source: MaskSource::Opening("o".into()),
                coverage: MaskCoverage::FullSectionVoid,
            },
            Mask {
                run_id: "e".into(),
                course_index: 0,
                interval: Interval {
                    start: 30_000,
                    end: 50_000,
                },
                source: MaskSource::Beam("b".into()),
                coverage: MaskCoverage::FullSectionVoid,
            },
        ];
        let result = calculate(
            &t,
            &Constraints {
                masks,
                ..Default::default()
            },
            &profile(),
        )
        .unwrap();
        assert_eq!(covered(&result.blocks, "e"), 70_000);
        assert!(result
            .blocks
            .iter()
            .all(|b| !((b.start.x < 50_000) && (b.end.x > 20_000))));
    }
    #[test]
    fn lintel_requires_both_solid_supports_and_catalog_length() {
        let t = topology(vec![edge("e", "wall", (0, 0), (100_000, 0))], 0);
        let lintel = LintelCandidate {
            opening_id: "opening".into(),
            run_id: "e".into(),
            course_index: 0,
            span: Interval {
                start: 40_000,
                end: 60_000,
            },
            left_support: Some(Interval {
                start: 35_000,
                end: 40_000,
            }),
            right_support: Some(Interval {
                start: 60_000,
                end: 65_000,
            }),
        };
        let constraints = Constraints {
            lintel_candidates: vec![lintel.clone()],
            ..Default::default()
        };
        let mut p = profile();
        p.bridge_nominal_lengths_centimm = vec![64_000];
        let result = calculate(&t, &constraints, &p).unwrap();
        assert_eq!(covered(&result.blocks, "e"), 100_000);
        assert_eq!(result.blocks.iter().filter(|b| b.is_bridge).count(), 1);
        let mut bad = lintel;
        bad.right_support = None;
        let err = calculate(
            &t,
            &Constraints {
                lintel_candidates: vec![bad],
                ..Default::default()
            },
            &p,
        )
        .unwrap_err();
        assert!(err.iter().any(|d| d.code == DiagnosticCode::InvalidSupport));
    }

    #[test]
    fn t_lintel_replaces_longitudinal_parts_and_shifts_away_from_type10_1() {
        use crate::constraints::{build_constraints, RawOpening};
        use crate::domain::{NormalizedWall, RawPoint};
        let e = edge("e", "wall", (0, 0), (320_000, 0));
        let mut t = topology(vec![e.clone()], 0);
        t.walls.push(NormalizedWall {
            id: "wall".into(),
            start: e.start,
            end: e.end,
            bottom_start: 0,
            bottom_end: 0,
            top_start: 25_200,
            top_end: 25_200,
            thickness: 19_300,
        });
        let template = t.courses[0].clone();
        t.courses = (0..4)
            .map(|index| {
                let mut c = template.clone();
                c.index = index;
                c.z = index * 6_300;
                c
            })
            .collect();
        let c = build_constraints(
            &t,
            &[RawOpening {
                id: "o".into(),
                start: RawPoint {
                    x_mm: 1200.0,
                    y_mm: 0.0,
                },
                end: RawPoint {
                    x_mm: 2000.0,
                    y_mm: 0.0,
                },
                bottom_start_mm: 0.0,
                bottom_end_mm: 0.0,
                top_start_mm: 63.0,
                top_end_mm: 63.0,
            }],
            &[],
            63.0,
            200.0,
        )
        .unwrap();
        let mut p = profile();
        p.lintel_support_mm = 200.0;
        p.bridge_nominal_lengths_centimm = vec![128_000];
        let parts = |course, stem: &str| {
            let mut parts = Vec::new();
            for (index, (start, end)) in [
                (96_000, 128_000),
                (128_000, 160_000),
                (160_000, 192_000),
                (192_000, 224_000),
            ]
            .into_iter()
            .enumerate()
            {
                let mut part = block(
                    &e,
                    course,
                    course * 6_300,
                    Span { start, end },
                    320_000,
                    "ordinary",
                    false,
                    Vec::new(),
                    vec![format!("source-{index}")],
                );
                if index == 1 || index == 2 {
                    part.kind = "node_T".into();
                    part.product_key = Some("Type6".into());
                    part.arms.push(BlockArm {
                        edge_id: "e".into(),
                        wall_id: "wall".into(),
                        start: part.start,
                        end: part.end,
                        length_centimm: end - start,
                    });
                    part.cuts.push(format!(
                        "Type6:x{}:y{}:e",
                        if index == 1 { 2 } else { 1 },
                        if index == 1 { 1 } else { 3 }
                    ));
                }
                parts.push(part);
            }
            let mut stem_part = block(
                &e,
                course,
                course * 6_300,
                Span {
                    start: 160_000,
                    end: 192_000,
                },
                320_000,
                "node_T",
                false,
                Vec::new(),
                vec!["stem-source".into()],
            );
            stem_part.product_key = Some(stem.into());
            stem_part.start = Point { x: 160_000, y: 0 };
            stem_part.end = stem_part.start;
            stem_part.arms.push(BlockArm {
                edge_id: "stem".into(),
                wall_id: "stem".into(),
                start: stem_part.start,
                end: Point {
                    x: 160_000,
                    y: 32_000,
                },
                length_centimm: 32_000,
            });
            parts.push(stem_part);
            parts
        };
        let mut blocked = parts(1, "Type10_1");
        assert!(merge_t_lintels(&mut blocked, &c, &p).is_empty());
        assert!(blocked.iter().all(|b| !b.is_bridge));
        for offset in [0, 1] {
            let mut blocks = if offset == 0 {
                parts(1, "Type5_1")
            } else {
                let mut blocks = parts(1, "Type10_1");
                blocks.extend(parts(2, "Type5_1"));
                blocks
            };
            let completed = merge_t_lintels(&mut blocks, &c, &p);
            assert!(completed.contains(&(1, "o".into())));
            let bridge = blocks.iter().find(|b| b.is_bridge).unwrap();
            assert_eq!(bridge.course_index, 1 + offset);
            assert_eq!(
                (bridge.start.x, bridge.end.x, bridge.length_centimm),
                (96_000, 224_000, 128_000)
            );
            assert!(bridge.cuts.contains(&"Type6:x3:y1:e".into()));
            assert!(bridge.cuts.contains(&"Type6:x3:y3:e".into()));
            assert_eq!(bridge.source_ids.len(), 5);
            assert_eq!(
                blocks
                    .iter()
                    .filter(|b| b.course_index == bridge.course_index)
                    .count(),
                2
            );
            assert!(blocks.iter().any(|b| b.course_index == bridge.course_index
                && b.product_key.as_deref() == Some("Type5_1")));
            if offset == 1 {
                assert_eq!(blocks.iter().filter(|b| b.course_index == 1).count(), 5);
            }
        }
        // 149 мм недостаточно: склеивается ещё одна деталь с каждой стороны.
        // Ровно 150 мм допустимо даже при ошибочно малом профильном минимуме.
        p.lintel_support_mm = 50.0;
        p.bridge_nominal_lengths_centimm = vec![256_000];
        let chain = |course, boundaries: &[i64]| -> Vec<Block> {
            boundaries.windows(2).map(|pair| block(
                &e, course, course * 6_300,
                Span { start: pair[0], end: pair[1] }, 320_000,
                "ordinary", false, Vec::new(), Vec::new(),
            )).collect()
        };
        for (boundaries, expected) in [
            (vec![73_100, 105_100, 214_900, 246_900], (73_100, 246_900)),
            (vec![73_000, 105_000, 215_000, 247_000], (105_000, 215_000)),
        ] {
            let mut blocks = chain(1, &boundaries);
            assert!(merge_t_lintels(&mut blocks, &c, &p).contains(&(1, "o".into())));
            let bridge = blocks.iter().find(|b| b.is_bridge).unwrap();
            assert_eq!((bridge.start.x, bridge.end.x), expected);
        }
        let mut blocks = chain(1, &[73_100, 105_100, 214_900, 246_900]);
        blocks.extend(chain(2, &[73_000, 105_000, 215_000, 247_000]));
        merge_t_lintels(&mut blocks, &c, &p);
        let bridge = blocks.iter().find(|b| b.is_bridge).unwrap();
        assert_eq!(bridge.course_index, 2);
        assert_eq!(bridge.length_centimm, 110_000);

        let mut too_high = chain(3, &[105_000, 215_000]);
        let mut first_candidate = c.clone();
        first_candidate.lintel_candidates.retain(|candidate| candidate.course_index == 1);
        assert!(merge_t_lintels(&mut too_high, &first_candidate, &p).is_empty());

        // Обычная перемычка проходит тот же путь склейки после раскладки.
        let output = calculate(&t, &c, &p).unwrap();
        let bridge = output.blocks.iter().find(|b| b.is_bridge).unwrap();
        assert!(bridge.start.x <= 105_000 && bridge.end.x >= 215_000);
        assert!(bridge.length_centimm > 110_000);

        let nearby = build_constraints(
            &t,
            &[("left", 1200.0, 1400.0), ("right", 1600.0, 1800.0)].map(|(id, start, end)| RawOpening {
                id: id.into(),
                start: RawPoint { x_mm: start, y_mm: 0.0 },
                end: RawPoint { x_mm: end, y_mm: 0.0 },
                bottom_start_mm: 0.0, bottom_end_mm: 0.0,
                top_start_mm: 63.0, top_end_mm: 63.0,
            }),
            &[], 63.0, 150.0,
        ).unwrap();
        let output = calculate(&t, &nearby, &p).unwrap();
        let bridges: Vec<_> = output.blocks.iter().filter(|block| block.is_bridge).collect();
        assert!(!bridges.is_empty());
        assert!(bridges.iter().all(|block| block.source_ids.contains(&"left".into())
            && block.source_ids.contains(&"right".into())));

    }

    #[test]
    fn real_640_pier_splits_glued_lintels_only_at_next_course_existing_joint() {
        use crate::constraints::{build_constraints, RawOpening};
        use crate::domain::{NormalizedWall, RawPoint};
        let e = edge("e", "wall", (1_408_000, 576_000), (1_408_000, 992_000));
        let mut t = topology(vec![e.clone()], 38);
        t.courses[0].z = 214_200;
        let mut below = t.courses[0].clone();
        below.index = 37;
        below.z = 207_900;
        t.courses.push(below);
        let mut next = t.courses[0].clone();
        next.index = 39;
        next.z = 220_500;
        t.courses.push(next);
        t.walls.push(NormalizedWall {
            id: "wall".into(), start: e.start, end: e.end,
            bottom_start: -25_200, bottom_end: -25_200,
            top_start: 300_000, top_end: 300_000, thickness: 19_300,
        });
        let constraints = build_constraints(&t,
            &[("lower", 7040.0, 6066.0), ("upper", 8654.0, 7680.0)].map(|(id, start, end)| RawOpening {
                id: id.into(),
                start: RawPoint { x_mm: 14080.0, y_mm: start },
                end: RawPoint { x_mm: 14080.0, y_mm: end },
                bottom_start_mm: 0.0, bottom_end_mm: 0.0,
                top_start_mm: 2142.0, top_end_mm: 2142.0,
            }), &[], 63.0, 150.0).unwrap();
        assert_eq!(constraints.lintel_candidates.len(), 2);
        let parts = |course, boundaries: &[i64]| -> Vec<Block> {
            boundaries.windows(2).map(|pair| block(
                &e, course, course * 6_300 - 25_200,
                Span { start: pair[0] - 576_000, end: pair[1] - 576_000 },
                416_000, "ordinary", false, Vec::new(), Vec::new(),
            )).collect()
        };
        let mut p = profile();
        p.lintel_support_mm = 150.0;
        p.bridge_nominal_lengths_centimm = vec![275_000];
        let mut current = parts(38, &[576_000, 640_000, 704_000, 768_000, 832_000, 896_000, 960_000, 992_000]);
        assert!(merge_t_lintels(&mut current, &constraints, &p).is_empty());
        assert!(current.iter().all(|block| !block.is_bridge));
        let mut blocks = current;
        blocks.extend(parts(39, &[576_000, 608_000, 672_000, 736_000, 800_000, 864_000, 928_000, 992_000]));
        let completed = merge_t_lintels(&mut blocks, &constraints, &p);
        assert!(completed.contains(&(38, "lower".into())) && completed.contains(&(38, "upper".into())));
        let mut bridges: Vec<_> = blocks.iter().filter(|block| block.is_bridge).collect();
        bridges.sort_by_key(|block| block.start.y.min(block.end.y));
        assert_eq!(bridges.len(), 2);
        assert!(bridges.iter().all(|block| block.course_index == 39));
        assert_eq!((bridges[0].start.y.min(bridges[0].end.y), bridges[0].start.y.max(bridges[0].end.y), bridges[0].length_centimm), (576_000, 736_000, 160_000));
        assert_eq!((bridges[1].start.y.min(bridges[1].end.y), bridges[1].start.y.max(bridges[1].end.y), bridges[1].length_centimm), (736_000, 928_000, 192_000));
        assert!(bridges[0].source_ids.contains(&"lower".into()) && !bridges[0].source_ids.contains(&"upper".into()));
        assert!(bridges[1].source_ids.contains(&"upper".into()) && !bridges[1].source_ids.contains(&"lower".into()));
        assert_eq!(blocks.iter().filter(|block| block.course_index == 38).count(), 7);
        assert_eq!(covered(&blocks, "e"), 832_000);
    }

    #[test]
    fn narrow_pier_is_included_once_and_unavailable_maximum_is_a_warning() {
        let t = topology(vec![edge("e", "wall", (0, 0), (140_000, 0))], 0);
        let candidate = |id: &str, start, end| LintelCandidate {
            opening_id: id.into(),
            run_id: "e".into(),
            course_index: 0,
            span: Interval { start, end },
            left_support: Some(Interval {
                start: start - 5_000,
                end: start,
            }),
            right_support: Some(Interval {
                start: end,
                end: end + 5_000,
            }),
        };
        let c = Constraints {
            lintel_candidates: vec![
                candidate("a", 40_000, 60_000),
                candidate("b", 80_000, 100_000),
            ],
            ..Default::default()
        };
        let mut p = profile();
        p.bridge_nominal_lengths_centimm = vec![128_000];
        let result = calculate(&t, &c, &p).unwrap();
        let bridges: Vec<_> = result.blocks.iter().filter(|b| b.is_bridge).collect();
        assert_eq!(bridges.len(), 1);
        assert_eq!(bridges[0].length_centimm, 90_000);
        assert!(
            bridges[0].source_ids.contains(&"a".into())
                && bridges[0].source_ids.contains(&"b".into())
        );
        assert_eq!(covered(&result.blocks, "e"), 140_000);
        p.bridge_nominal_lengths_centimm = vec![30_000];
        let fallback = calculate_candidate(&t, &c, &p).unwrap();
        assert!(fallback.layout.blocks.iter().all(|b| !b.is_bridge));
        assert_eq!(covered(&fallback.layout.blocks, "e"), 140_000);
        assert!(fallback
            .diagnostics
            .iter()
            .any(|d| d.code == DiagnosticCode::InvalidSupport));
    }
    #[test]
    fn canonical_l_catalog_rotates_with_geometry() {
        let mut p = profile();
        p.nodes.push(NodeSpec {
            kind: "L".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 5_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 7_000,
                },
            ],
        });
        let t = local_topology(
            vec![
                edge("north", "n", (0, 0), (0, 30_000)),
                edge("west", "w", (-30_000, 0), (0, 0)),
            ],
            0,
        );
        let result = calculate(&t, &Constraints::default(), &p).unwrap();
        let node = result.blocks.iter().find(|b| b.kind == "node_L").unwrap();
        assert_eq!(node.arms.len(), 2);
        assert!(node
            .arms
            .iter()
            .any(|a| a.edge_id == "north" && a.length_centimm == 5_000));
        assert!(node
            .arms
            .iter()
            .any(|a| a.edge_id == "west" && a.length_centimm == 7_000));
    }
    #[test]
    fn legacy_family_materializes_two_physical_l_parts() {
        let mut p = profile();
        p.node_assembly_family = Some("forestbrick_legacy_observed".into());
        p.nodes = vec![NodeSpec {
            kind: "L".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 32_000,
                },
                RayLength {
                    direction_deg: 90,
                    length_centimm: 64_000,
                },
            ],
        }];
        let t = topology(
            vec![
                edge("east", "east", (0, 0), (100_000, 0)),
                edge("north", "north", (0, 0), (0, 100_000)),
            ],
            0,
        );
        let result = calculate(&t, &Constraints::default(), &p).unwrap();
        let parts: Vec<_> = result
            .blocks
            .iter()
            .filter(|b| b.kind == "node_L")
            .collect();
        assert_eq!(parts.len(), 2);
        assert!(parts
            .iter()
            .all(|p| p.product_key.is_some() && p.catalog_status == "KnownPattern"));
        assert_eq!(covered(&result.blocks, "east"), 100_000);
        assert_eq!(covered(&result.blocks, "north"), 100_000);
    }
    #[test]
    fn rotated_y_keeps_one_owner_and_both_rays() {
        let mut p = profile();
        p.nodes.push(NodeSpec {
            kind: "Y".into(),
            phase: 0,
            variant: None,
            rays: vec![
                RayLength {
                    direction_deg: 0,
                    length_centimm: 5_000,
                },
                RayLength {
                    direction_deg: 135,
                    length_centimm: 6_000,
                },
            ],
        });
        let t = local_topology(
            vec![
                edge("n", "n", (0, 0), (0, 30_000)),
                edge("sw", "sw", (-30_000, -30_000), (0, 0)),
            ],
            0,
        );
        let result = calculate(&t, &Constraints::default(), &p).unwrap();
        let nodes: Vec<_> = result
            .blocks
            .iter()
            .filter(|b| b.kind == "node_Y")
            .collect();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].arms.len(), 2);
        for id in ["n", "sw"] {
            assert!(nodes[0].arms.iter().any(|a| a.edge_id == id));
        }
    }
    #[test]
    fn short_fragments_never_absorb_a_long_neighbour_or_remove_150mm_rust() {
        let mut p = profile();
        p.minimum_cut_centimm = 15_000;
        p.maximum_special_blank_centimm = Some(128_000);
        let pieces = choose_pieces_with_roof_ends(0, 74_000, &p, 0, Some(10_000), false, false).unwrap();
        assert_eq!(pieces.iter().map(|p| p.0).collect::<Vec<_>>(), vec![10_000, 64_000]);
        let pieces = choose_pieces_with_roof_ends(0, 79_000, &p, 0, Some(15_000), false, false).unwrap();
        assert_eq!(pieces.iter().map(|p| p.0).collect::<Vec<_>>(), vec![15_000, 64_000]);
    }

    #[test]
    fn finalize_beam_removes_rear_and_vertical_bridges_and_creates_two_parts() {
        let request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
            "schema_version":1,"request_id":"split","snapshot_hash":"a".repeat(64),"z0_mm":0,
            "wall_volumes":[{"guid":"w","startXmm":0,"startYmm":0,"endXmm":640,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],
            "beams":[{"guid":"b","startXmm":320,"startYmm":-500,"startZmm":20,
                "endXmm":320,"endYmm":76.5,"endZmm":20,
                "geometry":{"widthMm":160,"heightMm":20,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}]
        })).unwrap();
        let e = edge("e", "w", (0, 0), (64_000, 0));
        let mut input = block(&e, 0, 0, Span {start:0,end:64_000}, 64_000, "ordinary", false, Vec::new(), vec!["wall:w".into()]);
        input.wall_id = "run:0:0:64000:0".into();
        let parts = finalize_parts(&request, &profile(), &[input]).unwrap();
        assert_eq!(parts.len(), 2);
        assert_eq!(parts.iter().map(|p|p.length_centimm).collect::<Vec<_>>(), vec![24_000, 24_000]);
        assert_eq!(parts[0].end.x, 24_000);
        assert_eq!(parts[1].start.x, 40_000);
        assert!(!parts[0].natural_end_right && parts[0].hide_spikes_right);
        assert!(!parts[1].natural_end_left && parts[1].hide_spikes_left);
        assert!(parts[0].natural_end_left && parts[0].hide_spikes_left);
        assert!(parts[1].natural_end_right && parts[1].hide_spikes_right);
        assert!(parts.iter().all(|p| p.catalog_nominal_centimm == Some(p.length_centimm) && p.cuts.is_empty()));
        assert_eq!(finalize_parts(&request, &profile(), &parts).unwrap(), parts);
    }

    #[test]
    fn finalize_untouched_neighbour_keeps_spikes_and_clears_inherited_beam_hide() {
        let request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
            "schema_version":1,"request_id":"neighbour","snapshot_hash":"b".repeat(64),"z0_mm":0,
            "wall_volumes":[{"guid":"w","startXmm":0,"startYmm":0,"endXmm":1280,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}]
        })).unwrap();
        let e = edge("e", "w", (0, 0), (128_000, 0));
        let mut input = block(&e, 0, 0, Span {start:32_000,end:96_000}, 128_000, "ordinary", false, vec!["beam_volume:b".into()], Vec::new());
        input.hide_spikes_left = true;
        input.hide_spikes_right = true;
        let parts = finalize_parts(&request, &profile(), &[input]).unwrap();
        assert!(!parts[0].hide_spikes_left && !parts[0].hide_spikes_right);
        assert!(parts[0].natural_end_left && parts[0].natural_end_right);
    }

    #[test]
    fn finalize_normalizes_only_exact_encoded_five_mm_beam_end() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        for end in [0.0, 5.0] {
            let request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
                "schema_version":1,"request_id":"nominal","snapshot_hash":"c".repeat(64),"z0_mm":0,
                "wall_volumes":[{"guid":"w","startXmm":-640,"startYmm":0,"endXmm":640,"endYmm":0,
                    "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],
                "beams":[{"guid":"b","startXmm":-640,"startYmm":0,"startZmm":0,
                    "endXmm":end,"endYmm":0,"endZmm":0,
                    "geometry":{"widthMm":193,"heightMm":63,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}]
            })).unwrap();
            let e = edge("e", "w", (0, 0), (64_000, 0));
            let input = block(&e, 0, 0, Span {start:0,end:64_000}, 64_000, "ordinary", false, Vec::new(), Vec::new());
            let parts = finalize_parts(&request, &p, &[input]).unwrap();
            assert_eq!(parts.len(), 1);
            assert_eq!(parts[0].length_centimm, 64_000);
            assert_eq!(parts[0].start.x, 0);
            assert!(parts[0].natural_end_left && parts[0].hide_spikes_left);
            assert_eq!(request.beams[0].end_xmm, end);
        }
    }

    #[test]
    fn finalize_rejects_deep_component_tree_before_recursive_overflow() {
        let request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
            "schema_version":1,"request_id":"depth","snapshot_hash":"d".repeat(64),"z0_mm":0,
            "wall_volumes":[{"guid":"w","startXmm":0,"startYmm":0,"endXmm":1280,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}]
        })).unwrap();
        let e = edge("e", "w", (0, 0), (64_000, 0));
        let template = block(&e, 0, 0, Span {start:0,end:64_000}, 64_000, "ordinary", false, Vec::new(), Vec::new());
        let mut nested = template.clone();
        for _ in 0..18 {
            let mut parent = template.clone();
            parent.components = vec![nested];
            nested = parent;
        }
        let error = finalize_parts(&request, &profile(), &[nested]).unwrap_err();
        assert_eq!(error.code, "GEOMETRY_LIMIT");
    }

    #[test]
    fn factory_320_and_640_ends_touching_opening_remove_five_once() {
        for length in [32_000, 64_000] {
            let mut request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
                "schema_version":1,"request_id":"opening-contact","snapshot_hash":"e".repeat(64),"z0_mm":0,
                "wall_volumes":[{"guid":"w","startXmm":-640,"startYmm":0,"endXmm":1280,"endYmm":0,
                    "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],
                "opening_volumes":[{"guid":"o","openingType":"WINDOW",
                    "startXmm":length as f64 / 100.0,"startYmm":0,"endXmm":length as f64 / 100.0+160.0,"endYmm":0,
                    "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63}]
            })).unwrap();
            let e = edge("e", "w", (0, 0), (length, 0));
            let input = block(&e, 0, 0, Span {start:0,end:length}, length, "ordinary", false, Vec::new(), vec!["wall:w".into()]);
            let parts = finalize_parts(&request, &profile(), &[input.clone()]).unwrap();
            assert_eq!(parts.len(), 1);
            assert_eq!(parts[0].length_centimm, length);
            assert!(parts[0].natural_end_right && parts[0].hide_spikes_right);
            assert!(parts[0].natural_end_left && !parts[0].hide_spikes_left);
            let rendered = crate::materialize::materialize(&request, &parts, &profile()).unwrap();
            let volume: f64 = rendered.physical.blocks.iter().flat_map(|part| &part.bodies)
                .map(crate::solid_geometry::volume).sum();
            assert!((volume - (length as f64 / 100.0 - 5.0) * 193.0 * 63.0).abs() < 1e-4);

            // Разрез внутри исходного изделия не получает дополнительного -5.
            request.opening_volumes[0].start_xmm = 100.0;
            request.opening_volumes[0].end_xmm = 260.0;
            let parts = finalize_parts(&request, &profile(), &[input]).unwrap();
            assert_eq!(parts.len(), 2);
            assert!(!parts[0].natural_end_right && parts[0].hide_spikes_right);
            assert!(!parts[1].natural_end_left && parts[1].hide_spikes_left);
            let rendered = crate::materialize::materialize(&request, &parts, &profile()).unwrap();
            let volume: f64 = rendered.physical.blocks.iter().flat_map(|part| &part.bodies)
                .map(crate::solid_geometry::volume).sum();
            assert!((volume - (length as f64 / 100.0 - 160.0) * 193.0 * 63.0).abs() < 1e-4);
        }
    }

    #[test]
    fn type8_fragment_320_has_straight_artificial_end_at_opening() {
        let request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
            "schema_version":1,"request_id":"type8-contact","snapshot_hash":"f".repeat(64),"z0_mm":0,
            "wall_volumes":[{"guid":"w","startXmm":-640,"startYmm":0,"endXmm":1280,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],
            "opening_volumes":[{"guid":"o","openingType":"OPENING","startXmm":320,"startYmm":0,"endXmm":1280,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63}]
        })).unwrap();
        let e = edge("e", "w", (0, 0), (64_000, 0));
        let mut input = block(&e, 0, 0, Span {start:0,end:64_000}, 64_000, "node_T", false,
            vec!["Type8:x1:y1:e".into()], vec!["wall:w".into()]);
        input.product_key = Some("Type8".into());
        input.catalog_nominal_centimm = Some(64_000);
        let parts = finalize_parts(&request, &profile(), &[input]).unwrap();
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].length_centimm, 32_000);
        assert!(parts[0].natural_end_left && !parts[0].hide_spikes_left);
        assert!(!parts[0].natural_end_right && parts[0].hide_spikes_right);
        let rendered = crate::materialize::materialize(&request, &parts, &profile()).unwrap();
        let rightmost = rendered.physical.blocks.iter().flat_map(|part| &part.bodies)
            .flat_map(|mesh| &mesh.vertices).map(|point| point[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!((rightmost - 320.0).abs() < 1e-6);
    }

    #[test]
    fn mixed_opening_reveal_aligns_short_artificial_node_once() {
        let request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
            "schema_version":1,"request_id":"mixed-reveal","snapshot_hash":"f".repeat(64),"z0_mm":0,
            "wall_volumes":[{"guid":"w","startXmm":-640,"startYmm":0,"endXmm":1280,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":126,"endTopZmm":126,"thicknessMm":193,"purposeType":1}],
            "opening_volumes":[{"guid":"o","openingType":"WINDOW","startXmm":320,"startYmm":0,"endXmm":1280,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":126,"endTopZmm":126}]
        })).unwrap();
        let e = edge("e", "w", (0, 0), (64_000, 0));
        let mut short = block(&e, 0, 0, Span {start:0,end:64_000}, 64_000, "node_T", false,
            vec!["Type8:x1:y1:e".into()], vec!["wall:w".into()]);
        short.product_key = Some("Type8".into());
        let mut reference = block(&e, 1, 6300, Span {start:0,end:64_000}, 64_000, "ordinary", false,
            Vec::new(), vec!["wall:w".into()]);
        reference.start.x = -32_000;
        reference.end.x = 32_000;
        let parts = finalize_parts(&request, &profile(), &[short.clone(), reference]).unwrap();
        assert_eq!(parts[0].length_centimm, 31_500);
        assert_eq!(parts[0].end.x, 31_500);
        assert!(!parts[0].natural_end_right && parts[0].hide_spikes_right);
        assert!(parts[0].cuts.iter().any(|cut| cut.starts_with("Type8:p0:")));
        assert_eq!(parts[1].length_centimm, 64_000);
        assert_eq!(parts[1].end.x, 32_000);
        assert!(parts[1].natural_end_right && parts[1].hide_spikes_right);
        assert_eq!(finalize_parts(&request, &profile(), &parts).unwrap(), parts);
        let rendered = crate::materialize::materialize(&request, &parts, &profile()).unwrap();
        for physical in &rendered.physical.blocks {
            let max = physical.bodies.iter().flat_map(|body| &body.vertices)
                .map(|point| point[0]).fold(f64::NEG_INFINITY, f64::max);
            assert!((max - 315.0).abs() < 1e-6);
        }
        // Без натуральной опорной плоскости все искусственные торцы остаются номинальными.
        let alone = finalize_parts(&request, &profile(), &[short.clone()]).unwrap();
        assert_eq!(alone[0].length_centimm, 32_000);
        let mut long_artificial = parts[1].clone();
        long_artificial.id = "long-artificial".into();
        long_artificial.natural_end_right = false;
        let both = finalize_parts(&request, &profile(), &[parts[1].clone(), long_artificial]).unwrap();
        assert_eq!(both[1].length_centimm, 64_000);
        assert_eq!(both[1].end.x, 32_000);
        for purpose in ["DOOR", "OPENING"] {
            let mut other_request = request.clone();
            other_request.opening_volumes[0].opening_type = purpose.into();
            let aligned = finalize_parts(&other_request, &profile(), &[short.clone(), parts[1].clone()]).unwrap();
            assert_eq!(aligned[0].length_centimm, 31_500);
            assert_eq!(aligned[1].length_centimm, 64_000);
            assert_eq!(finalize_parts(&other_request, &profile(), &aligned).unwrap(), aligned);
        }
    }

    #[test]
    fn reversed_b07_and_forward_b21_remove_factory_spikes_at_both_gap_edges() {
        let mut p = profile();
        p.world_joint_policy = Some("affine_checkerboard_v1".into());
        let request: crate::api::LayoutRequest = serde_json::from_value(serde_json::json!({
            "schema_version":1,"request_id":"b21-b07-contact","snapshot_hash":"7".repeat(64),"z0_mm":-252,
            "wall_volumes":[{"guid":"w","startXmm":0,"startYmm":320,"endXmm":8000,"endYmm":320,
                "startBottomZmm":-252,"endBottomZmm":-252,"startTopZmm":3000,"endTopZmm":3000,"thicknessMm":193,"purposeType":1}],
            "beams":[{"guid":"B21","startXmm":-896.5,"startYmm":320,"startZmm":2268,"endXmm":1920,"endYmm":320,"endZmm":2268,
                "geometry":{"widthMm":160,"heightMm":315,"heightDirectionX":-6.7962199403762088e-17,"heightDirectionY":-1.689582373646205e-32,"heightDirectionZ":1}},
                {"guid":"B07","startXmm":7685,"startYmm":320,"startZmm":2268,"endXmm":3200,"endYmm":320,"endZmm":2268,
                "geometry":{"widthMm":160,"heightMm":315,"heightDirectionX":4.6648026244754483e-17,"heightDirectionY":-5.16443349516189e-34,"heightDirectionZ":1}}]
        })).unwrap();
        let e = edge("e", "w", (0, 32_000), (800_000, 32_000));
        let left = block(&e, 44, 252_000, Span {start:192_000,end:256_000}, 800_000, "ordinary", false, Vec::new(), vec!["wall:w".into()]);
        let right = block(&e, 44, 252_000, Span {start:256_000,end:320_000}, 800_000, "ordinary", false, Vec::new(), vec!["wall:w".into()]);
        let parts = finalize_parts(&request, &p, &[left, right]).unwrap();
        assert_eq!(parts.iter().map(|part| part.length_centimm).collect::<Vec<_>>(), vec![64_000,64_000]);
        assert!(parts[0].natural_end_left && parts[0].hide_spikes_left);
        assert!(!parts[0].hide_spikes_right && !parts[1].hide_spikes_left);
        assert!(parts[1].natural_end_right && parts[1].hide_spikes_right);
        let physical = crate::materialize::materialize(&request, &parts, &p).unwrap().physical;
        let leftmost = physical.blocks[0].bodies.iter().flat_map(|mesh| &mesh.vertices)
            .map(|point| point[0]).fold(f64::INFINITY, f64::min);
        let rightmost = physical.blocks[1].bodies.iter().flat_map(|mesh| &mesh.vertices)
            .map(|point| point[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!((leftmost - 1925.0).abs() < 1e-6);
        assert!((rightmost - 3195.0).abs() < 1e-6);
    }
}

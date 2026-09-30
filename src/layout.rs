//! Детерминированная раскладка по курсовым ребрам; координаты в 0,01 мм.

use crate::choice::{self, BinaryConstraint, Problem, SolveError, UnaryConstraint, Variable};
use crate::constraints::{Constraints, Exclusion, Interval, MaskCoverage, MaskSource, SolidBox};
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
    pub cuts: Vec<String>,
    pub source_ids: Vec<String>,
    pub catalog_nominal_centimm: Option<i64>,
    #[serde(default)]
    pub arms: Vec<BlockArm>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<Block>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BlockArm {
    pub wall_id: String,
    pub edge_id: String,
    pub start: Point,
    pub end: Point,
    pub length_centimm: i64,
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
    let result = if profile.full_course_clearance
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
            if remaining < profile.minimum_cut_centimm {
                let Some(last) = result.last_mut() else {
                    // Материал задан моделью. Фрагмент остаётся в кандидате,
                    // а производственный минимум проверяется предупреждением.
                    result.push((remaining, true));
                    break;
                };
                if last.0 + remaining > special_limit {
                    return None;
                }
                last.0 += remaining;
                last.1 = !profile.ordinary_nominal_lengths_centimm.contains(&last.0);
                break;
            }
            let mut distance = (residue - at).rem_euclid(profile.ordinary_length_centimm);
            if distance == 0 {
                distance = profile.ordinary_length_centimm;
            }
            if distance < profile.minimum_cut_centimm {
                distance += profile.ordinary_length_centimm;
            }
            let piece = if remaining <= profile.ordinary_length_centimm {
                remaining
            } else {
                distance.min(remaining)
            };
            if piece > special_limit {
                return None;
            }
            result.push((
                piece,
                !profile.ordinary_nominal_lengths_centimm.contains(&piece),
            ));
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
        cuts,
        source_ids: sources,
        arms: Vec::new(),
        catalog_nominal_centimm: None,
        components: Vec::new(),
    }
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
                            hide_spikes_left: hide_left,
                            hide_spikes_right: hide_right,
                            cuts,
                            source_ids: sources,
                            arms: part_arms,
                            catalog_nominal_centimm: Some(part.nominal_length_centimm),
                            components: Vec::new(),
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
                        cuts: geometry.cuts.clone(),
                        source_ids: geometry.source_ids.clone(),
                        arms: node_arms,
                        catalog_nominal_centimm: None,
                        components: Vec::new(),
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
            let expected_solid = solid.clone();
            let mut placed = Vec::new();
            for lintel in constraints
                .lintel_candidates
                .iter()
                .filter(|v| v.course_index == course.index && v.run_id == edge.edge_id)
            {
                let Some(left) = lintel.left_support else {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Нет левой опоры перемычки",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                };
                let Some(right) = lintel.right_support else {
                    errors.push(diagnostic(
                        DiagnosticCode::InvalidSupport,
                        "Нет правой опоры перемычки",
                        Some(edge),
                        Some(course.index),
                    ));
                    continue;
                };
                let bridge = Span {
                    start: left.start,
                    end: right.end,
                };
                if left.end != lintel.span.start
                    || right.start != lintel.span.end
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
                let nominal = profile
                    .bridge_nominal_lengths_centimm
                    .iter()
                    .copied()
                    .filter(|v| *v >= bridge.len())
                    .min();
                let Some(nominal) = nominal else {
                    errors.push(diagnostic(
                        DiagnosticCode::UnsupportedCatalog,
                        "Длина перемычки превышает подтверждённый каталожный номинал",
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
                    vec![lintel.opening_id.clone()],
                );
                item.catalog_nominal_centimm = Some(nominal);
                blocks.push(item);
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
                let Some(pieces) =
                    choose_pieces_on_span(run.start, run.end, profile, phase, residue)
                else {
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
                    match classify_candidate(topology, constraints, course, edge, current, profile)
                    {
                        Ok(Exclusion::None) => {}
                        Ok(Exclusion::Partial { .. }) => {}
                        Ok(Exclusion::FullVoid { source_ids }) => {
                            errors.push(diagnostic(
                                DiagnosticCode::UnsupportedGeometry,
                                format!(
                                    "Ordinary требует 3D-подрезки из-за {:?} на run {} [{}, {})",
                                    source_ids, edge.edge_id, current.start, current.end
                                ),
                                Some(edge),
                                Some(course.index),
                            ));
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
        }
    }
    let mut support_warnings = Vec::new();
    errors.retain(|issue| {
        if issue.code==DiagnosticCode::InvalidSupport {
            let mut warning=issue.clone();
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
        let result = calculate(&t, &constraints, &profile()).unwrap();
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
            &profile(),
        )
        .unwrap_err();
        assert!(err.iter().any(|d| d.code == DiagnosticCode::InvalidSupport));
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
}

//! Чистая проверка междетальных швов двух соседних венцов.
//! Она не доказывает несущую способность опоры или геометрию тела изделия.

use crate::constraints::Interval;
use crate::domain::{Point, WallRun};
use crate::node_assembly::PhysicalNodePart;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct WallAxis {
    pub dx: i8,
    pub dy: i8,
    pub line: i128,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MaterialSegment {
    pub axis: WallAxis,
    pub start: Point,
    pub end: Point,
    pub source_ids: Vec<String>,
    pub part_index: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InterpartJoint {
    pub axis: WallAxis,
    pub at: Point,
    pub source_ids: Vec<String>,
    pub part_indices: (usize, usize),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JointEvidence {
    pub segments: Vec<MaterialSegment>,
    pub joints: Vec<InterpartJoint>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum VerticalError {
    MissingEvidence {
        reason: &'static str,
        run_id: Option<String>,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum VerticalVerdict {
    Clear,
    Conflict {
        axis: WallAxis,
        lower_at: Point,
        upper_at: Point,
        lower_source_ids: Vec<String>,
        upper_source_ids: Vec<String>,
    },
    MissingEvidence {
        reason: &'static str,
    },
}

fn axis(run: &WallRun) -> Option<WallAxis> {
    let dx = i128::from(run.end.x) - i128::from(run.start.x);
    let dy = i128::from(run.end.y) - i128::from(run.start.y);
    if (dx == 0 && dy == 0) || (dx != 0 && dy != 0 && dx.abs() != dy.abs()) {
        return None;
    }
    let mut sx = dx.signum() as i8;
    let mut sy = dy.signum() as i8;
    if sx < 0 || (sx == 0 && sy < 0) {
        sx = -sx;
        sy = -sy;
    }
    Some(WallAxis {
        dx: sx,
        dy: sy,
        line: i128::from(sy) * i128::from(run.start.x) - i128::from(sx) * i128::from(run.start.y),
    })
}

/// Сегмент опубликованной физической детали; все её плечи имеют один part_index.
pub fn material_segment(
    run: &WallRun,
    a: Point,
    b: Point,
    part_index: usize,
    source_ids: &[String],
) -> Result<MaterialSegment, VerticalError> {
    let axis = axis(run).ok_or_else(|| VerticalError::MissingEvidence {
        reason: "Нет достоверной оси физической детали",
        run_id: Some(run.id.clone()),
    })?;
    let on_axis = |p: Point| {
        i128::from(axis.dy) * i128::from(p.x) - i128::from(axis.dx) * i128::from(p.y) == axis.line
    };
    if !on_axis(a) || !on_axis(b) || a == b {
        return Err(VerticalError::MissingEvidence {
            reason: "Сегмент детали не лежит на оси WallRun",
            run_id: Some(run.id.clone()),
        });
    }
    let (start, end) = if scalar(axis, a) <= scalar(axis, b) {
        (a, b)
    } else {
        (b, a)
    };
    Ok(MaterialSegment {
        axis,
        start,
        end,
        part_index,
        source_ids: source_ids.to_vec(),
    })
}

fn scalar(axis: WallAxis, point: Point) -> i128 {
    i128::from(axis.dx) * i128::from(point.x) + i128::from(axis.dy) * i128::from(point.y)
}

fn point_at(run: &WallRun, offset: i64) -> Option<Point> {
    if run.length <= 0 || offset < 0 || offset > run.length {
        return None;
    }
    if offset == 0 {
        return Some(run.start);
    }
    if offset == run.length {
        return Some(run.end);
    }
    let t = offset as f64 / run.length as f64;
    Some(Point {
        x: (run.start.x as f64 + (i128::from(run.end.x) - i128::from(run.start.x)) as f64 * t)
            .round() as i64,
        y: (run.start.y as f64 + (i128::from(run.end.y) - i128::from(run.start.y)) as f64 * t)
            .round() as i64,
    })
}

fn segment(
    run: &WallRun,
    interval: Interval,
    part_index: usize,
    source_ids: &[String],
) -> Result<MaterialSegment, VerticalError> {
    let fail = || VerticalError::MissingEvidence {
        reason: "Нет достоверной оси или интервала WallRun",
        run_id: Some(run.id.clone()),
    };
    if interval.start >= interval.end {
        return Err(fail());
    }
    let axis = axis(run).ok_or_else(fail)?;
    let a = point_at(run, interval.start).ok_or_else(fail)?;
    let b = point_at(run, interval.end).ok_or_else(fail)?;
    let (start, end) = if scalar(axis, a) <= scalar(axis, b) {
        (a, b)
    } else {
        (b, a)
    };
    if scalar(axis, start) >= scalar(axis, end) {
        return Err(fail());
    }
    Ok(MaterialSegment {
        axis,
        start,
        end,
        source_ids: source_ids.to_vec(),
        part_index,
    })
}

fn extract_parts_evidence(
    parts: &[PhysicalNodePart],
    runs: &[WallRun],
) -> Result<JointEvidence, VerticalError> {
    let by_id: BTreeMap<_, _> = runs.iter().map(|run| (run.id.as_str(), run)).collect();
    let mut segments = Vec::new();
    for (part_index, part) in parts.iter().enumerate() {
        if part.covered_arms.is_empty() {
            return Err(VerticalError::MissingEvidence {
                reason: "У физической детали нет покрытых плеч",
                run_id: None,
            });
        }
        for arm in &part.covered_arms {
            let run =
                by_id
                    .get(arm.run_id.as_str())
                    .ok_or_else(|| VerticalError::MissingEvidence {
                        reason: "Нет WallRun для покрытого плеча",
                        run_id: Some(arm.run_id.clone()),
                    })?;
            segments.push(segment(run, arm.interval, part_index, &part.source_ids)?);
        }
    }
    if segments.is_empty() {
        return Err(VerticalError::MissingEvidence {
            reason: "Нет физического материала для проверки",
            run_id: None,
        });
    }
    extract_segments_evidence(segments)
}

/// Источник швов — границы разных деталей, а не границы аналитических плеч.
pub fn extract_segments_evidence(
    mut segments: Vec<MaterialSegment>,
) -> Result<JointEvidence, VerticalError> {
    segments.sort_by_key(|segment| {
        (
            segment.axis,
            segment.part_index,
            scalar(segment.axis, segment.start),
        )
    });
    let mut merged: Vec<MaterialSegment> = Vec::new();
    for segment in segments {
        if let Some(last) = merged.last_mut() {
            if last.axis == segment.axis
                && last.part_index == segment.part_index
                && scalar(last.axis, segment.start) <= scalar(last.axis, last.end)
            {
                if scalar(last.axis, segment.end) > scalar(last.axis, last.end) {
                    last.end = segment.end;
                }
                last.source_ids.extend(segment.source_ids);
                last.source_ids.sort();
                last.source_ids.dedup();
                continue;
            }
        }
        merged.push(segment);
    }
    let mut segments = merged;
    segments.sort_by_key(|segment| {
        (
            segment.axis,
            scalar(segment.axis, segment.start),
            scalar(segment.axis, segment.end),
            segment.part_index,
        )
    });
    let mut joints = Vec::new();
    for pair in segments.windows(2) {
        let (left, right) = (&pair[0], &pair[1]);
        if left.axis != right.axis {
            continue;
        }
        let end = scalar(left.axis, left.end);
        let start = scalar(right.axis, right.start);
        if end > start && left.part_index != right.part_index {
            return Err(VerticalError::MissingEvidence {
                reason: "Физические детали перекрываются на одной оси",
                run_id: None,
            });
        }
        if end != start || left.part_index == right.part_index {
            continue;
        }
        let source_ids = left
            .source_ids
            .iter()
            .chain(&right.source_ids)
            .cloned()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        joints.push(InterpartJoint {
            axis: left.axis,
            at: left.end,
            source_ids,
            part_indices: (left.part_index, right.part_index),
        });
    }
    Ok(JointEvidence { segments, joints })
}

/// Извлекает только реальные междетальные швы узла после coalescence.
pub fn extract_node_evidence(
    vertex: Point,
    parts: &[PhysicalNodePart],
    runs: &[WallRun],
) -> Result<JointEvidence, VerticalError> {
    let evidence = extract_parts_evidence(parts, runs)?;
    if !evidence
        .segments
        .iter()
        .any(|segment| segment.start == vertex || segment.end == vertex)
    {
        return Err(VerticalError::MissingEvidence {
            reason: "Покрытые плечи не достигают вершины узла",
            run_id: None,
        });
    }
    Ok(evidence)
}

/// Извлекает швы всех переданных физических деталей плана венца.
pub fn extract_full_plan_evidence(
    parts: &[PhysicalNodePart],
    runs: &[WallRun],
) -> Result<JointEvidence, VerticalError> {
    extract_parts_evidence(parts, runs)
}

fn material_ranges(evidence: &JointEvidence) -> BTreeMap<WallAxis, Vec<(i128, i128)>> {
    let mut axes = BTreeMap::<WallAxis, Vec<(i128, i128)>>::new();
    for segment in &evidence.segments {
        axes.entry(segment.axis).or_default().push((
            scalar(segment.axis, segment.start),
            scalar(segment.axis, segment.end),
        ));
    }
    for ranges in axes.values_mut() {
        ranges.sort_unstable();
        let mut merged: Vec<(i128, i128)> = Vec::new();
        for &(start, end) in ranges.iter() {
            if let Some(last) = merged.last_mut() {
                if start <= last.1 {
                    last.1 = last.1.max(end);
                    continue;
                }
            }
            merged.push((start, end));
        }
        *ranges = merged;
    }
    axes
}

/// Сравнивает швы лишь внутри общей материальной области соседних венцов.
pub fn check_adjacent_nodes(
    lower: &JointEvidence,
    upper: &JointEvidence,
    min_joint_offset_centimm: i64,
) -> VerticalVerdict {
    check_adjacent_material(lower, upper, min_joint_offset_centimm, true)
}

/// Полные планы могут иметь непересекающиеся области материала.
pub fn check_adjacent_plans(
    lower: &JointEvidence,
    upper: &JointEvidence,
    min_joint_offset_centimm: i64,
) -> VerticalVerdict {
    check_adjacent_material(lower, upper, min_joint_offset_centimm, false)
}

fn check_adjacent_material(
    lower: &JointEvidence,
    upper: &JointEvidence,
    min_joint_offset_centimm: i64,
    require_common_material: bool,
) -> VerticalVerdict {
    if min_joint_offset_centimm < 0 || lower.segments.is_empty() || upper.segments.is_empty() {
        return VerticalVerdict::MissingEvidence {
            reason: "Нет сегментов или неверное минимальное смещение",
        };
    }
    let lower_ranges = material_ranges(lower);
    let upper_ranges = material_ranges(upper);
    let mut common = BTreeMap::<WallAxis, Vec<(i128, i128)>>::new();
    for (axis, below) in &lower_ranges {
        if let Some(above) = upper_ranges.get(axis) {
            for &(a, b) in below {
                for &(c, d) in above {
                    let overlap = (a.max(c), b.min(d));
                    if overlap.0 < overlap.1 {
                        common.entry(*axis).or_default().push(overlap);
                    }
                }
            }
        }
    }
    if common.is_empty() {
        if !require_common_material {
            return VerticalVerdict::Clear;
        }
        return VerticalVerdict::MissingEvidence {
            reason: "Нет общей материальной области соседних венцов",
        };
    }
    let threshold = i128::from(min_joint_offset_centimm);
    for below in &lower.joints {
        for above in &upper.joints {
            if below.axis != above.axis {
                continue;
            }
            let a = scalar(below.axis, below.at);
            let b = scalar(above.axis, above.at);
            if !common.get(&below.axis).is_some_and(|ranges| {
                ranges
                    .iter()
                    .any(|&(start, end)| start < a && a < end && start < b && b < end)
            }) {
                continue;
            }
            let dx = i128::from(below.at.x) - i128::from(above.at.x);
            let dy = i128::from(below.at.y) - i128::from(above.at.y);
            let Some(squared) = dx
                .checked_mul(dx)
                .and_then(|x| dy.checked_mul(dy).and_then(|y| x.checked_add(y)))
            else {
                return VerticalVerdict::MissingEvidence {
                    reason: "Переполнение расстояния между швами",
                };
            };
            if squared == 0 || squared < threshold * threshold {
                return VerticalVerdict::Conflict {
                    axis: below.axis,
                    lower_at: below.at,
                    upper_at: above.at,
                    lower_source_ids: below.source_ids.clone(),
                    upper_source_ids: above.source_ids.clone(),
                };
            }
        }
    }
    VerticalVerdict::Clear
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node_assembly::{CatalogStatus, CoveredArm, ProductKey};

    fn point(x: i64, y: i64) -> Point {
        Point { x, y }
    }

    fn run(id: &str, start: Point, end: Point, length: i64) -> WallRun {
        WallRun {
            id: id.into(),
            start,
            end,
            length,
            sources: Vec::new(),
        }
    }

    fn part(product: ProductKey, arms: &[(&str, i64, i64)]) -> PhysicalNodePart {
        PhysicalNodePart {
            product,
            catalog_status: CatalogStatus::KnownPattern,
            local_origin: point(0, 0),
            local_rotation_deg: 0,
            world_origin: point(0, 0),
            world_rotation_deg: 0,
            nominal_length_centimm: 64_000,
            covered_arms: arms
                .iter()
                .map(|&(id, start, end)| CoveredArm {
                    run_id: id.into(),
                    interval: Interval { start, end },
                    nominal_length_centimm: end - start,
                    source_ids: vec![format!("wall:{id}")],
                })
                .collect(),
            cuts: Vec::new(),
            source_ids: arms
                .iter()
                .map(|&(id, _, _)| format!("wall:{id}"))
                .collect(),
        }
    }

    fn t_runs() -> Vec<WallRun> {
        vec![
            run("west", point(-400, 0), point(0, 0), 400),
            run("east", point(0, 0), point(400, 0), 400),
            run("stem", point(0, 0), point(0, 400), 400),
        ]
    }

    fn t0_cross(short_stem: bool) -> Vec<PhysicalNodePart> {
        vec![
            part(ProductKey::Type6, &[("west", 0, 400), ("east", 0, 400)]),
            part(
                if short_stem {
                    ProductKey::Type5
                } else {
                    ProductKey::Type5_1
                },
                &[("stem", 0, if short_stem { 200 } else { 400 })],
            ),
        ]
    }

    fn t1_split() -> Vec<PhysicalNodePart> {
        vec![
            part(ProductKey::Type8, &[("west", 0, 400)]),
            part(ProductKey::Type8_1, &[("east", 0, 400)]),
            part(ProductKey::Type10_1, &[("stem", 0, 400)]),
        ]
    }

    #[test]
    fn t0_type6_cross_has_no_virtual_joint_against_t1_split() {
        let runs = t_runs();
        let lower = extract_node_evidence(point(0, 0), &t0_cross(false), &runs).unwrap();
        let upper = extract_node_evidence(point(0, 0), &t1_split(), &runs).unwrap();
        assert!(lower.joints.is_empty());
        assert_eq!(upper.joints.len(), 1);
        assert_eq!(upper.joints[0].at, point(0, 0));
        assert_eq!(
            check_adjacent_nodes(&lower, &upper, 100),
            VerticalVerdict::Clear
        );
    }

    #[test]
    fn identical_t1_split_conflicts_at_real_interpart_joint() {
        let evidence = extract_node_evidence(point(0, 0), &t1_split(), &t_runs()).unwrap();
        assert!(matches!(
            check_adjacent_nodes(&evidence, &evidence, 100),
            VerticalVerdict::Conflict {
                lower_at: Point { x: 0, y: 0 },
                upper_at: Point { x: 0, y: 0 },
                ..
            }
        ));
    }

    #[test]
    fn short_type5_stem_free_end_is_not_joint() {
        let evidence = extract_node_evidence(point(0, 0), &t0_cross(true), &t_runs()).unwrap();
        assert!(evidence.joints.is_empty());
        assert_eq!(
            check_adjacent_nodes(&evidence, &evidence, 100),
            VerticalVerdict::Clear
        );
    }

    #[test]
    fn l_corner_does_not_create_joint_between_perpendicular_axes() {
        let runs = vec![
            run("east", point(0, 0), point(400, 0), 400),
            run("north", point(0, 0), point(0, 400), 400),
        ];
        let parts = vec![
            part(ProductKey::Type1, &[("east", 0, 400)]),
            part(ProductKey::Type2, &[("north", 0, 400)]),
        ];
        let evidence = extract_node_evidence(point(0, 0), &parts, &runs).unwrap();
        assert!(evidence.joints.is_empty());
        assert_eq!(
            check_adjacent_nodes(&evidence, &evidence, 100),
            VerticalVerdict::Clear
        );
    }

    #[test]
    fn gap_and_absent_common_material_are_missing_evidence() {
        let runs = vec![run("r", point(0, 0), point(1_000, 0), 1_000)];
        let lower = extract_full_plan_evidence(
            &[
                part(ProductKey::Type8, &[("r", 0, 200)]),
                part(ProductKey::Type8_1, &[("r", 300, 500)]),
            ],
            &runs,
        )
        .unwrap();
        assert!(lower.joints.is_empty()); // void/free edge is not an interpart joint
        let upper =
            extract_full_plan_evidence(&[part(ProductKey::Type6, &[("r", 600, 800)])], &runs)
                .unwrap();
        assert!(matches!(
            check_adjacent_nodes(&lower, &upper, 100),
            VerticalVerdict::MissingEvidence { .. }
        ));
    }

    #[test]
    fn minimum_joint_offset_uses_physical_distance() {
        let runs = vec![run("r", point(0, 0), point(1_000, 0), 1_000)];
        let lower = extract_full_plan_evidence(
            &[
                part(ProductKey::Type8, &[("r", 0, 500)]),
                part(ProductKey::Type8_1, &[("r", 500, 1_000)]),
            ],
            &runs,
        )
        .unwrap();
        let upper = extract_full_plan_evidence(
            &[
                part(ProductKey::Type8, &[("r", 0, 600)]),
                part(ProductKey::Type8_1, &[("r", 600, 1_000)]),
            ],
            &runs,
        )
        .unwrap();
        assert!(matches!(
            check_adjacent_nodes(&lower, &upper, 101),
            VerticalVerdict::Conflict { .. }
        ));
        assert_eq!(
            check_adjacent_nodes(&lower, &upper, 100),
            VerticalVerdict::Clear
        );
    }

    #[test]
    fn node_to_ordinary_joint_is_real_and_detected_in_full_plan() {
        let runs = t_runs();
        let mut parts = t0_cross(true);
        parts.push(part(ProductKey::Type8, &[("stem", 200, 400)]));
        let evidence = extract_full_plan_evidence(&parts, &runs).unwrap();
        assert_eq!(evidence.joints.len(), 1);
        assert_eq!(evidence.joints[0].at, point(0, 200));
        assert!(matches!(
            check_adjacent_nodes(&evidence, &evidence, 1),
            VerticalVerdict::Conflict {
                lower_at: Point { x: 0, y: 200 },
                ..
            }
        ));
    }

    #[test]
    fn overlapping_analytical_segments_of_one_part_do_not_hide_boundary_joint() {
        let runs = vec![run("r", point(0, 0), point(1_000, 0), 1_000)];
        let parts = vec![
            part(ProductKey::Type6, &[("r", 0, 400), ("r", 200, 600)]),
            part(ProductKey::Type8, &[("r", 600, 1_000)]),
        ];
        let evidence = extract_full_plan_evidence(&parts, &runs).unwrap();
        assert_eq!(evidence.segments.len(), 2);
        assert_eq!(evidence.joints.len(), 1);
        assert_eq!(evidence.joints[0].at, point(600, 0));
    }
}

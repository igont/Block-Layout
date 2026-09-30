//! Разрешение масок на фиксированных плечах узла до размещения блоков.
//! Все интервалы полуоткрытые и заданы в канонической оси WallRun, в 0,01 мм.

use crate::constraints::{Interval, Mask, MaskCoverage, MaskSource};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NodeRayInput {
    pub run_id: String,
    pub direction_deg: u16,
    pub at_start: bool,
    pub run_length_centimm: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NodeArmInput {
    pub ray: NodeRayInput,
    pub nominal_length_centimm: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ResolvedArm {
    pub run_id: String,
    pub direction_deg: u16,
    pub at_start: bool,
    pub active_interval: Option<Interval>,
    pub nominal_length_centimm: i64,
    pub cut_at_distal_end: bool,
    pub cut_source_ids: Vec<String>,
}

/// Кандидат после исключения сквозных масок. Карманы балок сохраняют интервал
/// и требуют точного вычитания физического объёма при материализации.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AllowedSegment {
    pub run_id: String,
    pub interval: Interval,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NodeGeometry {
    pub arms: Vec<ResolvedArm>,
    pub allowed_segments: Vec<AllowedSegment>,
    pub source_ids: Vec<String>,
    pub cuts: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NodeGeometryError {
    InvalidMask {
        run_id: String,
        source_id: String,
        interval: Interval,
    },
    UnsupportedNodeCut {
        run_id: String,
        source_id: String,
        interval: Interval,
        reason: &'static str,
    },
}

fn source_id(source: &MaskSource) -> String {
    match source {
        MaskSource::Opening(id) => format!("opening:{id}"),
        MaskSource::Beam(id) => format!("beam:{id}"),
    }
}

fn on_ray(mask: &Mask, ray: &NodeRayInput) -> Interval {
    if ray.at_start {
        mask.interval
    } else {
        Interval {
            start: ray.run_length_centimm - mask.interval.end,
            end: ray.run_length_centimm - mask.interval.start,
        }
    }
}

fn checked_masks<'a>(
    course_index: i64,
    ray: &NodeRayInput,
    masks: &'a [Mask],
) -> Result<Vec<(&'a Mask, Interval)>, NodeGeometryError> {
    let mut relevant = Vec::new();
    for mask in masks
        .iter()
        .filter(|mask| mask.course_index == course_index && mask.run_id == ray.run_id)
    {
        if ray.run_length_centimm <= 0
            || mask.interval.start < 0
            || mask.interval.start >= mask.interval.end
            || mask.interval.end > ray.run_length_centimm
        {
            return Err(NodeGeometryError::InvalidMask {
                run_id: ray.run_id.clone(),
                source_id: source_id(&mask.source),
                interval: mask.interval,
            });
        }
        relevant.push((mask, on_ray(mask, ray)));
    }
    relevant.sort_by_key(|(mask, local)| (local.start, local.end, source_id(&mask.source)));
    Ok(relevant)
}

/// Луч исчезает из конструктивного узла лишь при доказанном полном void у вершины.
pub fn active_rays(
    course_index: i64,
    rays: &[NodeRayInput],
    masks: &[Mask],
) -> Result<BTreeSet<u16>, NodeGeometryError> {
    let mut active = BTreeSet::new();
    for ray in rays {
        let relevant = checked_masks(course_index, ray, masks)?;
        if relevant
            .iter()
            .any(|(mask, local)| local.start == 0 && mask.coverage == MaskCoverage::FullSectionVoid)
        {
            continue;
        }
        if let Some((mask, _)) = relevant.iter().find(|(mask, local)| {
            local.start == 0
                && mask.coverage == MaskCoverage::PartialDepth
                && !matches!(mask.source, MaskSource::Beam(_))
        }) {
            return Err(NodeGeometryError::UnsupportedNodeCut {
                run_id: ray.run_id.clone(),
                source_id: source_id(&mask.source),
                interval: mask.interval,
                reason: "Неполная глубина маски у вершины узла",
            });
        }
        active.insert(ray.direction_deg);
    }
    Ok(active)
}

/// Сохраняет каталожный номинал. Доказанный полный вырез в плече оставляет
/// только связный с вершиной solid; карман балки сохраняет номинал для 3D-выреза.
pub fn resolve_node(
    course_index: i64,
    arms: &[NodeArmInput],
    masks: &[Mask],
) -> Result<NodeGeometry, NodeGeometryError> {
    let mut resolved = Vec::with_capacity(arms.len());
    let mut allowed = Vec::new();
    let mut sources = BTreeSet::new();
    let mut cuts = BTreeSet::new();
    let mut runs = BTreeMap::<&str, i64>::new();
    for arm in arms {
        let ray = &arm.ray;
        let relevant = checked_masks(course_index, ray, masks)?;
        if arm.nominal_length_centimm <= 0 || arm.nominal_length_centimm > ray.run_length_centimm {
            return Err(NodeGeometryError::UnsupportedNodeCut {
                run_id: ray.run_id.clone(),
                source_id: String::new(),
                interval: Interval {
                    start: 0,
                    end: arm.nominal_length_centimm,
                },
                reason: "Каталожное плечо выходит за run",
            });
        }
        if let Some(previous) = runs.insert(&ray.run_id, ray.run_length_centimm) {
            if previous != ray.run_length_centimm {
                return Err(NodeGeometryError::UnsupportedNodeCut {
                    run_id: ray.run_id.clone(),
                    source_id: String::new(),
                    interval: Interval {
                        start: 0,
                        end: ray.run_length_centimm,
                    },
                    reason: "Противоречивые длины одного run",
                });
            }
        }
        let first = relevant.iter().find(|(mask, local)| {
            mask.coverage == MaskCoverage::FullSectionVoid
                && local.start < arm.nominal_length_centimm
        });
        let (effective, cut_sources) = if let Some((_, local)) = first {
            let ids: Vec<String> = relevant
                .iter()
                .filter(|(mask, candidate)| {
                    mask.coverage == MaskCoverage::FullSectionVoid && candidate.start == local.start
                })
                .map(|(mask, _)| source_id(&mask.source))
                .collect();
            (local.start, ids)
        } else {
            (arm.nominal_length_centimm, Vec::new())
        };
        if let Some((mask, _)) = relevant.iter().find(|(mask, local)| {
            mask.coverage == MaskCoverage::PartialDepth
                && local.start < effective
                && !matches!(mask.source, MaskSource::Beam(_))
        }) {
            return Err(NodeGeometryError::UnsupportedNodeCut {
                run_id: ray.run_id.clone(),
                source_id: source_id(&mask.source),
                interval: mask.interval,
                reason: "Неполная глубина маски в плече узла",
            });
        }
        for (mask, local) in &relevant {
            if mask.coverage == MaskCoverage::PartialDepth
                && matches!(mask.source, MaskSource::Beam(_))
                && local.start < effective
            {
                sources.insert(source_id(&mask.source));
            }
        }
        let active_interval = if effective == 0 {
            None
        } else if ray.at_start {
            Some(Interval {
                start: 0,
                end: effective,
            })
        } else {
            Some(Interval {
                start: ray.run_length_centimm - effective,
                end: ray.run_length_centimm,
            })
        };
        let cut_at_distal_end = effective > 0 && effective < arm.nominal_length_centimm;
        if cut_at_distal_end {
            cuts.insert(format!("arm:{}:distal", ray.run_id));
        }
        for id in &cut_sources {
            sources.insert(id.clone());
        }
        resolved.push(ResolvedArm {
            run_id: ray.run_id.clone(),
            direction_deg: ray.direction_deg,
            at_start: ray.at_start,
            active_interval,
            nominal_length_centimm: arm.nominal_length_centimm,
            cut_at_distal_end,
            cut_source_ids: cut_sources,
        });
    }
    for (run_id, length) in runs {
        let mut solid = vec![Interval {
            start: 0,
            end: length,
        }];
        for mask in masks.iter().filter(|mask| {
            mask.course_index == course_index
                && mask.run_id == run_id
                && !(mask.coverage == MaskCoverage::PartialDepth
                    && matches!(mask.source, MaskSource::Beam(_)))
        }) {
            let mut next = Vec::new();
            for span in solid {
                if mask.interval.start > span.start {
                    next.push(Interval {
                        start: span.start,
                        end: mask.interval.start.min(span.end),
                    });
                }
                if mask.interval.end < span.end {
                    next.push(Interval {
                        start: mask.interval.end.max(span.start),
                        end: span.end,
                    });
                }
            }
            solid = next.into_iter().filter(|s| s.start < s.end).collect();
        }
        allowed.extend(solid.into_iter().map(|interval| AllowedSegment {
            run_id: run_id.to_string(),
            interval,
        }));
    }
    Ok(NodeGeometry {
        arms: resolved,
        allowed_segments: allowed,
        source_ids: sources.into_iter().collect(),
        cuts: cuts.into_iter().collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ray(id: &str, direction: u16, at_start: bool) -> NodeRayInput {
        NodeRayInput {
            run_id: id.into(),
            direction_deg: direction,
            at_start,
            run_length_centimm: 1_000,
        }
    }

    fn arm(id: &str, direction: u16, at_start: bool) -> NodeArmInput {
        NodeArmInput {
            ray: ray(id, direction, at_start),
            nominal_length_centimm: 400,
        }
    }

    fn mask(id: &str, start: i64, end: i64, source: MaskSource) -> Mask {
        Mask {
            run_id: id.into(),
            course_index: 2,
            interval: Interval { start, end },
            source,
            coverage: MaskCoverage::FullSectionVoid,
        }
    }

    #[test]
    fn half_open_mask_touching_node_removes_only_covered_ray() {
        let rays = [ray("east", 0, true), ray("north", 90, true)];
        let masks = [mask("east", 0, 100, MaskSource::Opening("door".into()))];
        assert_eq!(active_rays(2, &rays, &masks).unwrap(), BTreeSet::from([90]));
        let result =
            resolve_node(2, &[arm("east", 0, true), arm("north", 90, true)], &masks).unwrap();
        assert_eq!(result.arms[0].active_interval, None);
        assert_eq!(
            result.arms[1].active_interval,
            Some(Interval { start: 0, end: 400 })
        );
        assert!(result.allowed_segments.contains(&AllowedSegment {
            run_id: "east".into(),
            interval: Interval {
                start: 100,
                end: 1_000
            }
        }));
        let boundary = [mask("east", 100, 200, MaskSource::Opening("door".into()))];
        assert_eq!(
            active_rays(2, &rays, &boundary).unwrap(),
            BTreeSet::from([0, 90])
        );
    }

    #[test]
    fn beam_across_vertex_removes_both_masked_rays() {
        let rays = [
            ray("west", 180, false),
            ray("east", 0, true),
            ray("north", 90, true),
        ];
        let masks = [
            mask("west", 900, 1_000, MaskSource::Beam("b".into())),
            mask("east", 0, 100, MaskSource::Beam("b".into())),
        ];
        assert_eq!(active_rays(2, &rays, &masks).unwrap(), BTreeSet::from([90]));
        let result = resolve_node(
            2,
            &[
                arm("west", 180, false),
                arm("east", 0, true),
                arm("north", 90, true),
            ],
            &masks,
        )
        .unwrap();
        assert_eq!(result.arms[0].active_interval, None);
        assert_eq!(result.arms[1].active_interval, None);
        assert_eq!(
            result.arms[2].active_interval,
            Some(Interval { start: 0, end: 400 })
        );
        assert_eq!(result.source_ids, vec!["beam:b"]);
    }

    #[test]
    fn partial_carve_retains_catalog_nominal_and_frees_distant_solid() {
        let masks = [mask("east", 200, 300, MaskSource::Beam("b".into()))];
        let result = resolve_node(2, &[arm("east", 0, true)], &masks).unwrap();
        assert_eq!(result.arms[0].nominal_length_centimm, 400);
        assert_eq!(
            result.arms[0].active_interval,
            Some(Interval { start: 0, end: 200 })
        );
        assert!(result.arms[0].cut_at_distal_end);
        assert_eq!(result.arms[0].cut_source_ids, vec!["beam:b"]);
        assert_eq!(
            result.allowed_segments,
            vec![
                AllowedSegment {
                    run_id: "east".into(),
                    interval: Interval { start: 0, end: 200 }
                },
                AllowedSegment {
                    run_id: "east".into(),
                    interval: Interval {
                        start: 300,
                        end: 1_000
                    }
                },
            ]
        );
    }

    #[test]
    fn opening_near_t_cuts_one_arm_and_preserves_other_two() {
        let masks = [mask("branch", 350, 500, MaskSource::Opening("o".into()))];
        let result = resolve_node(
            2,
            &[
                arm("branch", 90, true),
                arm("left", 180, false),
                arm("right", 0, true),
            ],
            &masks,
        )
        .unwrap();
        assert_eq!(
            result
                .arms
                .iter()
                .map(|a| a.active_interval)
                .collect::<Vec<_>>(),
            vec![
                Some(Interval { start: 0, end: 350 }),
                Some(Interval {
                    start: 600,
                    end: 1_000
                }),
                Some(Interval { start: 0, end: 400 }),
            ]
        );
        assert_eq!(
            result.arms.iter().filter(|a| a.cut_at_distal_end).count(),
            1
        );
    }

    #[test]
    fn rotation_and_opposite_run_orientation_keep_physical_cut() {
        let forward = resolve_node(
            2,
            &[arm("r", 0, true)],
            &[mask("r", 200, 300, MaskSource::Beam("b".into()))],
        )
        .unwrap();
        let reverse = resolve_node(
            2,
            &[arm("r", 180, false)],
            &[mask("r", 700, 800, MaskSource::Beam("b".into()))],
        )
        .unwrap();
        assert_eq!(
            forward.arms[0].active_interval,
            Some(Interval { start: 0, end: 200 })
        );
        assert_eq!(
            reverse.arms[0].active_interval,
            Some(Interval {
                start: 800,
                end: 1_000
            })
        );
        assert_eq!(
            forward.arms[0].nominal_length_centimm,
            reverse.arms[0].nominal_length_centimm
        );
        assert_eq!(
            forward.arms[0].cut_at_distal_end,
            reverse.arms[0].cut_at_distal_end
        );
    }

    #[test]
    fn phase_changes_nominal_without_changing_mask_geometry() {
        let masks = [mask("east", 200, 300, MaskSource::Opening("o".into()))];
        let phase_zero = [arm("east", 0, true), arm("north", 90, true)];
        let mut phase_one = phase_zero.clone();
        phase_one[0].nominal_length_centimm = 600;
        phase_one[1].nominal_length_centimm = 300;
        let zero = resolve_node(2, &phase_zero, &masks).unwrap();
        let one = resolve_node(2, &phase_one, &masks).unwrap();
        assert_eq!(zero.arms[0].active_interval, one.arms[0].active_interval);
        assert_eq!(
            zero.arms[0].active_interval,
            Some(Interval { start: 0, end: 200 })
        );
        assert_eq!(one.arms[0].nominal_length_centimm, 600);
        assert_eq!(
            one.arms[1].active_interval,
            Some(Interval { start: 0, end: 300 })
        );
    }

    #[test]
    fn rejects_invalid_mask_and_arm_that_cannot_fit() {
        let invalid = [mask("r", 900, 1_100, MaskSource::Beam("b".into()))];
        assert!(matches!(
            active_rays(2, &[ray("r", 0, true)], &invalid),
            Err(NodeGeometryError::InvalidMask { .. })
        ));
        let oversized = NodeArmInput {
            ray: ray("r", 0, true),
            nominal_length_centimm: 1_100,
        };
        assert!(matches!(
            resolve_node(2, &[oversized], &[]),
            Err(NodeGeometryError::UnsupportedNodeCut { .. })
        ));
    }

    #[test]
    fn partial_depth_beam_keeps_node_material_and_provenance() {
        let mut partial = mask("r", 0, 300, MaskSource::Beam("b".into()));
        partial.coverage = MaskCoverage::PartialDepth;
        assert_eq!(
            active_rays(2, &[ray("r", 0, true)], &[partial.clone()]).unwrap(),
            BTreeSet::from([0])
        );
        let result = resolve_node(2, &[arm("r", 0, true)], &[partial]).unwrap();
        assert_eq!(
            result.arms[0].active_interval,
            Some(Interval { start: 0, end: 400 })
        );
        assert_eq!(
            result.allowed_segments[0].interval,
            Interval {
                start: 0,
                end: 1_000
            }
        );
        assert_eq!(result.source_ids, vec!["beam:b"]);
    }

    #[test]
    fn partial_opening_remains_diagnostic() {
        let mut partial = mask("r", 200, 300, MaskSource::Opening("o".into()));
        partial.coverage = MaskCoverage::PartialDepth;
        assert!(matches!(
            resolve_node(2, &[arm("r", 0, true)], &[partial]),
            Err(NodeGeometryError::UnsupportedNodeCut { .. })
        ));
    }
}

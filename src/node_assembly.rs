//! Pure assembly of a verified node candidate into physical product parts.
//! Lengths and coordinates are in 0.01 mm; no SUP product codes are inferred here.

use crate::constraints::Interval;
use crate::domain::Point;
use crate::node_geometry::ResolvedArm;
use std::collections::{BTreeMap, BTreeSet};

const HALF: i64 = 32_000;
const FULL: i64 = 64_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeKind {
    L,
    T,
    X,
    Y,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProductKey {
    Type1,
    Type2,
    Type3,
    Type4,
    Type5,
    Type5_1,
    Type6,
    Type7_1,
    Type8,
    Type8_1,
    Type10_1,
    Type11,
    Type12,
    Type13,
    Type14,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CatalogStatus {
    KnownPattern,
    UnsupportedExportCatalog,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FixedCut {
    pub product: ProductKey,
    pub x: u8,
    pub y: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RayPiece {
    pub direction_deg: u16,
    pub product: ProductKey,
    pub reserved_length_centimm: i64,
    pub cut: FixedCut,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CoveredArm {
    pub run_id: String,
    pub interval: Interval,
    pub nominal_length_centimm: i64,
    pub source_ids: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PhysicalCut {
    pub cut: FixedCut,
    /// Both contributors survive when two equal Type6 cuts become one cut.
    pub source_run_ids: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PhysicalNodePart {
    pub product: ProductKey,
    pub catalog_status: CatalogStatus,
    pub local_origin: Point,
    pub local_rotation_deg: u16,
    pub world_origin: Point,
    pub world_rotation_deg: u16,
    pub nominal_length_centimm: i64,
    pub covered_arms: Vec<CoveredArm>,
    pub cuts: Vec<PhysicalCut>,
    pub source_ids: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NodeAssemblyError {
    InvalidPhase(u8),
    UnsupportedVariant {
        kind: NodeKind,
        phase: u8,
        variant: NodePatternVariant,
    },
    InvalidRotation(u16),
    MissingRay(u16),
    DuplicateRay(u16),
    UnexpectedRay(u16),
    WrongNominal {
        run_id: String,
        expected: i64,
        actual: i64,
    },
    ModifiedArm {
        run_id: String,
    },
    MissingSources {
        run_id: String,
    },
    CoordinateOverflow,
    InvalidType6Pair,
}

const fn cut(product: ProductKey, y: u8) -> FixedCut {
    FixedCut { product, x: 1, y }
}

const fn ray(direction_deg: u16, product: ProductKey, length: i64, y: u8) -> RayPiece {
    RayPiece {
        direction_deg,
        product,
        reserved_length_centimm: length,
        cut: cut(product, y),
    }
}

const L0: [RayPiece; 2] = [
    ray(0, ProductKey::Type2, HALF, 3),
    ray(90, ProductKey::Type1, FULL, 1),
];
const L1: [RayPiece; 2] = [
    ray(0, ProductKey::Type3, FULL, 3),
    ray(90, ProductKey::Type4, HALF, 1),
];
const Y0: [RayPiece; 2] = [
    ray(0, ProductKey::Type12, HALF, 3),
    ray(135, ProductKey::Type11, FULL, 1),
];
const Y1: [RayPiece; 2] = [
    ray(0, ProductKey::Type13, FULL, 3),
    ray(135, ProductKey::Type14, HALF, 1),
];
const T0: [RayPiece; 3] = [
    ray(0, ProductKey::Type6, HALF, 3),
    ray(90, ProductKey::Type5_1, FULL, 2),
    ray(180, ProductKey::Type6, HALF, 1),
];
// A 320 mm T0 stem is a Type5 physical product, but its fixed node cut
// retains the Type5.1 cut identity used by the Java/TestAddon catalog.
const T0_SHORT: [RayPiece; 3] = [
    ray(0, ProductKey::Type6, HALF, 3),
    RayPiece {
        direction_deg: 90,
        product: ProductKey::Type5,
        reserved_length_centimm: HALF,
        cut: cut(ProductKey::Type5_1, 2),
    },
    ray(180, ProductKey::Type6, HALF, 1),
];
const T1: [RayPiece; 3] = [
    ray(0, ProductKey::Type8_1, FULL, 1),
    ray(90, ProductKey::Type10_1, HALF, 2),
    ray(180, ProductKey::Type8, FULL, 3),
];
const X0: [RayPiece; 4] = [
    ray(0, ProductKey::Type6, HALF, 3),
    ray(90, ProductKey::Type5_1, FULL, 2),
    ray(180, ProductKey::Type6, HALF, 3),
    ray(270, ProductKey::Type5_1, FULL, 2),
];
const X1: [RayPiece; 4] = [
    ray(0, ProductKey::Type5_1, FULL, 2),
    ray(90, ProductKey::Type6, HALF, 3),
    ray(180, ProductKey::Type5_1, FULL, 2),
    ray(270, ProductKey::Type6, HALF, 3),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeAssemblyPattern {
    pub rays: &'static [RayPiece],
    pub type6_pair: Option<(usize, usize)>,
    pub catalog_status: CatalogStatus,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodePatternVariant {
    Standard,
    T0Short,
}

/// The ray table comes from Java ConnectionClass; a Type6 pair is one physical
/// part only when both opposite 320 mm reservations are present in full.
pub fn pattern(kind: NodeKind, phase: u8) -> Result<NodeAssemblyPattern, NodeAssemblyError> {
    pattern_variant(kind, phase, NodePatternVariant::Standard)
}

/// The finite short-stem option is available only for the T0 central ray.
/// It must be selected before resolving a candidate; trimming a 640 mm stem
/// does not create this product.
pub fn pattern_variant(
    kind: NodeKind,
    phase: u8,
    variant: NodePatternVariant,
) -> Result<NodeAssemblyPattern, NodeAssemblyError> {
    if phase > 1 {
        return Err(NodeAssemblyError::InvalidPhase(phase));
    }
    let (rays, type6_pair): (&'static [RayPiece], _) = match (kind, phase, variant) {
        (NodeKind::L, 0, NodePatternVariant::Standard) => (&L0, None),
        (NodeKind::L, 1, NodePatternVariant::Standard) => (&L1, None),
        (NodeKind::Y, 0, NodePatternVariant::Standard) => (&Y0, None),
        (NodeKind::Y, 1, NodePatternVariant::Standard) => (&Y1, None),
        (NodeKind::T, 0, NodePatternVariant::Standard) => (&T0, Some((0, 2))),
        (NodeKind::T, 0, NodePatternVariant::T0Short) => (&T0_SHORT, Some((0, 2))),
        (NodeKind::T, 1, NodePatternVariant::Standard) => (&T1, None),
        (NodeKind::X, 0, NodePatternVariant::Standard) => (&X0, Some((0, 2))),
        (NodeKind::X, 1, NodePatternVariant::Standard) => (&X1, Some((1, 3))),
        _ => {
            return Err(NodeAssemblyError::UnsupportedVariant {
                kind,
                phase,
                variant,
            })
        }
    };
    Ok(NodeAssemblyPattern {
        rays,
        type6_pair,
        catalog_status: if kind == NodeKind::Y {
            CatalogStatus::UnsupportedExportCatalog
        } else {
            CatalogStatus::KnownPattern
        },
    })
}

fn rotate_local(
    local: Point,
    rotation_deg: u16,
    vertex: Point,
) -> Result<Point, NodeAssemblyError> {
    let radians = f64::from(rotation_deg).to_radians();
    let x = local.x as f64 * radians.cos() - local.y as f64 * radians.sin();
    let y = local.x as f64 * radians.sin() + local.y as f64 * radians.cos();
    if !x.is_finite()
        || !y.is_finite()
        || x.abs() > i64::MAX as f64 / 2.0
        || y.abs() > i64::MAX as f64 / 2.0
    {
        return Err(NodeAssemblyError::CoordinateOverflow);
    }
    let x = vertex
        .x
        .checked_add(x.round() as i64)
        .ok_or(NodeAssemblyError::CoordinateOverflow)?;
    let y = vertex
        .y
        .checked_add(y.round() as i64)
        .ok_or(NodeAssemblyError::CoordinateOverflow)?;
    Ok(Point { x, y })
}

fn source_union(arms: &[CoveredArm]) -> Vec<String> {
    arms.iter()
        .flat_map(|arm| arm.source_ids.iter().cloned())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

fn join_type6_pair(
    first: PhysicalNodePart,
    second: PhysicalNodePart,
    vertex: Point,
    rotation_deg: u16,
) -> Result<PhysicalNodePart, NodeAssemblyError> {
    if first.product != ProductKey::Type6
        || second.product != ProductKey::Type6
        || first.nominal_length_centimm != HALF
        || second.nominal_length_centimm != HALF
        || (first.local_rotation_deg + 180) % 360 != second.local_rotation_deg
        || first.cuts.len() != 1
        || second.cuts.len() != 1
    {
        return Err(NodeAssemblyError::InvalidType6Pair);
    }
    let first_direction = first.local_rotation_deg;
    let local_origin = match first_direction {
        0 => Point { x: HALF, y: 0 },
        90 => Point { x: 0, y: HALF },
        180 => Point { x: -HALF, y: 0 },
        270 => Point { x: 0, y: -HALF },
        _ => return Err(NodeAssemblyError::InvalidType6Pair),
    };
    let first_cut = &first.cuts[0];
    let second_cut = &second.cuts[0];
    let mirrored_y = 4 - first_cut.cut.y;
    let mut cuts = vec![PhysicalCut {
        cut: FixedCut {
            product: ProductKey::Type6,
            x: 2,
            y: mirrored_y,
        },
        source_run_ids: first_cut.source_run_ids.clone(),
    }];
    if second_cut.cut.y != mirrored_y {
        cuts.push(PhysicalCut {
            cut: FixedCut {
                product: ProductKey::Type6,
                x: 2,
                y: second_cut.cut.y,
            },
            source_run_ids: second_cut.source_run_ids.clone(),
        });
    } else {
        cuts[0]
            .source_run_ids
            .extend(second_cut.source_run_ids.iter().cloned());
    }
    let product = match cuts.len() {
        1 if cuts[0].cut.y == 1 => ProductKey::Type6,
        2 if cuts.iter().map(|cut| cut.cut.y).collect::<BTreeSet<_>>()
            == BTreeSet::from([1, 3]) =>
        {
            ProductKey::Type7_1
        }
        _ => return Err(NodeAssemblyError::InvalidType6Pair),
    };
    let mut covered_arms = first.covered_arms;
    covered_arms.extend(second.covered_arms);
    let source_ids = source_union(&covered_arms);
    let local_rotation_deg = (first_direction + 180) % 360;
    Ok(PhysicalNodePart {
        product,
        catalog_status: CatalogStatus::KnownPattern,
        local_origin,
        local_rotation_deg,
        world_origin: rotate_local(local_origin, rotation_deg, vertex)?,
        world_rotation_deg: (rotation_deg + local_rotation_deg) % 360,
        nominal_length_centimm: FULL,
        covered_arms,
        cuts,
        source_ids,
    })
}

/// Assembles one analytical vertex. `source_map` maps run IDs to original wall
/// GUIDs; all interval ownership and cut-source IDs survive the transformation.
pub fn assemble_node(
    kind: NodeKind,
    phase: u8,
    rotation_deg: u16,
    vertex: Point,
    arms: &[ResolvedArm],
    source_map: &BTreeMap<String, Vec<String>>,
) -> Result<Vec<PhysicalNodePart>, NodeAssemblyError> {
    if rotation_deg >= 360 {
        return Err(NodeAssemblyError::InvalidRotation(rotation_deg));
    }
    // Select by the resolved nominal, never by a downstream geometric cut.
    let short_direction = (rotation_deg + 90) % 360;
    let variant = if kind == NodeKind::T
        && phase == 0
        && arms
            .iter()
            .any(|arm| arm.direction_deg == short_direction && arm.nominal_length_centimm == HALF)
    {
        NodePatternVariant::T0Short
    } else {
        NodePatternVariant::Standard
    };
    let definition = pattern_variant(kind, phase, variant)?;
    let expected: BTreeSet<u16> = definition
        .rays
        .iter()
        .map(|ray| (ray.direction_deg + rotation_deg) % 360)
        .collect();
    let mut input = BTreeMap::new();
    for arm in arms {
        if !expected.contains(&arm.direction_deg) {
            return Err(NodeAssemblyError::UnexpectedRay(arm.direction_deg));
        }
        if input.insert(arm.direction_deg, arm).is_some() {
            return Err(NodeAssemblyError::DuplicateRay(arm.direction_deg));
        }
    }
    let mut parts = Vec::with_capacity(definition.rays.len());
    for ray in definition.rays {
        let world_direction = (ray.direction_deg + rotation_deg) % 360;
        let arm = input
            .get(&world_direction)
            .ok_or(NodeAssemblyError::MissingRay(world_direction))?;
        if arm.nominal_length_centimm != ray.reserved_length_centimm {
            return Err(NodeAssemblyError::WrongNominal {
                run_id: arm.run_id.clone(),
                expected: ray.reserved_length_centimm,
                actual: arm.nominal_length_centimm,
            });
        }
        let Some(interval) = arm.active_interval else {
            return Err(NodeAssemblyError::ModifiedArm {
                run_id: arm.run_id.clone(),
            });
        };
        let actual = interval.end.checked_sub(interval.start);
        if actual.is_none_or(|v| v <= 0 || v > ray.reserved_length_centimm)
            || (actual != Some(ray.reserved_length_centimm) && !arm.cut_at_distal_end)
        {
            return Err(NodeAssemblyError::ModifiedArm {
                run_id: arm.run_id.clone(),
            });
        }
        let source_ids = source_map
            .get(&arm.run_id)
            .filter(|ids| !ids.is_empty())
            .ok_or_else(|| NodeAssemblyError::MissingSources {
                run_id: arm.run_id.clone(),
            })?;
        let mut source_ids: BTreeSet<String> = source_ids.iter().cloned().collect();
        source_ids.extend(arm.cut_source_ids.iter().cloned());
        let source_ids: Vec<_> = source_ids.into_iter().collect();
        let covered_arms = vec![CoveredArm {
            run_id: arm.run_id.clone(),
            interval,
            nominal_length_centimm: arm.nominal_length_centimm,
            source_ids: source_ids.clone(),
        }];
        let is_short_stem = variant == NodePatternVariant::T0Short && ray.direction_deg == 90;
        // Java's 320 mm Type5 part runs from the free stem end towards the
        // vertex. Mirroring the physical axis moves its fixed Type5.1 cut to
        // the node end; the analytical covered arm still points outwards.
        let local_origin = if is_short_stem {
            Point { x: 0, y: HALF }
        } else {
            Point { x: 0, y: 0 }
        };
        let local_rotation_deg = if is_short_stem {
            (ray.direction_deg + 180) % 360
        } else {
            ray.direction_deg
        };
        let mut physical_cut = ray.cut;
        if is_short_stem {
            physical_cut.x = 2;
        }
        parts.push(PhysicalNodePart {
            product: ray.product,
            catalog_status: definition.catalog_status,
            local_origin,
            local_rotation_deg,
            world_origin: rotate_local(local_origin, rotation_deg, vertex)?,
            world_rotation_deg: (rotation_deg + local_rotation_deg) % 360,
            nominal_length_centimm: ray.reserved_length_centimm,
            covered_arms,
            cuts: vec![PhysicalCut {
                cut: physical_cut,
                source_run_ids: vec![arm.run_id.clone()],
            }],
            source_ids,
        });
    }
    if let Some((first, second)) = definition.type6_pair {
        let joined = join_type6_pair(
            parts[first].clone(),
            parts[second].clone(),
            vertex,
            rotation_deg,
        )?;
        parts = parts
            .into_iter()
            .enumerate()
            .filter_map(|(index, part)| {
                if index == first {
                    Some(joined.clone())
                } else if index == second {
                    None
                } else {
                    Some(part)
                }
            })
            .collect();
    }
    Ok(parts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(
        kind: NodeKind,
        phase: u8,
        rotation: u16,
    ) -> (Vec<ResolvedArm>, BTreeMap<String, Vec<String>>) {
        let mut arms = Vec::new();
        let mut sources = BTreeMap::new();
        for (index, ray) in pattern(kind, phase).unwrap().rays.iter().enumerate() {
            let run_id = format!("run{index}");
            sources.insert(run_id.clone(), vec![format!("wall{index}")]);
            arms.push(ResolvedArm {
                run_id,
                direction_deg: (ray.direction_deg + rotation) % 360,
                at_start: true,
                active_interval: Some(Interval {
                    start: 0,
                    end: ray.reserved_length_centimm,
                }),
                nominal_length_centimm: ray.reserved_length_centimm,
                cut_at_distal_end: false,
                cut_source_ids: Vec::new(),
            });
        }
        (arms, sources)
    }

    #[test]
    fn all_patterns_have_exact_java_ray_products_and_cuts() {
        let cases = [
            (
                NodeKind::L,
                0,
                vec![ProductKey::Type2, ProductKey::Type1],
                vec![3, 1],
            ),
            (
                NodeKind::L,
                1,
                vec![ProductKey::Type3, ProductKey::Type4],
                vec![3, 1],
            ),
            (
                NodeKind::Y,
                0,
                vec![ProductKey::Type12, ProductKey::Type11],
                vec![3, 1],
            ),
            (
                NodeKind::Y,
                1,
                vec![ProductKey::Type13, ProductKey::Type14],
                vec![3, 1],
            ),
            (
                NodeKind::T,
                0,
                vec![ProductKey::Type6, ProductKey::Type5_1, ProductKey::Type6],
                vec![3, 2, 1],
            ),
            (
                NodeKind::T,
                1,
                vec![ProductKey::Type8_1, ProductKey::Type10_1, ProductKey::Type8],
                vec![1, 2, 3],
            ),
            (
                NodeKind::X,
                0,
                vec![
                    ProductKey::Type6,
                    ProductKey::Type5_1,
                    ProductKey::Type6,
                    ProductKey::Type5_1,
                ],
                vec![3, 2, 3, 2],
            ),
            (
                NodeKind::X,
                1,
                vec![
                    ProductKey::Type5_1,
                    ProductKey::Type6,
                    ProductKey::Type5_1,
                    ProductKey::Type6,
                ],
                vec![2, 3, 2, 3],
            ),
        ];
        for (kind, phase, products, faces) in cases {
            let rays = pattern(kind, phase).unwrap().rays;
            assert_eq!(
                rays.iter().map(|ray| ray.product).collect::<Vec<_>>(),
                products
            );
            assert_eq!(rays.iter().map(|ray| ray.cut.y).collect::<Vec<_>>(), faces);
            assert!(rays.iter().all(|ray| ray.cut.x == 1));
        }
    }

    #[test]
    fn type6_opposite_reservations_form_one_physical_part() {
        for (kind, phase, expected_product, expected_count) in [
            (NodeKind::T, 0, ProductKey::Type6, 2),
            (NodeKind::X, 0, ProductKey::Type7_1, 3),
            (NodeKind::X, 1, ProductKey::Type7_1, 3),
        ] {
            let (arms, sources) = input(kind, phase, 0);
            let parts =
                assemble_node(kind, phase, 0, Point { x: 0, y: 0 }, &arms, &sources).unwrap();
            assert_eq!(parts.len(), expected_count);
            let joined = parts
                .iter()
                .find(|part| part.covered_arms.len() == 2)
                .unwrap();
            assert_eq!(joined.product, expected_product);
            assert_eq!(joined.nominal_length_centimm, FULL);
            assert!(joined
                .cuts
                .iter()
                .all(|cut| cut.cut.product == ProductKey::Type6));
            assert_eq!(
                joined
                    .cuts
                    .iter()
                    .map(|cut| (cut.cut.x, cut.cut.y))
                    .collect::<Vec<_>>(),
                if kind == NodeKind::T {
                    vec![(2, 1)]
                } else {
                    vec![(2, 1), (2, 3)]
                }
            );
            assert_eq!(joined.source_ids.len(), 2);
            assert_eq!(
                joined
                    .cuts
                    .iter()
                    .flat_map(|cut| cut.source_run_ids.iter())
                    .collect::<BTreeSet<_>>()
                    .len(),
                2
            );
        }
    }

    #[test]
    fn rotated_node_preserves_origin_interval_and_source_guid() {
        let (mut arms, sources) = input(NodeKind::L, 1, 270);
        let vertex = Point {
            x: 123_000,
            y: -456_000,
        };
        let parts = assemble_node(NodeKind::L, 1, 270, vertex, &arms, &sources).unwrap();
        assert_eq!(parts.len(), 2);
        assert_eq!(
            parts
                .iter()
                .map(|p| p.world_rotation_deg)
                .collect::<Vec<_>>(),
            vec![270, 0]
        );
        assert!(parts.iter().all(|p| p.world_origin == vertex));
        assert_eq!(
            parts[0].covered_arms[0].interval,
            Interval {
                start: 0,
                end: FULL
            }
        );
        assert_eq!(parts[0].source_ids, vec!["wall0"]);
        arms.reverse();
        assert_eq!(
            assemble_node(NodeKind::L, 1, 270, vertex, &arms, &sources).unwrap(),
            parts
        );
    }

    #[test]
    fn coalesced_part_origin_and_rotation_follow_first_reserved_ray() {
        let (arms, sources) = input(NodeKind::T, 0, 90);
        let vertex = Point {
            x: 10_000,
            y: 20_000,
        };
        let parts = assemble_node(NodeKind::T, 0, 90, vertex, &arms, &sources).unwrap();
        let joined = parts
            .iter()
            .find(|part| part.covered_arms.len() == 2)
            .unwrap();
        assert_eq!(joined.local_origin, Point { x: HALF, y: 0 });
        assert_eq!(joined.local_rotation_deg, 180);
        assert_eq!(
            joined.world_origin,
            Point {
                x: 10_000,
                y: 20_000 + HALF
            }
        );
        assert_eq!(joined.world_rotation_deg, 270);
        assert_eq!(joined.cuts[0].source_run_ids, vec!["run0", "run2"]);
    }

    #[test]
    fn y_is_assembled_but_export_catalog_is_typed_unsupported() {
        let (arms, sources) = input(NodeKind::Y, 0, 45);
        let parts =
            assemble_node(NodeKind::Y, 0, 45, Point { x: 0, y: 0 }, &arms, &sources).unwrap();
        assert_eq!(parts.len(), 2);
        assert!(parts
            .iter()
            .all(|p| p.catalog_status == CatalogStatus::UnsupportedExportCatalog));
    }

    #[test]
    fn obstacle_crop_preserves_type6_stock_and_fixed_join() {
        let (mut arms, sources) = input(NodeKind::T, 0, 0);
        arms[0].active_interval = Some(Interval {
            start: 0,
            end: HALF - 1,
        });
        arms[0].cut_at_distal_end = true;
        arms[0].cut_source_ids.push("beam:abc".into());
        let parts =
            assemble_node(NodeKind::T, 0, 0, Point { x: 0, y: 0 }, &arms, &sources).unwrap();
        let part = parts
            .iter()
            .find(|p| p.product == ProductKey::Type6)
            .unwrap();
        assert_eq!(part.nominal_length_centimm, FULL);
        assert_eq!(
            part.covered_arms
                .iter()
                .map(|a| a.interval.end - a.interval.start)
                .sum::<i64>(),
            FULL - 1
        );
        assert!(part.source_ids.contains(&"beam:abc".into()));
        arms[0].cut_at_distal_end = false;
        assert!(matches!(
            assemble_node(NodeKind::T, 0, 0, Point { x: 0, y: 0 }, &arms, &sources),
            Err(NodeAssemblyError::ModifiedArm { .. })
        ));
    }

    #[test]
    fn t0_short_stem_is_type5_with_type5_1_node_cut_and_original_sources() {
        let (mut arms, mut sources) = input(NodeKind::T, 0, 0);
        arms[1].nominal_length_centimm = HALF;
        arms[1].active_interval = Some(Interval {
            start: 0,
            end: HALF,
        });
        arms[1].cut_source_ids.push("opening:stem".into());
        sources.insert("run1".into(), vec!["wall:short".into()]);
        let vertex = Point {
            x: -480_000,
            y: -128_000,
        };
        let parts = assemble_node(NodeKind::T, 0, 0, vertex, &arms, &sources).unwrap();
        assert_eq!(parts.len(), 2);
        let stem = parts
            .iter()
            .find(|part| part.product == ProductKey::Type5)
            .unwrap();
        assert_eq!(stem.nominal_length_centimm, HALF);
        assert_eq!(stem.local_origin, Point { x: 0, y: HALF });
        assert_eq!(
            stem.world_origin,
            Point {
                x: -480_000,
                y: -96_000
            }
        );
        assert_eq!(stem.local_rotation_deg, 270);
        assert_eq!(stem.world_rotation_deg, 270);
        assert_eq!(
            stem.cuts[0].cut,
            FixedCut {
                product: ProductKey::Type5_1,
                x: 2,
                y: 2
            }
        );
        assert_eq!(stem.cuts[0].source_run_ids, vec!["run1"]);
        assert_eq!(stem.source_ids, vec!["opening:stem", "wall:short"]);
        assert_eq!(
            stem.covered_arms[0].interval,
            Interval {
                start: 0,
                end: HALF
            }
        );
        assert!(parts.iter().any(|part| part.product == ProductKey::Type6));
        assert_eq!(
            pattern(NodeKind::T, 0).unwrap().rays[1].reserved_length_centimm,
            FULL
        );
        assert_eq!(
            pattern_variant(NodeKind::T, 0, NodePatternVariant::T0Short)
                .unwrap()
                .rays[1]
                .reserved_length_centimm,
            HALF
        );
    }

    #[test]
    fn t0_full_stem_remains_type5_1_and_short_cannot_be_inferred_from_trim() {
        let (arms, sources) = input(NodeKind::T, 0, 90);
        let vertex = Point {
            x: 100_000,
            y: 200_000,
        };
        let parts = assemble_node(NodeKind::T, 0, 90, vertex, &arms, &sources).unwrap();
        let stem = parts
            .iter()
            .find(|part| part.product == ProductKey::Type5_1)
            .unwrap();
        assert_eq!(stem.nominal_length_centimm, FULL);
        assert_eq!(stem.local_origin, Point { x: 0, y: 0 });
        assert_eq!(stem.world_origin, vertex);
        assert_eq!(stem.world_rotation_deg, 180);
        assert_eq!(
            stem.cuts[0].cut,
            FixedCut {
                product: ProductKey::Type5_1,
                x: 1,
                y: 2
            }
        );

        let mut trimmed = arms.clone();
        trimmed[1].active_interval = Some(Interval {
            start: 0,
            end: HALF,
        });
        trimmed[1].cut_source_ids.push("beam:cut".into());
        assert_eq!(
            assemble_node(NodeKind::T, 0, 90, vertex, &trimmed, &sources),
            Err(NodeAssemblyError::ModifiedArm {
                run_id: "run1".into()
            })
        );
    }

    #[test]
    fn t0_stem_other_nominal_is_rejected_and_short_variant_is_t0_only() {
        let (mut arms, sources) = input(NodeKind::T, 0, 0);
        arms[1].nominal_length_centimm = 30_000;
        arms[1].active_interval = Some(Interval {
            start: 0,
            end: 30_000,
        });
        assert_eq!(
            assemble_node(NodeKind::T, 0, 0, Point { x: 0, y: 0 }, &arms, &sources),
            Err(NodeAssemblyError::WrongNominal {
                run_id: "run1".into(),
                expected: FULL,
                actual: 30_000,
            })
        );
        assert_eq!(
            pattern_variant(NodeKind::T, 1, NodePatternVariant::T0Short),
            Err(NodeAssemblyError::UnsupportedVariant {
                kind: NodeKind::T,
                phase: 1,
                variant: NodePatternVariant::T0Short,
            })
        );
    }
}

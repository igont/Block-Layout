pub mod api;
pub mod choice;
pub mod code_result;
pub mod constraints;
pub mod domain;
pub mod exchange;
pub mod grid;
pub mod layout;
pub mod materialize;
pub mod node_assembly;
pub mod node_geometry;
pub mod node_shapes;
pub mod offcut_plan;
pub mod product_catalog;
pub mod production_plan;
pub mod solid_geometry;
pub mod topology;
pub mod vertical;

use std::time::Instant;

use api::{ApiFailure, LayoutRequest, LayoutResponse};
use constraints::{BeamCutPolicy, ConstraintErrorKind, RawBeam, RawOpening};
use domain::RawPoint;
use layout::{Block, Profile};

#[derive(Debug, serde::Serialize)]
pub struct CandidateResult {
    pub blocks: Vec<Block>,
    pub diagnostics: Vec<ApiFailure>,
    pub elapsed_ms: u128,
}

/// Полный исследовательский план; замечания не делают его производственным результатом.
fn source_beam_spans_opening(o: &RawOpening, b: &RawBeam, support_mm: f64) -> bool {
    let dx = o.end.x_mm - o.start.x_mm;
    let dy = o.end.y_mm - o.start.y_mm;
    let width = dx.hypot(dy);
    if width <= 0.0 || (o.top_start_mm - o.top_end_mm).abs() > 0.01 {
        return false;
    }
    let (ux, uy) = (dx / width, dy / width);
    let start = (b.start.x_mm - o.start.x_mm) * ux + (b.start.y_mm - o.start.y_mm) * uy;
    let end = (b.end.x_mm - o.start.x_mm) * ux + (b.end.y_mm - o.start.y_mm) * uy;
    (b.height_mm - 315.0).abs() < 0.01
        && (b.bottom_start_mm - o.top_start_mm).abs() < 0.01
        && (b.bottom_end_mm - o.top_end_mm).abs() < 0.01
        && [b.start, b.end]
            .iter()
            .all(|p| ((p.x_mm - o.start.x_mm) * uy - (p.y_mm - o.start.y_mm) * ux).abs() < 0.01)
        && start.min(end) <= -support_mm + 0.01
        && start.max(end) >= width + support_mm - 0.01
}

pub fn inspect_request(request: &LayoutRequest, profile: &Profile) -> CandidateResult {
    let started = Instant::now();
    let failure = |diagnostics: Vec<ApiFailure>| CandidateResult {
        blocks: Vec::new(),
        diagnostics,
        elapsed_ms: started.elapsed().as_millis(),
    };
    if let Err(error) = request.validate() {
        return failure(vec![error]);
    }
    // Единая локальная копия нужна и ограничениям, и окончательной подрезке узлов.
    let mut effective = request.clone();
    for opening in &mut effective.opening_volumes {
        if !opening.is_outside && opening.start_bottom_zmm.abs() < 0.01
            && opening.end_bottom_zmm.abs() < 0.01 {
            opening.start_bottom_zmm = -2.0 * profile.index_centimm as f64 / 100.0;
            opening.end_bottom_zmm = opening.start_bottom_zmm;
        }
    }
    let request = &effective;
    let building = match topology::normalize_building(request.raw_building()) {
        Ok(value) => value,
        Err(message) => {
            return failure(vec![ApiFailure::new("NORMALIZATION_FAILED", message, None)])
        }
    };
    let topology = match topology::build_topology(building) {
        Ok(value) => value,
        Err(message) => return failure(vec![ApiFailure::new("TOPOLOGY_FAILED", message, None)]),
    };
    let openings: Vec<RawOpening> = request
        .opening_volumes
        .iter()
        .filter(|o| {
            matches!(
                o.opening_type.as_str(),
                "OPENING" | "CONSOLE" | "WINDOW" | "DOOR"
            )
        })
        .map(|o| RawOpening {
            id: o.guid.clone(),
            start: RawPoint {
                x_mm: o.start_xmm,
                y_mm: o.start_ymm,
            },
            end: RawPoint {
                x_mm: o.end_xmm,
                y_mm: o.end_ymm,
            },
            bottom_start_mm: o.start_bottom_zmm,
            bottom_end_mm: o.end_bottom_zmm,
            top_start_mm: o.start_top_zmm,
            top_end_mm: o.end_top_zmm,
        })
        .collect();
    let beams: Vec<RawBeam> = request
        .beams
        .iter()
        .map(|b| RawBeam {
            id: b.guid.clone(),
            start: RawPoint {
                x_mm: b.start_xmm,
                y_mm: b.start_ymm,
            },
            end: RawPoint {
                x_mm: b.end_xmm,
                y_mm: b.end_ymm,
            },
            bottom_start_mm: b.start_zmm,
            bottom_end_mm: b.end_zmm,
            top_start_mm: b.start_zmm + b.geometry.height_mm * b.geometry.height_direction_z,
            top_end_mm: b.end_zmm + b.geometry.height_mm * b.geometry.height_direction_z,
            width_mm: b.geometry.width_mm,
            height_mm: b.geometry.height_mm,
            height_direction_x: b.geometry.height_direction_x,
            height_direction_y: b.geometry.height_direction_y,
            height_direction_z: b.geometry.height_direction_z,
        })
        .collect();
    let mut constraints = match constraints::build_constraints_with_policy(
        &topology,
        &openings,
        &beams,
        63.0,
        profile.lintel_support_mm,
        BeamCutPolicy {
            longitudinal_full_wall_thickness: profile.longitudinal_full_wall_thickness,
            assembly_clearance_mm: profile.assembly_clearance_mm,
            full_course_clearance: profile.full_course_clearance,
            affine_world_joint_spike_compensation: profile.world_joint_policy.as_deref()
                == Some("affine_checkerboard_v1"),
        },
    ) {
        Ok(value) => value,
        Err(errors) => {
            return failure(
                errors
                    .into_iter()
                    .map(|e| {
                        ApiFailure::new(
                            match e.kind {
                                ConstraintErrorKind::InvalidGeometry => "INVALID_GEOMETRY",
                                ConstraintErrorKind::UnsupportedBeamGeometry => {
                                    "UNSUPPORTED_BEAM_GEOMETRY"
                                }
                                ConstraintErrorKind::AmbiguousOpeningRun => "AMBIGUOUS_OPENING_RUN",
                            },
                            e.message,
                            Some(e.source_id),
                        )
                    })
                    .collect(),
            )
        }
    };
    for wall in request.wall_volumes.iter().filter(|w| w.opening_type == "CONSOLE") {
        for course in &topology.courses {
            for run in &course.runs {
                let Some(source) = run.sources.iter().find(|s| s.wall_id == wall.guid) else { continue; };
                let (root, tip) = if source.end_offset == run.length && source.start_offset > 0 {
                    (source.start, source.end)
                } else if source.start_offset == 0 && source.end_offset < run.length {
                    (source.end, source.start)
                } else if course.vertices.iter().any(|v| v.point == source.start && v.rays.len() >= 3)
                    && course.vertices.iter().any(|v| v.point == source.end && v.rays.len() == 1) {
                    (source.start, source.end)
                } else if course.vertices.iter().any(|v| v.point == source.end && v.rays.len() >= 3)
                    && course.vertices.iter().any(|v| v.point == source.start && v.rays.len() == 1) {
                    (source.end, source.start)
                } else { continue; };
                let raw = |p: domain::Point| RawPoint { x_mm: p.x as f64 / 100.0, y_mm: p.y as f64 / 100.0 };
                constraints.console_axes.insert(wall.guid.clone(), (raw(root), raw(tip)));
                constraints.lintel_candidates.push(constraints::LintelCandidate {
                    opening_id: wall.guid.clone(), run_id: run.id.clone(), course_index: course.index,
                    span: constraints::Interval { start: source.start_offset, end: source.end_offset },
                    left_support: None, right_support: None,
                });
            }
        }
    }
    let beam_spanned: Vec<_> = openings
        .iter()
        .filter(|o| (o.end.x_mm - o.start.x_mm).hypot(o.end.y_mm - o.start.y_mm) <= 1500.0)
        .filter_map(|o| {
            beams
                .iter()
                .find(|b| source_beam_spans_opening(o, b, profile.lintel_support_mm))
                .map(|b| (o.id.clone(), b.id.clone()))
        })
        .collect();
    constraints.lintel_candidates.retain(|candidate| {
        !beam_spanned
            .iter()
            .any(|(id, _)| *id == candidate.opening_id)
    });
    // Разделяющая маска балки занимает всю толщину и высоту детали.
    // Маски источников проверяются отдельно, до окончательного разделения.
    for index in 0..constraints.masks.len() {
        let mask = constraints.masks[index].clone();
        let Some(course) = topology
            .courses
            .iter()
            .find(|c| c.index == mask.course_index)
        else {
            continue;
        };
        let Some(run) = course.runs.iter().find(|r| r.id == mask.run_id) else {
            continue;
        };
        let Some(wall) = run
            .sources
            .first()
            .and_then(|s| topology.walls.iter().find(|w| w.id == s.wall_id))
        else {
            continue;
        };
        let solid = constraints::SolidBox {
            u: mask.interval,
            v: constraints::Interval {
                start: -wall.thickness / 2,
                end: wall.thickness - wall.thickness / 2,
            },
            z: constraints::Interval {
                start: course.z,
                end: course.z + profile.index_centimm,
            },
        };
        let classified =
            if profile.full_course_clearance || profile.longitudinal_full_wall_thickness {
                constraints.classify_assembly_box(&run.id, course.index, solid)
            } else {
                constraints.classify_box(&run.id, course.index, solid)
            };
        if matches!(classified, Ok(constraints::Exclusion::FullVoid { .. })) {
            constraints.masks[index].coverage = constraints::MaskCoverage::FullSectionVoid;
        }
    }
    // Опирание задано минимумом. Непрерывный короткий остаток у грани балки
    // включается в перемычку, когда он помещается в каталожную заготовку.
    for lintel in &mut constraints.lintel_candidates {
        let (Some(mut left), Some(mut right)) = (lintel.left_support, lintel.right_support) else {
            continue;
        };
        for mask in constraints.masks.iter().filter(|m| {
            m.course_index == lintel.course_index
                && m.run_id == lintel.run_id
                && m.coverage == constraints::MaskCoverage::FullSectionVoid
        }) {
            let left_gap = left.start - mask.interval.end;
            let right_gap = mask.interval.start - right.end;
            let (new_left, new_right) = if left_gap > 0 && left_gap < profile.minimum_cut_centimm {
                (mask.interval.end, right.end)
            } else if right_gap > 0 && right_gap < profile.minimum_cut_centimm {
                (left.start, mask.interval.start)
            } else {
                continue;
            };
            if profile
                .bridge_nominal_lengths_centimm
                .iter()
                .any(|n| *n >= new_right - new_left)
            {
                left.start = new_left;
                right.end = new_right;
            }
        }
        lintel.left_support = Some(left);
        lintel.right_support = Some(right);
    }
    let layout = match layout::calculate_candidate(&topology, &constraints, profile) {
        Ok(value) => value,
        Err(errors) => {
            return failure(
                errors
                    .into_iter()
                    .map(|e| {
                        let mut item = ApiFailure::new(
                            serde_json::to_value(&e.code)
                                .ok()
                                .and_then(|v| v.as_str().map(str::to_owned))
                                .unwrap_or_else(|| "LAYOUT_REJECTED".into()),
                            e.message,
                            e.wall_id.or(e.edge_id),
                        );
                        item.source_ids = e.source_ids;
                        item.course_index = e.course_index;
                        item.occurrences = e.occurrences;
                        item
                    })
                    .collect(),
            )
        }
    };
    let mut diagnostics: Vec<ApiFailure> =
        layout.diagnostics.into_iter().map(layout_failure).collect();
    for wall in request.wall_volumes.iter().filter(|w| w.opening_type == "CONSOLE") {
        if !constraints.console_axes.contains_key(&wall.guid) {
            diagnostics.push(ApiFailure::new("CONSOLE_ROOT_REQUIRED",
                "Не найден корень консоли на непрерывной стене; материал сохранён", Some(wall.guid.clone())));
            continue;
        }
        for part in layout.layout.blocks.iter().filter(|b| b.product_key.as_deref() == Some("Type6")
            && b.source_ids.contains(&format!("wall:{}", wall.guid))) {
            let mut warning = ApiFailure::new("CONSOLE_LINTEL_UNPLACED",
                "Перемычка консоли не помещается в исходном ряду типа 6 с заглублением больше длины консоли; материал сохранён",
                Some(wall.guid.clone()));
            warning.course_index = Some(part.course_index);
            diagnostics.push(warning);
        }
    }
    for (opening_id, beam_id) in &beam_spanned {
        let mut note = ApiFailure::new("OPENING_SPANNED_BY_SOURCE_BEAM",
            "Исходная балка высотой 315 мм перекрывает проём с опиранием с обеих сторон и заменяет обычные перемычки", None);
        note.source_ids = vec![opening_id.clone(), beam_id.clone()];
        diagnostics.push(note);
    }
    for opening in &openings {
        if beam_spanned.iter().any(|(id, _)| *id == opening.id) {
            continue;
        }
        let width =
            (opening.end.x_mm - opening.start.x_mm).hypot(opening.end.y_mm - opening.start.y_mm);
        if width > 1500.0 || (opening.top_start_mm - opening.top_end_mm).abs() > 0.01 {
            continue;
        }
        let required = if width <= 1000.0 { 3 } else { 5 };
        let available = constraints
            .lintel_candidates
            .iter()
            .filter(|l| l.opening_id == opening.id)
            .map(|l| l.course_index)
            .collect::<std::collections::BTreeSet<_>>()
            .len();
        if available < required {
            let top_row = opening.top_start_mm
                + ((required - 1) * 2) as f64 * profile.index_centimm as f64 / 100.0;
            let mut note = ApiFailure::new("LINTEL_ROWS_INCOMPLETE",format!("Для проёма имеется {available} из {required} требуемых рядов перемычек; верхний требуемый ряд {top_row} мм отсутствует в материале стены"),Some(opening.id.clone()));
            note.source_ids.push(opening.id.clone());
            diagnostics.push(note);
        }
    }
    let excluded: Vec<_> = request
        .opening_volumes
        .iter()
        .filter(|o| {
            !matches!(
                o.opening_type.as_str(),
                "OPENING" | "CONSOLE" | "WINDOW" | "DOOR"
            )
        })
        .map(|o| o.guid.clone())
        .collect();
    if !excluded.is_empty() {
        let mut note = ApiFailure::new(
            "OPENING_PURPOSE_EXCLUDED",
            "Проёмы исключённых типов сохранены во входе и не применяются к несущим стенам",
            None,
        );
        note.source_ids = excluded;
        diagnostics.push(note);
    }
    let wide: Vec<_> = openings
        .iter()
        .filter(|o| {
            let dx = o.end.x_mm - o.start.x_mm;
            let dy = o.end.y_mm - o.start.y_mm;
            let width = dx.hypot(dy);
            if width <= 1500.0 {
                return false;
            }
            !beams
                .iter()
                .any(|b| source_beam_spans_opening(o, b, profile.lintel_support_mm))
        })
        .map(|o| o.id.clone())
        .collect();
    if !wide.is_empty() {
        let mut note = ApiFailure::new("OPENING_SUPPORT_BEAM_REQUIRED",
            "Для проёма шире 1500 мм нужна переданная балка высотой 315 мм с опиранием; вычет проёма сохранён", None);
        note.source_ids = wide;
        diagnostics.push(note);
    }
    let special: Vec<String> = layout
        .layout
        .blocks
        .iter()
        .filter(|b| b.kind == "special_ordinary")
        .map(|b| b.id.clone())
        .collect();
    if !special.is_empty() {
        let mut note=ApiFailure::new("SPECIAL_PRODUCTS_REQUIRED",format!("{} специальных рядовых изделий после объединения коротких остатков требуют согласования каталога",special.len()),None);
        note.source_ids = special;
        diagnostics.push(note);
    }
    let short: Vec<String> = layout
        .layout
        .blocks
        .iter()
        .filter(|b| b.kind == "ordinary" && b.length_centimm < profile.minimum_cut_centimm)
        .map(|b| b.id.clone())
        .collect();
    if !short.is_empty() {
        let mut note=ApiFailure::new("SHORT_PART_REQUIRES_REDESIGN",format!("{} коротких участков сохранены по входной геометрии; их нельзя выпускать отдельными изделиями без решения объединения",short.len()),None);
        note.source_ids = short;
        diagnostics.push(note);
    }
    CandidateResult {
        blocks: match layout::finalize_parts(request, profile, &layout.layout.blocks) {
            Ok(blocks) => blocks,
            Err(error) => return failure(vec![error]),
        },
        diagnostics,
        elapsed_ms: started.elapsed().as_millis(),
    }
}

fn layout_failure(e: layout::Diagnostic) -> ApiFailure {
    let mut item = ApiFailure::new(
        serde_json::to_value(&e.code)
            .ok()
            .and_then(|v| v.as_str().map(str::to_owned))
            .unwrap_or_else(|| "LAYOUT_REJECTED".into()),
        e.message,
        e.wall_id.or(e.edge_id),
    );
    item.source_ids = e.source_ids;
    item.course_index = e.course_index;
    item.occurrences = e.occurrences;
    item
}

#[cfg(test)]
mod opening_tests {
    use super::*;
    use serde_json::json;

    fn request(bottom: f64) -> LayoutRequest {
        serde_json::from_value(json!({"schema_version":1,"request_id":"opening-volume","snapshot_hash":"a".repeat(64),"z0_mm":0,
            "wall_volumes":[{"guid":"wall","startXmm":0,"startYmm":0,"endXmm":1280,"endYmm":0,
                "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],
            "opening_volumes":[{"guid":"opening","startXmm":640,"startYmm":0,"endXmm":960,"endYmm":0,
                "startBottomZmm":bottom,"endBottomZmm":bottom,"startTopZmm":63,"endTopZmm":63,"openingType":"WINDOW"}],"beams":[]})).unwrap()
    }

    #[test]
    fn full_volume_leaves_exact_material_and_cuts_spikes_at_both_faces() {
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let result = inspect_request(&request(0.0), &profile);
        assert!(!result.blocks.is_empty(), "{:?}", result.diagnostics);
        assert_eq!(
            result.blocks.iter().map(|b| b.length_centimm).sum::<i64>(),
            96_000
        );
        let left = result.blocks.iter().find(|b| b.end.x == 64_000).unwrap();
        let right = result.blocks.iter().find(|b| b.start.x == 96_000).unwrap();
        assert!(left.hide_spikes_right && right.hide_spikes_left);
        assert!(result
            .blocks
            .iter()
            .all(|b| b.end.x <= 64_000 || b.start.x >= 96_000));
    }

    #[test]
    fn nonaligned_opening_bottom_clears_whole_course_and_keeps_source() {
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let result = inspect_request(&request(20.0), &profile);
        assert!(!result.blocks.is_empty(), "{:?}", result.diagnostics);
        assert_eq!(
            result.blocks.iter().map(|b| b.length_centimm).sum::<i64>(),
            96_000
        );
        assert!(result
            .blocks
            .iter()
            .all(|b| !b.cuts.iter().any(|c| c.starts_with("opening_volume:"))));
        assert!(result
            .blocks
            .iter()
            .any(|b| b.source_ids.contains(&"opening:opening".into())));
    }

    #[test]
    fn longitudinal_160_beam_clears_full_193_wall_thickness_for_five_courses() {
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let mut r = request(0.0);
        r.wall_volumes[0].end_xmm = 3200.0;
        r.wall_volumes[0].start_top_zmm = 378.0;
        r.wall_volumes[0].end_top_zmm = 378.0;
        r.opening_volumes[0].end_xmm = 2240.0;
        let missing = inspect_request(&r, &profile);
        assert!(missing.blocks.iter().all(|b| !b.is_bridge));
        assert!(missing
            .diagnostics
            .iter()
            .any(|d| d.code == "OPENING_SUPPORT_BEAM_REQUIRED"
                && d.source_ids.contains(&"opening".into())));
        r.beams.push(
            serde_json::from_value(
                json!({"guid":"wide-beam","startXmm":320,"startYmm":0,"startZmm":63,
            "endXmm":2560,"endYmm":0,"endZmm":63,"geometry":{"widthMm":160,"heightMm":315,
            "heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}),
            )
            .unwrap(),
        );
        let present = inspect_request(&r, &profile);
        assert!(present.blocks.iter().all(|b| !b.is_bridge));
        assert!(!present
            .diagnostics
            .iter()
            .any(|d| d.code == "OPENING_SUPPORT_BEAM_REQUIRED"));
        for course in 1..=5 {
            let blocks: Vec<_> = present
                .blocks
                .iter()
                .filter(|b| b.course_index == course)
                .collect();
            assert_eq!(blocks.iter().map(|b| b.length_centimm).sum::<i64>(), 96_000);
            assert!(blocks
                .iter()
                .all(|b| b.end.x <= 32_000 || b.start.x >= 256_000));
            assert!(blocks
                .iter()
                .all(|b| !b.cuts.iter().any(|c| c.starts_with("beam_volume:"))));
            assert!(blocks
                .iter()
                .filter(|b| b.end.x == 32_000)
                .all(|b| b.hide_spikes_right));
            assert!(blocks
                .iter()
                .filter(|b| b.start.x == 256_000)
                .all(|b| b.hide_spikes_left));
        }
        let mut narrow = r.clone();
        narrow.opening_volumes[0].end_xmm = 1600.0;
        let covered = inspect_request(&narrow, &profile);
        assert!(covered.blocks.iter().all(|b| !b.is_bridge));
        assert!(covered
            .diagnostics
            .iter()
            .any(|d| d.code == "OPENING_SPANNED_BY_SOURCE_BEAM"
                && d.source_ids == vec!["opening", "wide-beam"]));
        assert!(covered
            .diagnostics
            .iter()
            .all(|d| d.code != "INVALID_SUPPORT" && d.code != "LINTEL_ROWS_INCOMPLETE"));
        narrow.beams[0].start_xmm = 700.0;
        let partial = inspect_request(&narrow, &profile);
        assert!(partial
            .diagnostics
            .iter()
            .all(|d| d.code != "OPENING_SPANNED_BY_SOURCE_BEAM"));
        let mut stem = r.wall_volumes[0].clone();
        stem.guid = "stem".into();
        stem.start_xmm = 1600.0;
        stem.end_xmm = 1600.0;
        stem.end_ymm = 1280.0;
        r.wall_volumes.push(stem);
        let full_node = inspect_request(&r, &profile);
        for part in full_node
            .blocks
            .iter()
            .filter(|b| (1..=5).contains(&b.course_index))
        {
            for arm in &part.arms {
                if arm.start.y == 0 && arm.end.y == 0 {
                    assert!(
                        arm.start.x.max(arm.end.x) <= 32_000
                            || arm.start.x.min(arm.end.x) >= 256_000,
                        "Продольная часть T осталась внутри вычета балки: {:?}",
                        part
                    );
                }
            }
        }
        let mut clipped_node_request = r.clone();
        clipped_node_request.beams[0].start_xmm = 1620.0;
        let clipped_node = inspect_request(&clipped_node_request, &profile);
        assert!(
            clipped_node
                .blocks
                .iter()
                .all(|b| b.product_key.as_deref() != Some("Type6")
                    || b.course_index == 0),
            "Поперечный проход должен удалить остаток Type6 на толщине стены"
        );
        // Даже частичное пересечение по высоте окончательно разделяет детали.
        r.beams[0].start_zmm = 80.0;
        r.beams[0].end_zmm = 80.0;
        let node = inspect_request(&r, &profile);
        assert!(
            node.blocks
                .iter()
                .all(|b| !b.cuts.iter().any(|cut| cut.starts_with("beam_volume:"))),
            "{:?} nodes={:?}",
            node.diagnostics,
            node.blocks
                .iter()
                .filter(|b| !b.arms.is_empty())
                .map(|b| (&b.kind, &b.source_ids, &b.cuts))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn actual_t_end_may_touch_type10_without_offset() {
        let mut r = request(0.0);
        r.wall_volumes[0].end_xmm = 3200.0;
        r.wall_volumes[0].start_top_zmm = 504.0;
        r.wall_volumes[0].end_top_zmm = 504.0;
        let mut stem = r.wall_volumes[0].clone();
        stem.guid = "stem".into();
        stem.start_xmm = 1600.0;
        stem.end_xmm = 1600.0;
        stem.end_ymm = 1280.0;
        r.wall_volumes.push(stem);
        r.opening_volumes[0].start_xmm = 600.0;
        r.opening_volumes[0].end_xmm = 1400.0;
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let mut touched = false;
        for rotation in [0, 90, 180, 270] {
            for side in [-1.0, 1.0] {
                let mut oriented = r.clone();
                oriented.wall_volumes[1].end_ymm = side * 1280.0;
                let rotate = |x: f64, y: f64| match rotation {
                    0 => (x, y),
                    90 => (-y, x),
                    180 => (-x, -y),
                    _ => (y, -x),
                };
                for volume in oriented
                    .wall_volumes
                    .iter_mut()
                    .chain(oriented.opening_volumes.iter_mut())
                {
                    (volume.start_xmm, volume.start_ymm) =
                        rotate(volume.start_xmm, volume.start_ymm);
                    (volume.end_xmm, volume.end_ymm) = rotate(volume.end_xmm, volume.end_ymm);
                }
                let result = inspect_request(&oriented, &profile);
                assert!(!result.blocks.is_empty(), "{:?}", result.diagnostics);
                for bridge in result.blocks.iter().filter(|b| b.is_bridge) {
                    touched |= result.blocks.iter().any(|b| {
                        b.course_index == bridge.course_index
                            && b.product_key.as_deref() == Some("Type10_1")
                            && b.start == bridge.end
                    });
                }
            }
        }
        assert!(
            touched,
            "Type10.1 на точном конце должен сохранить исходный венец перемычки"
        );
    }
    #[test]
    fn actual_t_catalog_cut_survives_in_long_bridge_and_stem_is_kept() {
        let mut r = request(0.0);
        r.wall_volumes[0].end_xmm = 3200.0;
        r.wall_volumes[0].start_top_zmm = 504.0;
        r.wall_volumes[0].end_top_zmm = 504.0;
        let mut stem = r.wall_volumes[0].clone();
        stem.guid = "stem".into();
        stem.start_xmm = 1600.0;
        stem.end_xmm = 1600.0;
        stem.end_ymm = 1280.0;
        r.wall_volumes.push(stem);
        r.opening_volumes[0].start_xmm = 1200.0;
        r.opening_volumes[0].end_xmm = 2000.0;
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        for rotation in [0, 90, 180, 270] {
            for stem_side in [-1.0, 1.0] {
                let mut oriented = r.clone();
                oriented.wall_volumes[1].end_ymm = stem_side * 1280.0;
                let rotate = |x: f64, y: f64| match rotation {
                    0 => (x, y),
                    90 => (-y, x),
                    180 => (-x, -y),
                    _ => (y, -x),
                };
                for volume in oriented
                    .wall_volumes
                    .iter_mut()
                    .chain(oriented.opening_volumes.iter_mut())
                {
                    (volume.start_xmm, volume.start_ymm) =
                        rotate(volume.start_xmm, volume.start_ymm);
                    (volume.end_xmm, volume.end_ymm) = rotate(volume.end_xmm, volume.end_ymm);
                }
                let result = inspect_request(&oriented, &profile);
                assert!(
                    !result.blocks.is_empty(),
                    "rotation={rotation} side={stem_side} {:?}",
                    result.diagnostics
                );
                let bridges: Vec<_> = result.blocks.iter().filter(|b| b.is_bridge).collect();
                assert_eq!(
                    bridges.len(),
                    3,
                    "rotation={rotation} side={stem_side} {:?}",
                    result.diagnostics
                );
                for bridge in bridges {
                    let cuts: Vec<_> = bridge
                        .cuts
                        .iter()
                        .filter(|c| c.starts_with("Type6:"))
                        .collect();
                    assert_eq!(cuts.len(), 1);
                    let expected_face = if stem_side > 0.0 { ":y3:" } else { ":y1:" };
                    assert!(
                        cuts[0].contains(expected_face),
                        "rotation={rotation} side={stem_side} {:?}",
                        bridge.cuts
                    );
                    assert!(result
                        .blocks
                        .iter()
                        .any(|b| b.course_index == bridge.course_index
                            && b.product_key.as_deref() == Some("Type5_1")));
                    assert!(!result
                        .blocks
                        .iter()
                        .any(|b| b.course_index == bridge.course_index
                            && b.product_key.as_deref() == Some("Type10_1")));
                }
            }
        }
    }
}

/// Строгая проверка остается отдельной от результата для просмотра.
pub fn calculate_request(request: &LayoutRequest, profile: &Profile) -> LayoutResponse<Block> {
    let result = inspect_request(request, profile);
    if result.diagnostics.is_empty() {
        LayoutResponse::Success {
            schema_version: 1,
            request_id: request.request_id.clone(),
            snapshot_hash: request.snapshot_hash.clone(),
            profile_id: profile.profile_id.clone(),
            profile_revision: profile.revision.clone(),
            catalog_version: profile.catalog_version.clone(),
            blocks: result.blocks,
            elapsed_ms: result.elapsed_ms,
        }
    } else {
        LayoutResponse::Failure {
            schema_version: 1,
            request_id: request.request_id.clone(),
            snapshot_hash: request.snapshot_hash.clone(),
            profile_id: profile.profile_id.clone(),
            profile_revision: profile.revision.clone(),
            catalog_version: profile.catalog_version.clone(),
            diagnostics: result.diagnostics,
        }
    }
}

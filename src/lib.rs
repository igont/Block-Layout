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
        .filter(|o| o.opening_type == "OPENING" || o.opening_type == "CONSOLE")
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
    // Only a proven full-section obstacle splits the one-dimensional course.
    // Partial pockets stay inside a physical block and are subtracted later.
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
        blocks: layout.layout.blocks,
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

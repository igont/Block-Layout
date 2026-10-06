//! Independent lamella calculation reads saved placements and actual material.
//! It never calls the block solver or publishes block changes.
use crate::api::ApiFailure;
use crate::domain::WallRun;
use crate::exchange::ExchangeRequest;
use crate::lamella::{Lamella, LamellaPlan, Placement};
use crate::layout::Profile;
use crate::solid_geometry::{self, Mesh, Plane};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

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

fn planes(mesh: &Mesh, id: &str) -> Result<Vec<Plane>, ApiFailure> {
    if mesh.vertices.len() < 4
        || mesh.faces.len() < 4
        || mesh.vertices.iter().flatten().any(|v| !v.is_finite())
        || solid_geometry::volume(mesh) <= 1e-6
    {
        return Err(invalid(
            id,
            "Сохранённое тело блока должно быть замкнутым выпуклым объёмом",
        ));
    }
    let center = std::array::from_fn(|i| {
        mesh.vertices.iter().map(|p| p[i]).sum::<f64>() / mesh.vertices.len() as f64
    });
    let mut edges = BTreeMap::new();
    let mut result = Vec::new();
    for face in &mesh.faces {
        if face.len() < 3 || face.iter().any(|i| *i >= mesh.vertices.len()) {
            return Err(invalid(id, "Некорректные грани сохранённого блока"));
        }
        for i in 0..face.len() {
            let a = face[i];
            let b = face[(i + 1) % face.len()];
            *edges.entry((a.min(b), a.max(b))).or_insert(0usize) += 1;
        }
        let p = mesh.vertices[face[0]];
        let mut normal = None;
        for pair in face[1..].windows(2) {
            let a = delta(mesh.vertices[pair[0]], p);
            let b = delta(mesh.vertices[pair[1]], p);
            let n = [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ];
            let len = dot(n, n).sqrt();
            if len > EPS {
                normal = Some(n.map(|v| v / len));
                break;
            }
        }
        let Some(mut normal) = normal else {
            return Err(invalid(id, "В сохранённом блоке есть вырожденная грань"));
        };
        if dot(normal, delta(center, p)) > 0.0 {
            normal = normal.map(|v| -v);
        }
        let offset = dot(normal, p);
        // Две независимо округлённые вершины могут разойтись по нормали
        // на диагональ одного координатного кванта.
        let coordinate_tolerance = 3.0_f64.sqrt() * crate::precision::QUANTUM_MM;
        if face
            .iter()
            .any(|i| (dot(normal, mesh.vertices[*i]) - offset).abs() > coordinate_tolerance)
        {
            return Err(invalid(id, "В сохранённом блоке есть неплоская грань"));
        }
        if mesh
            .vertices
            .iter()
            .any(|v| dot(normal, *v) > offset + coordinate_tolerance)
        {
            return Err(invalid(
                id,
                "Сохранённые тела должны быть выпуклыми частями фактической геометрии блока",
            ));
        }
        result.push(Plane { normal, offset });
    }
    if edges.values().any(|n| *n != 2) {
        return Err(invalid(id, "Поверхность сохранённого блока не замкнута"));
    }
    Ok(result)
}

fn validate(blocks: &[SavedBlock]) -> Result<Vec<Vec<(Mesh, Vec<Plane>)>>, ApiFailure> {
    let mut ids = BTreeSet::new();
    let mut material = Vec::new();
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
        let pieces = b
            .solids
            .iter()
            .map(|mesh| Ok((mesh.clone(), planes(mesh, &b.id)?)))
            .collect::<Result<Vec<_>, ApiFailure>>()?;
        material.push(pieces);
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
    Ok(material)
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

fn mesh(l: &Lamella) -> Mesh {
    if let Some(m) = &l.solid {
        return m.clone();
    }
    let p = &l.placement;
    let axes = [p.x_axis, p.y_axis, p.z_axis];
    let size = [l.length_mm, l.thickness_mm, l.height_mm];
    let origin = std::array::from_fn(|i| {
        p.center_mm[i] - (0..3).map(|j| axes[j][i] * size[j] / 2.0).sum::<f64>()
    });
    solid_geometry::box_mesh(origin, axes, size)
}
fn span(mesh: &Mesh, axis: [f64; 3]) -> f64 {
    let (lo, hi) = mesh
        .vertices
        .iter()
        .map(|p| dot(*p, axis))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
            (lo.min(v), hi.max(v))
        });
    hi - lo
}
fn touches(a: &Mesh, b: &Mesh) -> bool {
    let Some((al, ah)) = solid_geometry::bounds(a) else {
        return false;
    };
    let Some((bl, bh)) = solid_geometry::bounds(b) else {
        return false;
    };
    (0..3).all(|i| al[i] < bh[i] - EPS && bl[i] < ah[i] - EPS)
}

fn nominal_length(b: &SavedBlock) -> f64 {
    match (b.grid_start_mm, b.grid_end_mm) {
        (Some(a), Some(z)) => (z[0] - a[0]).hypot(z[1] - a[1]),
        _ => b.length_mm,
    }
}
fn same_row(b: &SavedBlock, l: &Lamella) -> bool {
    let p = &b.placement;
    (b.wall_ids.iter().any(|id| l.wall_ids.contains(id))
        || (b.wall_ids.is_empty()
            && dot(
                delta(p.origin_mm, l.placement.center_mm),
                l.placement.y_axis,
            )
            .abs()
                < b.width_mm / 2.0 + 0.01))
        && dot(p.x_axis, l.placement.x_axis).abs() > 1.0 - 1e-6
        && p.origin_mm[2] < l.placement.center_mm[2] + l.height_mm / 2.0 - EPS
        && p.origin_mm[2] + b.height_mm > l.placement.center_mm[2] - l.height_mm / 2.0 + EPS
}
fn outside_nominal_cell(b: &SavedBlock, body: &Mesh, axis: [f64; 3]) -> bool {
    let a = b.grid_start_mm.unwrap_or(b.placement.origin_mm);
    let z = b.grid_end_mm.unwrap_or(std::array::from_fn(|i| {
        b.placement.origin_mm[i] + b.placement.x_axis[i] * b.length_mm
    }));
    let start = dot(a, axis).min(dot(z, axis));
    let end = dot(a, axis).max(dot(z, axis));
    let low = body
        .vertices
        .iter()
        .map(|v| dot(*v, axis))
        .fold(f64::INFINITY, f64::min);
    let high = body
        .vertices
        .iter()
        .map(|v| dot(*v, axis))
        .fold(f64::NEG_INFINITY, f64::max);
    high <= start + 0.01 || low >= end - 0.01
}
fn intersection(a: &Mesh, b: &Mesh, planes: &[Plane]) -> Option<Mesh> {
    if !touches(a, b) {
        return None;
    }
    let mut body = a.clone();
    for plane in planes {
        body = solid_geometry::clip(&body, plane)?;
    }
    (solid_geometry::volume(&body) > 1e-5).then_some(body)
}
fn intersects(body: &Mesh, pieces: &[(Mesh, Vec<Plane>)]) -> bool {
    pieces
        .iter()
        .any(|(b, p)| intersection(body, b, p).is_some())
}
fn full_skin_coverage(body: &Mesh, l: &Lamella, pieces: &[(Mesh, Vec<Plane>)]) -> f64 {
    let mut intervals = Vec::new();
    let mut full_thickness = false;
    for (b, p) in pieces {
        if let Some(i) = intersection(body, b, p) {
            full_thickness |= span(&i, l.placement.y_axis) >= 15.0 - 0.01;
            let low = i
                .vertices
                .iter()
                .map(|v| dot(*v, l.placement.x_axis))
                .fold(f64::INFINITY, f64::min);
            let high = i
                .vertices
                .iter()
                .map(|v| dot(*v, l.placement.x_axis))
                .fold(f64::NEG_INFINITY, f64::max);
            intervals.push((low, high));
        }
    }
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut total = 0.0;
    let mut end = f64::NEG_INFINITY;
    for (a, z) in intervals {
        total += (z - a.max(end)).max(0.0);
        end = end.max(z);
    }
    if full_thickness {
        total
    } else {
        0.0
    }
}

pub fn calculate(standard: &ExchangeRequest, profile: &Profile) -> Result<LamellaPlan, ApiFailure> {
    let request = standard.to_layout_request()?;
    standard.validate_profile(profile)?;
    let saved = standard.model.saved_blocks.as_deref().ok_or_else(|| {
        ApiFailure::new("INVALID_EXCHANGE", "Не переданы сохранённые блоки", None)
    })?;
    let mut normalized = saved.to_vec();
    for block in &mut normalized {
        block.placement.origin_mm = block.placement.origin_mm.map(crate::precision::mm);
        block.length_mm = crate::precision::mm(block.length_mm);
        block.width_mm = crate::precision::mm(block.width_mm);
        block.height_mm = crate::precision::mm(block.height_mm);
        block.grid_start_mm = block.grid_start_mm.map(|p| p.map(crate::precision::mm));
        block.grid_end_mm = block.grid_end_mm.map(|p| p.map(crate::precision::mm));
        for mesh in &mut block.solids {
            for point in &mut mesh.vertices {
                *point = point.map(crate::precision::mm);
            }
        }
    }
    let saved = normalized.as_slice();
    let material = validate(saved)?;
    let mut plan = crate::lamella::calculate_from_saved(&request, profile, saved)?;
    let mut finished = Vec::new();
    for l in plan.lamellas {
        let original = mesh(&l);
        let original_volume = solid_geometry::volume(&original);
        let module = profile.ordinary_length_centimm as f64 / 100.0;
        if saved.iter().zip(&material).any(|(b, p)| {
            same_row(b, &l)
                && nominal_length(b) >= module - 10.0 - 0.01
                // A whole stock body's spikes and interlocking facade wedges
                // can leave gaps at the outer face. Substantial skin material
                // still makes this a complete block cell, rather than a local
                // patch in an already recessed body.
                && full_skin_coverage(&original, &l, p) > l.length_mm.min(module) / 2.0
        }) {
            continue;
        }
        let mut allowed_material = BTreeSet::new();
        for (b, p) in saved.iter().zip(&material) {
            if same_row(b, &l)
                && (nominal_length(b) < module - 10.0 - 0.01
                    || outside_nominal_cell(b, &original, l.placement.x_axis))
                && intersects(&original, p)
            {
                allowed_material.insert(b.id.clone());
            }
        }
        let mut pieces = vec![original];
        for (saved_block, block_pieces) in saved.iter().zip(&material) {
            if allowed_material.contains(&saved_block.id) {
                continue;
            }
            for (block, planes) in block_pieces {
                pieces = pieces
                    .into_iter()
                    .flat_map(|piece| {
                        if touches(&piece, block) {
                            // Split along the facade first. Cutting thickness first
                            // creates thin fragments even in the unobstructed ends.
                            let mut ordered = planes.clone();
                            ordered.sort_by(|a, b| {
                                dot(a.normal, l.placement.y_axis)
                                    .abs()
                                    .total_cmp(&dot(b.normal, l.placement.y_axis).abs())
                            });
                            let fragments = solid_geometry::subtract(&piece, &ordered);
                            let retained: f64 = fragments.iter().map(solid_geometry::volume).sum();
                            if (retained - solid_geometry::volume(&piece)).abs() < 1e-5 {
                                vec![piece]
                            } else {
                                fragments
                            }
                        } else {
                            vec![piece]
                        }
                    })
                    .collect();
                if pieces.is_empty() {
                    break;
                }
            }
        }
        let remainder: f64 = pieces.iter().map(solid_geometry::volume).sum();
        if remainder < original_volume - 1e-5 {
            let mut warning = ApiFailure::new(
                "LAMELLA_SAVED_MATERIAL_CLIPPED",
                "Ламель подрезана по сохранённому материалу блоков; сохранённые блоки не изменены",
                None,
            );
            warning.course_index = Some(l.course_index);
            warning.source_ids = l.source_ids.iter().cloned().collect();
            plan.warnings.push(warning);
        }
        for (index, body) in pieces.into_iter().enumerate() {
            if (span(&body, l.placement.y_axis) - 15.0).abs() > 0.01 {
                let mut warning = ApiFailure::new(
                    "LAMELLA_SAVED_MATERIAL_THIN_FRAGMENT",
                    "Тонкий остаток ламели пропущен: сохранённый блок занимает часть толщины 15 мм",
                    None,
                );
                warning.course_index = Some(l.course_index);
                warning.source_ids = l.source_ids.iter().cloned().collect();
                plan.warnings.push(warning);
                continue;
            }
            if (solid_geometry::volume(&body) - original_volume).abs() < 1e-5 {
                finished.push(l.clone());
                continue;
            }
            let mut part = l.clone();
            part.id = format!("{}:saved:{index}", l.id);
            part.length_mm = span(&body, part.placement.x_axis);
            part.height_mm = span(&body, part.placement.z_axis);
            part.placement = Placement {
                center_mm: std::array::from_fn(|i| {
                    body.vertices.iter().map(|p| p[i]).sum::<f64>() / body.vertices.len() as f64
                }),
                ..part.placement
            };
            part.solid = Some(body);
            finished.push(part);
        }
    }
    assign_grid_recesses(&mut finished, saved);
    finished.sort_by(|a, b| a.id.cmp(&b.id));
    plan.lamellas = finished;
    Ok(plan)
}

/// Третий проход связывает срез стороны с опубликованными ячейками ламелей.
/// Физические тела, шипы и геометрические подрезки в нём не участвуют.
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
                    && !block.wall_ids.iter().any(|id| lamella.wall_ids.contains(id)))
                || dot(delta(block.placement.origin_mm, lamella.grid_start_mm), lamella.placement.y_axis).abs()
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
    let mut plan = calculate(standard, profile)?;
    crate::lamella::quantize_solids(&mut plan.lamellas);
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

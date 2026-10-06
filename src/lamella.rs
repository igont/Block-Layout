//! Physical facade strips use the same canonical run and course grid as blocks.
//! The finished outer face stays on the wall contour; all frames are centered.
use crate::api::{ApiFailure, LayoutRequest};
use crate::domain::{Course, WallRun};
pub use crate::lamella_saved::SavedBlock;
use crate::layout::{Block, Profile};
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::BTreeSet;

const THICKNESS: f64 = 15.0;
const EPS: f64 = 1e-7;

#[derive(Clone, Copy, Debug)]
struct V(f64, f64);
impl V {
    fn add(self, b: Self) -> Self {
        Self(self.0 + b.0, self.1 + b.1)
    }
    fn sub(self, b: Self) -> Self {
        Self(self.0 - b.0, self.1 - b.1)
    }
    fn scale(self, s: f64) -> Self {
        Self(self.0 * s, self.1 * s)
    }
    fn dot(self, b: Self) -> f64 {
        self.0 * b.0 + self.1 * b.1
    }
    fn cross(self, b: Self) -> f64 {
        self.0 * b.1 - self.1 * b.0
    }
}
fn point(p: crate::domain::Point) -> V {
    V(p.x as f64 / 100.0, p.y as f64 / 100.0)
}
fn frame(run: &WallRun) -> (V, V, V, f64) {
    let o = point(run.start);
    let d = point(run.end).sub(o);
    let len = d.0.hypot(d.1);
    let u = d.scale(1.0 / len);
    (o, u, V(-u.1, u.0), len)
}

#[derive(Clone, Debug, Serialize)]
pub struct Placement {
    pub center_mm: [f64; 3],
    pub x_axis: [f64; 3],
    pub y_axis: [f64; 3],
    pub z_axis: [f64; 3],
}
#[derive(Clone, Debug, Serialize)]
pub struct Lamella {
    pub id: String,
    pub placement: Placement,
    pub length_mm: f64,
    pub thickness_mm: f64,
    pub height_mm: f64,
    pub course_index: i64,
    pub side: &'static str,
    pub wall_ids: BTreeSet<String>,
    pub beam_ids: BTreeSet<String>,
    pub source_ids: BTreeSet<String>,
    // Номинальная ячейка сохраняется до подрезки физического тела.
    #[serde(skip)]
    pub grid_start_mm: [f64; 3],
    #[serde(skip)]
    pub grid_end_mm: [f64; 3],
    #[serde(skip_serializing_if = "BTreeSet::is_empty")]
    pub recess_block_ids: BTreeSet<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solid: Option<crate::solid_geometry::Mesh>,
}

pub struct LamellaPlan {
    pub lamellas: Vec<Lamella>,
    pub warnings: Vec<ApiFailure>,
}

/// Топология готового результата приводится к точности обмена после расчёта.
pub fn quantize_solids(lamellas: &mut [Lamella]) {
    for lamella in lamellas {
        if let Some(solid) = &lamella.solid {
            lamella.solid = Some(crate::solid_geometry::quantized(solid));
        }
    }
}

/// Clip a convex rectangle in local run coordinates. This avoids accepting a
/// diagonal beam merely because its axis-aligned bounding box touches a wall.
fn clip(poly: Vec<V>, axis: usize, bound: f64, lower: bool) -> Vec<V> {
    let coordinate = |p: V| if axis == 0 { p.0 } else { p.1 };
    let inside = |p: V| {
        if lower {
            coordinate(p) >= bound - EPS
        } else {
            coordinate(p) <= bound + EPS
        }
    };
    let mut out = Vec::new();
    if poly.is_empty() {
        return out;
    }
    let mut prev = poly[poly.len() - 1];
    for next in poly {
        if inside(prev) != inside(next) {
            let t = (bound - coordinate(prev)) / (coordinate(next) - coordinate(prev));
            out.push(prev.add(next.sub(prev).scale(t)));
        }
        if inside(next) {
            out.push(next);
        }
        prev = next;
    }
    out
}

struct Region {
    low: f64,
    high: f64,
    bottom: f64,
    top: f64,
    beam: String,
    side: &'static str,
}
fn regions(
    request: &LayoutRequest,
    run: &WallRun,
    low: f64,
    high: f64,
    half_width: f64,
    bottom: f64,
    top: f64,
) -> Vec<Region> {
    let (o, u, v, _) = frame(run);
    request
        .beams
        .iter()
        .filter_map(|b| {
            let g = &b.geometry;
            if (b.start_zmm - b.end_zmm).abs() > EPS
                || g.height_direction_x.abs() > EPS
                || g.height_direction_y.abs() > EPS
                || (g.height_direction_z.abs() - 1.0).abs() > EPS
            {
                return None;
            }
            let bz = b.start_zmm + g.height_mm * g.height_direction_z;
            let zlow = bottom.max(b.start_zmm.min(bz));
            let zhigh = top.min(b.start_zmm.max(bz));
            if zhigh <= zlow + EPS {
                return None;
            }
            let a = V(b.start_xmm, b.start_ymm);
            let d = V(b.end_xmm, b.end_ymm).sub(a);
            let length = d.0.hypot(d.1);
            if length <= EPS {
                return None;
            }
            let normal = V(-d.1, d.0).scale(g.width_mm / (2.0 * length));
            let end = a.add(d);
            let mut polygon: Vec<_> = [
                a.add(normal),
                end.add(normal),
                end.sub(normal),
                a.sub(normal),
            ]
            .into_iter()
            .map(|p| {
                let delta = p.sub(o);
                V(delta.dot(u), delta.dot(v))
            })
            .collect();
            for (axis, bound, lower) in [
                (0, low, true),
                (0, high, false),
                (1, -half_width, true),
                (1, half_width, false),
            ] {
                polygon = clip(polygon, axis, bound, lower);
            }
            if polygon.len() < 3 {
                return None;
            }
            let area: f64 = polygon
                .iter()
                .zip(polygon.iter().cycle().skip(1))
                .take(polygon.len())
                .map(|(a, b)| a.cross(*b))
                .sum();
            if area.abs() <= EPS {
                return None;
            }
            // A longitudinal beam leaves the same construction gap on both
            // faces (160mm beam / 193mm wall: 1.5mm after a 15mm skin). Its
            // nominal width defines that gap. Transverse/skew beams need to
            // physically reach the skin; an endpoint inside the wall must not
            // create a cover on the opposite facade.
            let parallel = d.scale(1.0 / length).dot(u).abs() > 1.0 - EPS;
            let gap = if parallel {
                (half_width - g.width_mm / 2.0 - THICKNESS).max(0.0)
            } else {
                0.0
            };
            let inner = half_width - THICKNESS - gap;
            let mut found = Vec::new();
            for (side, positive) in [("left", true), ("right", false)] {
                let skin = clip(
                    polygon.clone(),
                    1,
                    if positive { inner } else { -inner },
                    positive,
                );
                if skin.is_empty() {
                    continue;
                }
                let low = skin.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
                let high = skin.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
                if high <= low + EPS {
                    continue;
                }
                found.push(Region {
                    low,
                    high,
                    bottom: zlow,
                    top: zhigh,
                    beam: b.guid.clone(),
                    side,
                });
            }
            Some(found)
        })
        .flatten()
        .collect()
}

fn wall_width(request: &LayoutRequest, run: &WallRun, start: bool) -> Option<f64> {
    let source = if start {
        run.sources.first()?
    } else {
        run.sources.last()?
    };
    request
        .wall_volumes
        .iter()
        .find(|w| w.guid == source.wall_id)
        .map(|w| w.thickness_mm)
}

/// The line intersection locates the ideal corner. One canonical run covers
/// the complete corner square; the other butts on its inner face. The same
/// construction shortens inside corners and extends outside corners.
fn corner_bound(
    request: &LayoutRequest,
    course: &Course,
    run: &WallRun,
    start: bool,
    side: f64,
    offset: f64,
) -> f64 {
    let (o, u, v, len) = frame(run);
    let node = if start { run.start } else { run.end };
    let inward = u.scale(if start { 1.0 } else { -1.0 });
    let center = point(node).add(v.scale(side * offset));
    let mut bound = if start { 0.0 } else { len };
    for other in &course.runs {
        if other.id == run.id {
            continue;
        }
        let other_start = other.start == node;
        let (other_origin, ou, ov, other_len) = frame(other);
        let delta = point(node).sub(other_origin);
        let through =
            delta.dot(ov).abs() < EPS && delta.dot(ou) > EPS && delta.dot(ou) < other_len - EPS;
        if !other_start && other.end != node && !through {
            continue;
        }
        if through {
            // At a T the uninterrupted facade wins. The branch starts at its
            // contour, including the branch's finite transverse half-width.
            if inward.dot(ov).abs() < EPS {
                continue;
            }
            let Some(width) = wall_width(request, other, true) else {
                continue;
            };
            let distance = (width / 2.0 + THICKNESS / 2.0 * v.dot(ov).abs()) / inward.dot(ov).abs();
            if start {
                bound = bound.max(distance);
            } else {
                bound = bound.min(len - distance);
            }
            continue;
        }
        let other_inward = ou.scale(if other_start { 1.0 } else { -1.0 });
        // Parallel continuations have no corner.
        if u.cross(ou).abs() < EPS {
            continue;
        }
        let Some(width) = wall_width(request, other, other_start) else {
            continue;
        };
        let quadrant = v.scale(side).dot(other_inward).signum();
        let other_side = (ov.dot(inward) * quadrant).signum();
        let other_center = point(node).add(ov.scale(other_side * (width / 2.0 - THICKNESS / 2.0)));
        let at = other_center.sub(center).cross(ou) / u.cross(ou);
        let owner = (run.start, run.end) < (other.start, other.end);
        let outward = if start { -1.0 } else { 1.0 };
        let finite = THICKNESS / 2.0 / u.cross(ou).abs();
        let oblique_envelope = THICKNESS / 2.0 * u.dot(ou).abs() / u.cross(ou).abs();
        let candidate = if start { at } else { len + at }
            + outward * finite * if owner { 1.0 } else { -1.0 }
            + outward * oblique_envelope;
        // A T/X branch can create two inside limits. Keep the more restrictive
        // inner limit; at a simple L the outside limit is retained as well.
        if course
            .runs
            .iter()
            .filter(|r| r.start == node || r.end == node)
            .count()
            == 2
        {
            bound = candidate;
        } else if start {
            bound = bound.max(candidate);
        } else {
            bound = bound.min(candidate);
        }
    }
    // Offset is projected from the common node, hence absolute run origin.
    let _ = o;
    bound
}

fn clean(v: f64) -> f64 {
    if v.abs() < EPS {
        0.0
    } else {
        v
    }
}

/// Exact affine height boundaries preserve partial gable rows. Only changed
/// bodies carry a mesh; ordinary rectangular rows retain the compact contract.
fn clip_wall_height(
    l: &mut Lamella,
    wall: &crate::api::Volume,
    corner_planes: &[crate::solid_geometry::Plane],
    full_course: bool,
) -> bool {
    use crate::solid_geometry::{self, Plane};
    let p = &l.placement;
    let axes = [p.x_axis, p.y_axis, p.z_axis];
    let sizes = [l.length_mm, l.thickness_mm, l.height_mm];
    let origin = std::array::from_fn(|i| {
        p.center_mm[i] - (0..3).map(|j| axes[j][i] * sizes[j] / 2.0).sum::<f64>()
    });
    let original = solid_geometry::box_mesh(origin, axes, sizes);
    let dx = wall.end_xmm - wall.start_xmm;
    let dy = wall.end_ymm - wall.start_ymm;
    let norm = dx * dx + dy * dy;
    let mut mesh = original.clone();
    for (start, end, upper) in [
        (wall.start_top_zmm, wall.end_top_zmm, true),
        (wall.start_bottom_zmm, wall.end_bottom_zmm, false),
    ] {
        if full_course {
            continue;
        }
        let gradient = [(end - start) * dx / norm, (end - start) * dy / norm];
        let sign = if upper { 1.0 } else { -1.0 };
        let plane = Plane {
            normal: [-sign * gradient[0], -sign * gradient[1], sign],
            offset: sign * (start - gradient[0] * wall.start_xmm - gradient[1] * wall.start_ymm),
        };
        let Some(clipped) = solid_geometry::clip(&mesh, &plane) else {
            return false;
        };
        mesh = clipped;
    }
    for plane in corner_planes {
        let Some(clipped) = solid_geometry::clip(&mesh, plane) else {
            return false;
        };
        mesh = clipped;
    }
    if mesh != original {
        // The vertex mean is a convex combination, hence lies inside the actual
        // finished body even when a rectangular envelope center would be outside.
        l.placement.center_mm = std::array::from_fn(|i| {
            mesh.vertices.iter().map(|v| v[i]).sum::<f64>() / mesh.vertices.len() as f64
        });
        let span = |axis: [f64; 3]| {
            let values = mesh
                .vertices
                .iter()
                .map(|p| (0..3).map(|i| p[i] * axis[i]).sum::<f64>());
            let (low, high) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v), hi.max(v))
            });
            high - low
        };
        l.length_mm = span(l.placement.x_axis);
        l.height_mm = span(l.placement.z_axis);
        l.solid = Some(mesh);
    }
    true
}

fn oblique_corner_planes(
    request: &LayoutRequest,
    course: &Course,
    run: &WallRun,
    side: f64,
) -> Vec<crate::solid_geometry::Plane> {
    let (_, u, v, _) = frame(run);
    let mut planes = Vec::new();
    for start in [true, false] {
        let node = if start { run.start } else { run.end };
        let inward = u.scale(if start { 1.0 } else { -1.0 });
        for other in &course.runs {
            if run.id == other.id {
                continue;
            }
            let other_start = other.start == node;
            if !other_start && other.end != node {
                continue;
            }
            let (_, ou, ov, _) = frame(other);
            if u.cross(ou).abs() < EPS || u.dot(ou).abs() < EPS {
                continue;
            }
            let Some(width) = wall_width(request, other, other_start) else {
                continue;
            };
            let quadrant = v
                .scale(side)
                .dot(ou.scale(if other_start { 1.0 } else { -1.0 }))
                .signum();
            let other_side = (ov.dot(inward) * quadrant).signum();
            let center = point(node).add(ov.scale(other_side * (width / 2.0 - THICKNESS / 2.0)));
            let normal = ov.scale(-ov.dot(inward).signum());
            let owner = (run.start, run.end) < (other.start, other.end);
            planes.push(crate::solid_geometry::Plane {
                normal: [normal.0, normal.1, 0.0],
                offset: normal.dot(center) + THICKNESS / 2.0 * if owner { 1.0 } else { -1.0 },
            });
        }
    }
    planes
}

/// Export physical lamellas even when a beam removes every ordinary block in
/// its course. Grid residue comes from layout's authoritative policy, not a
/// surviving reference product.
pub fn calculate(
    request: &LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
) -> Result<LamellaPlan, ApiFailure> {
    calculate_internal(request, profile, blocks, None)
}

pub(crate) fn calculate_from_saved(
    request: &LayoutRequest,
    profile: &Profile,
    saved: &[SavedBlock],
) -> Result<LamellaPlan, ApiFailure> {
    calculate_internal(request, profile, &[], Some(saved))
}

fn calculate_internal(
    request: &LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
    saved: Option<&[SavedBlock]>,
) -> Result<LamellaPlan, ApiFailure> {
    if request.beams.is_empty() {
        return Ok(LamellaPlan {
            lamellas: Vec::new(),
            warnings: Vec::new(),
        });
    }
    let mut effective = request.normalized();
    crate::effective_geometry::normalize_beams(&mut effective);
    let building = crate::topology::normalize_building(effective.raw_building())
        .map_err(|e| ApiFailure::new("LAMELLA_TOPOLOGY_FAILED", e, None))?;
    let topology = crate::topology::build_topology(building)
        .map_err(|e| ApiFailure::new("LAMELLA_TOPOLOGY_FAILED", e, None))?;
    let mut out = Vec::new();
    let mut warnings = Vec::new();
    for course in &topology.courses {
        for run in &course.runs {
            let (o, u, v, len) = frame(run);
            let residue = if saved.is_none() {
                crate::layout::ordinary_residue(run, course.index, profile)
                    .map(|r| r as f64 / 100.0)
            } else {
                None
            };
            for source in &run.sources {
                let Some(wall) = effective
                    .wall_volumes
                    .iter()
                    .find(|w| w.guid == source.wall_id)
                else {
                    continue;
                };
                // A source-wall boundary is provenance, not a physical joint.
                // Equal adjoining sections share full cells on the chosen run.
                let course_bottom = course.z as f64 / 100.0;
                let course_top = course_bottom + profile.index_centimm as f64 / 100.0;
                let all_contain_course = run.sources.iter().all(|s| {
                    effective
                        .wall_volumes
                        .iter()
                        .find(|w| w.guid == s.wall_id)
                        .is_some_and(|w| {
                            course_bottom >= w.start_bottom_zmm.max(w.end_bottom_zmm) - EPS
                                && course_top <= w.start_top_zmm.min(w.end_top_zmm) + EPS
                        })
                });
                let same_section = run.sources.iter().all(|s| {
                    effective
                        .wall_volumes
                        .iter()
                        .find(|w| w.guid == s.wall_id)
                        .is_some_and(|w| {
                            (w.thickness_mm - wall.thickness_mm).abs() < EPS
                                && (all_contain_course
                                    || ((w.start_bottom_zmm - wall.start_bottom_zmm).abs() < EPS
                                        && (w.end_bottom_zmm - wall.end_bottom_zmm).abs() < EPS
                                        && (w.start_top_zmm - wall.start_top_zmm).abs() < EPS
                                        && (w.end_top_zmm - wall.end_top_zmm).abs() < EPS))
                        })
                });
                if same_section && source != &run.sources[0] {
                    continue;
                }
                let low = if same_section {
                    0.0
                } else {
                    source.start_offset as f64 / 100.0
                };
                let high = if same_section {
                    len
                } else {
                    source.end_offset as f64 / 100.0
                };
                let bottom = course_bottom.max(wall.start_bottom_zmm.min(wall.end_bottom_zmm));
                let top = (course_bottom + profile.index_centimm as f64 / 100.0)
                    .min(wall.start_top_zmm.max(wall.end_top_zmm));
                let areas = regions(
                    &effective,
                    run,
                    low,
                    high,
                    wall.thickness_mm / 2.0,
                    bottom,
                    top,
                );
                if areas.is_empty() {
                    continue;
                }
                let mut seams = Vec::new();
                if let Some(saved) = saved {
                    let Some(selected) = crate::lamella_saved::seams(
                        saved,
                        run,
                        &wall.guid,
                        bottom,
                        top,
                        course.index,
                        profile,
                        low,
                        high,
                    ) else {
                        let mut warning=ApiFailure::new("LAMELLA_SAVED_GRID_UNAVAILABLE",
                            "Для ламелей нет сохранённой сетки блоков в этом венце; блоки не изменены",Some(wall.guid.clone()));
                        warning.course_index = Some(course.index);
                        warnings.push(warning);
                        continue;
                    };
                    seams = selected;
                } else if let Some(residue) = residue {
                    let module = profile.ordinary_length_centimm as f64 / 100.0;
                    let mut seam = residue + ((low - residue) / module).floor() * module;
                    while seam < high + module - EPS {
                        seams.push(seam);
                        seam += module;
                    }
                } else {
                    // A custom non-world-grid profile uses its actual selected
                    // physical joints. There is no inferred 640mm reference.
                    let mut has_selected_joint = false;
                    for b in blocks.iter().filter(|b| b.course_index == course.index) {
                        for (a, z) in std::iter::once((b.start, b.end))
                            .chain(b.arms.iter().map(|a| (a.start, a.end)))
                        {
                            if point(a).sub(o).dot(v).abs() < EPS
                                && point(z).sub(o).dot(v).abs() < EPS
                            {
                                has_selected_joint = true;
                                seams.extend([point(a).sub(o).dot(u), point(z).sub(o).dot(u)]);
                            }
                        }
                    }
                    if !has_selected_joint {
                        let mut warning=ApiFailure::new("LAMELLA_GRID_UNAVAILABLE",
                            "Ламели покрывают не всю область балки: для пользовательского профиля в этом венце нет выбранных швов сетки",
                            Some(wall.guid.clone()));
                        warning.course_index = Some(course.index);
                        warning.source_ids = areas.iter().map(|a| a.beam.clone()).collect();
                        warnings.push(warning);
                        continue;
                    }
                    seams.extend([low, high]);
                }
                seams.sort_by(f64::total_cmp);
                seams.dedup_by(|a, b| (*a - *b).abs() < EPS);
                let mut zs = vec![bottom, top];
                for area in &areas {
                    zs.extend([area.bottom, area.top]);
                }
                zs.sort_by(f64::total_cmp);
                zs.dedup_by(|a, b| (*a - *b).abs() < EPS);
                for (side, sign) in [("left", 1.0), ("right", -1.0)] {
                    let offset = wall.thickness_mm / 2.0 - THICKNESS / 2.0;
                    let corner_planes = oblique_corner_planes(&effective, course, run, sign);
                    let start = if low < EPS {
                        corner_bound(&effective, course, run, true, sign, offset)
                    } else {
                        low
                    };
                    let end = if (high - len).abs() < EPS {
                        corner_bound(&effective, course, run, false, sign, offset)
                    } else {
                        high
                    };
                    for pair in seams.windows(2) {
                        let a = if pair[0] <= low + EPS {
                            start
                        } else {
                            pair[0].max(start)
                        };
                        let b = if pair[1] >= high - EPS {
                            end
                        } else {
                            pair[1].min(end)
                        };
                        if b <= a + EPS {
                            continue;
                        }
                        for z in zs.windows(2) {
                            let selected: Vec<_> = areas
                                .iter()
                                .filter(|r| {
                                    r.side == side
                                        && r.low < pair[1] - EPS
                                        && r.high > pair[0] + EPS
                                        && r.bottom < z[1] - EPS
                                        && r.top > z[0] + EPS
                                })
                                .collect();
                            if selected.is_empty() {
                                continue;
                            }
                            let beam_ids: BTreeSet<_> =
                                selected.iter().map(|r| r.beam.clone()).collect();
                            let wall_ids = if same_section {
                                run.sources
                                    .iter()
                                    .filter(|s| {
                                        s.start_offset as f64 / 100.0 < b - EPS
                                            && s.end_offset as f64 / 100.0 > a + EPS
                                    })
                                    .map(|s| s.wall_id.clone())
                                    .collect()
                            } else {
                                BTreeSet::from([wall.guid.clone()])
                            };
                            let source_ids = wall_ids.union(&beam_ids).cloned().collect();
                            let center = o.add(u.scale((a + b) / 2.0)).add(v.scale(sign * offset));
                            let mut lamella = Lamella {
                                id: format!(
                                    "lamella:{}:{}:{}:{:016x}:{:016x}",
                                    course.index,
                                    run.id,
                                    side,
                                    clean(a).to_bits(),
                                    clean(z[0]).to_bits()
                                ),
                                placement: Placement {
                                    center_mm: [
                                        clean(center.0),
                                        clean(center.1),
                                        (z[0] + z[1]) / 2.0,
                                    ],
                                    x_axis: [clean(u.0), clean(u.1), 0.0],
                                    y_axis: [clean(v.0), clean(v.1), 0.0],
                                    z_axis: [0.0, 0.0, 1.0],
                                },
                                length_mm: b - a,
                                thickness_mm: THICKNESS,
                                height_mm: z[1] - z[0],
                                course_index: course.index,
                                side,
                                wall_ids,
                                beam_ids,
                                source_ids,
                                grid_start_mm: [o.0 + u.0 * pair[0], o.1 + u.1 * pair[0], z[0]],
                                grid_end_mm: [o.0 + u.0 * pair[1], o.1 + u.1 * pair[1], z[0]],
                                recess_block_ids: BTreeSet::new(),
                                solid: None,
                            };
                            if clip_wall_height(
                                &mut lamella,
                                wall,
                                &corner_planes,
                                all_contain_course && same_section && run.sources.len() > 1,
                            ) {
                                out.push(lamella);
                            }
                        }
                    }
                }
            }
        }
    }
    out.sort_by(|a, b| a.id.cmp(&b.id));
    Ok(LamellaPlan {
        lamellas: out,
        warnings,
    })
}

fn overlaps(
    l: &Lamella,
    center: [f64; 3],
    axes: ([f64; 3], [f64; 3]),
    length: f64,
    width: f64,
    height: f64,
) -> bool {
    let mut lc = l.placement.center_mm;
    let mut ll = l.length_mm;
    let mut lh = l.height_mm;
    if let Some(mesh) = &l.solid {
        let u = l.placement.x_axis;
        let v = l.placement.y_axis;
        let range = |axis: [f64; 3]| {
            let values = mesh
                .vertices
                .iter()
                .map(|p| (0..3).map(|i| p[i] * axis[i]).sum::<f64>())
                .collect::<Vec<_>>();
            (
                values.iter().copied().fold(f64::INFINITY, f64::min),
                values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            )
        };
        let ur = range(u);
        let vr = range(v);
        let zr = range([0.0, 0.0, 1.0]);
        lc = [
            u[0] * (ur.0 + ur.1) / 2.0 + v[0] * (vr.0 + vr.1) / 2.0,
            u[1] * (ur.0 + ur.1) / 2.0 + v[1] * (vr.0 + vr.1) / 2.0,
            (zr.0 + zr.1) / 2.0,
        ];
        ll = ur.1 - ur.0;
        lh = zr.1 - zr.0;
    }
    if (lc[2] - center[2]).abs() >= (lh + height) / 2.0 - EPS {
        return false;
    }
    let delta = V(lc[0] - center[0], lc[1] - center[1]);
    let a = V(l.placement.x_axis[0], l.placement.x_axis[1]);
    let b = V(l.placement.y_axis[0], l.placement.y_axis[1]);
    let x = V(axes.0[0], axes.0[1]);
    let y = V(axes.1[0], axes.1[1]);
    [a, b, x, y].iter().all(|axis| {
        delta.dot(*axis).abs()
            < (length * x.dot(*axis).abs()
                + width * y.dot(*axis).abs()
                + ll * a.dot(*axis).abs()
                + THICKNESS * b.dot(*axis).abs())
                / 2.0
                - EPS
    })
}

/// Physical subtraction is explicit and uses the identical centered frame.
/// A cutter may extend outside the block; SUP intersects it with the product.
pub fn add_recesses(
    blocks: &mut [Value],
    originals: &[Block],
    lamellas: &[Lamella],
    request: &LayoutRequest,
) {
    for block in blocks {
        let p = &block["placement"];
        let array =
            |v: &Value| -> [f64; 3] { std::array::from_fn(|i| v[i].as_f64().unwrap_or(0.0)) };
        let x = array(&p["x_axis"]);
        let y = array(&p["y_axis"]);
        let origin = array(&p["origin_mm"]);
        let length = block["length_mm"].as_f64().unwrap_or(0.0);
        let width = block["width_mm"].as_f64().unwrap_or(0.0);
        let height = block["height_mm"].as_f64().unwrap_or(0.0);
        let center = [
            origin[0] + x[0] * length / 2.0,
            origin[1] + x[1] * length / 2.0,
            origin[2] + height / 2.0,
        ];
        let original = originals
            .iter()
            .find(|b| Some(b.id.as_str()) == block["id"].as_str());
        for l in lamellas {
            let hit = overlaps(l, center, (x, y), length, width, height)
                || original.is_some_and(|b| {
                    b.arms.iter().any(|arm| {
                        let a = point(arm.start);
                        let e = point(arm.end);
                        let d = e.sub(a);
                        let len = d.0.hypot(d.1);
                        if len < EPS {
                            return false;
                        }
                        let u = d.scale(1.0 / len);
                        let w = request
                            .wall_volumes
                            .iter()
                            .find(|w| w.guid == arm.wall_id)
                            .map(|w| w.thickness_mm)
                            .unwrap_or(width);
                        overlaps(
                            l,
                            [
                                (a.0 + e.0) / 2.0,
                                (a.1 + e.1) / 2.0,
                                b.z_centimm as f64 / 100.0 + height / 2.0,
                            ],
                            ([u.0, u.1, 0.0], [-u.1, u.0, 0.0]),
                            len,
                            w,
                            height,
                        )
                    })
                });
            if hit {
                if let Some(trims) = block["trims"].as_array_mut() {
                    let mut trim = json!({"kind":"lamella_recess","lamella_id":l.id,
                "placement":l.placement,"length_mm":l.length_mm,"thickness_mm":l.thickness_mm,"height_mm":l.height_mm});
                    if let Some(solid) = &l.solid {
                        trim["solid"] = json!(solid);
                    }
                    trims.push(trim);
                }
            }
        }
    }
}

//! Physical facade strips use the same canonical run and course grid as blocks.
//! The finished outer face stays on the wall contour; all frames are centered.
use crate::api::{ApiFailure, LayoutRequest};
use crate::domain::WallRun;
pub use crate::lamella_saved::SavedBlock;
use crate::layout::{Block, Profile};
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::BTreeSet;

mod facade;
use facade::Facade;

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
    // Полная ячейка сетки определяет тело ламели и срезы блоков.
    #[serde(skip)]
    pub grid_start_mm: [f64; 3],
    #[serde(skip)]
    pub grid_end_mm: [f64; 3],
    // Whole-side recesses must fit between the adjacent openings and beam faces.
    #[serde(skip)]
    pub(crate) recess_limits: [Option<f64>; 2],
    #[serde(skip_serializing_if = "BTreeSet::is_empty")]
    pub recess_block_ids: BTreeSet<String>,
}

pub struct LamellaPlan {
    pub lamellas: Vec<Lamella>,
    pub warnings: Vec<ApiFailure>,
}

struct Region {
    low: f64,
    high: f64,
    beam: String,
    end_cover: bool,
}
// End allowance must not select another grid cell. Keep genuinely short beams.
const BEAM_END_ALLOWANCE_MM: f64 = 5.0;
fn snap_interval(low: f64, high: f64, boundaries: &[f64]) -> (f64, f64) {
    let snap = |value: f64| {
        boundaries
            .iter()
            .copied()
            .filter(|edge| (value - edge).abs() <= BEAM_END_ALLOWANCE_MM + EPS)
            .min_by(|a, b| (value - a).abs().total_cmp(&(value - b).abs()))
            .unwrap_or(value)
    };
    let (a, b) = (snap(low), snap(high));
    if b > a + EPS {
        (a, b)
    } else {
        (low, high)
    }
}

struct CourseCoverage {
    regions: Vec<Region>,
    stops: Vec<(f64, f64)>,
}

fn course_coverage(
    request: &LayoutRequest,
    facade: &Facade<'_>,
    bottom: f64,
    course_height: f64,
) -> CourseCoverage {
    let (o, u, v, len) = frame(&facade.run);
    let project = |x, y| {
        let local = V(x, y).sub(o);
        (local.dot(u), local.dot(v))
    };
    let top = bottom + course_height;
    let mut result = CourseCoverage {
        regions: Vec::new(),
        stops: request
            .opening_volumes
            .iter()
            .filter(|opening| crate::effective_geometry::is_fb_opening(opening))
            .filter_map(|opening| {
                crate::effective_geometry::opening_course_interval(
                    opening,
                    project(opening.start_xmm, opening.start_ymm),
                    project(opening.end_xmm, opening.end_ymm),
                    facade.half_width,
                    [bottom, top],
                )
            })
            .map(|(a, b)| (a as f64 / 100.0, b as f64 / 100.0))
            .collect(),
    };
    let outer = facade.half_width;
    let inner = outer - THICKNESS;
    let sign = facade.side.sign();
    for beam in &request.beams {
        let g = &beam.geometry;
        if !crate::effective_geometry::horizontal(beam) {
            continue;
        }
        let bz = beam.start_zmm + g.height_mm * g.height_direction_z;
        let zlow = beam.start_zmm.min(bz);
        let zhigh = beam.start_zmm.max(bz);
        let zgrid = [zlow, zhigh]
            .map(|z| request.z0_mm + ((z - request.z0_mm) / course_height).round() * course_height);
        let (seed_bottom, seed_top) = snap_interval(zlow, zhigh, &zgrid);
        let seeds_course = seed_bottom < top - EPS && seed_top > bottom + EPS;
        let a = V(beam.start_xmm, beam.start_ymm).sub(o);
        let e = V(beam.end_xmm, beam.end_ymm).sub(o);
        let d = e.sub(a);
        let length = d.0.hypot(d.1);
        if length <= EPS {
            continue;
        }
        let direction = d.scale(1.0 / length);
        let half_beam = g.width_mm / 2.0;
        let gap = (inner - half_beam).max(0.0);
        let axial = direction.dot(u).abs();
        if axial < 1.0 - EPS {
            // A blind end can replace the last stock behind the opposite facing.
            let [av, bv] = [a, e].map(|p| sign * p.dot(v));
            let end = av.max(bv);
            let center = a.dot(u);
            if (center - e.dot(u)).abs() <= EPS
                && av.min(bv) <= -outer + EPS
                && end >= inner - gap - EPS
                && end <= inner + EPS
                && seeds_course
                && center + half_beam > facade.low + EPS
                && center - half_beam < facade.high - EPS
            {
                result.regions.push(Region {
                    low: center - half_beam,
                    high: center + half_beam,
                    beam: beam.guid.clone(),
                    end_cover: true,
                });
            }
            if zlow >= top - EPS || zhigh <= bottom + EPS {
                continue;
            }
            let w = V(-direction.1, direction.0).scale(half_beam);
            let corners =
                [a.add(w), e.add(w), e.sub(w), a.sub(w)].map(|p| V(p.dot(u), sign * p.dot(v)));
            if let Some(stop) = facade::blocked_span(corners, inner, outer) {
                if stop.0 <= facade.high + EPS && stop.1 >= facade.low - EPS {
                    result.stops.push(stop);
                }
            }
            continue;
        }

        if !seeds_course {
            continue;
        }
        let begin = a.dot(u).min(e.dot(u)).max(0.0);
        let end = a.dot(u).max(e.dot(u)).min(len);
        let transverse = a.dot(v);
        if end > begin + EPS
            && transverse.abs() < outer + half_beam - EPS
            && sign * transverse + half_beam >= inner - gap - EPS
        {
            result.regions.push(Region {
                low: begin,
                high: end,
                beam: beam.guid.clone(),
                end_cover: false,
            });
        }
    }
    result.stops.sort_by(|a, b| a.0.total_cmp(&b.0));
    result
}

struct FacingSpan {
    low: f64,
    high: f64,
    // Hard bounds on a whole-side block cut, in run coordinates.
    stops: [Option<f64>; 2],
}

fn facing_spans(low: f64, high: f64, stops: &[(f64, f64)]) -> Vec<FacingSpan> {
    let mut result = Vec::new();
    let mut from = low;
    let mut left = None;
    for &(a, b) in stops {
        if b < from - EPS {
            continue;
        }
        if a > high + EPS {
            break;
        }
        if a > from + EPS {
            result.push(FacingSpan {
                low: from,
                high: a.min(high),
                stops: [left, Some(a)],
            });
        }
        from = from.max(b);
        left = Some(from);
    }
    if high > from + EPS {
        result.push(FacingSpan {
            low: from,
            high,
            stops: [left, None],
        });
    }
    result
}
fn clean(v: f64) -> f64 {
    if v.abs() < EPS {
        0.0
    } else {
        v
    }
}

/// At a corner the return ends against the inner face of the longitudinal skin.
fn fit_end_cover(mut cover: Lamella, longitudinal: &[Lamella]) -> Option<Lamella> {
    let xy = |p: [f64; 3]| V(p[0], p[1]);
    let u = xy(cover.placement.x_axis);
    let v = xy(cover.placement.y_axis);
    let origin = xy(cover.placement.center_mm);
    let half = cover.length_mm / 2.0;
    let (mut from, mut to) = (-half, half);
    for side in longitudinal {
        if cover.beam_ids.is_disjoint(&side.beam_ids)
            || u.dot(xy(side.placement.x_axis)).abs() > EPS
        {
            continue;
        }
        let delta = xy(side.placement.center_mm).sub(origin);
        if delta.dot(v).abs() >= (side.length_mm + cover.thickness_mm) / 2.0 - EPS {
            continue;
        }
        let a = delta.dot(u) - side.thickness_mm / 2.0;
        let b = delta.dot(u) + side.thickness_mm / 2.0;
        if a <= from + EPS && b > from + EPS {
            from = b;
        } else if b >= to - EPS && a < to - EPS {
            to = a;
        }
    }
    if to <= from + EPS {
        return None;
    }
    let shift_point = |p: &mut [f64; 3], mm| {
        let next = xy(*p).add(u.scale(mm));
        p[0] = next.0;
        p[1] = next.1;
    };
    shift_point(&mut cover.placement.center_mm, (from + to) / 2.0);
    let grid_start = xy(cover.grid_start_mm).sub(origin).dot(u);
    let grid_end = xy(cover.grid_end_mm).sub(origin).dot(u);
    shift_point(
        &mut cover.grid_start_mm,
        grid_start.clamp(from, to) - grid_start,
    );
    shift_point(&mut cover.grid_end_mm, grid_end.clamp(from, to) - grid_end);
    cover.length_mm = to - from;
    Some(cover)
}

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

fn course_seams(
    run: &WallRun,
    course: &crate::domain::Course,
    profile: &Profile,
    blocks: &[Block],
    saved: Option<&[SavedBlock]>,
) -> Option<Vec<f64>> {
    let (o, u, v, len) = frame(run);
    if let Some(saved) = saved {
        let bottom = course.z as f64 / 100.0;
        let top = bottom + profile.index_centimm as f64 / 100.0;
        return crate::lamella_saved::seams(
            saved,
            run,
            bottom,
            top,
            course.index,
            profile,
            0.0,
            len,
        );
    }
    let mut seams = Vec::new();
    if let Some(residue) = crate::layout::ordinary_residue(run, course.index, profile) {
        let residue = residue as f64 / 100.0;
        let module = profile.ordinary_length_centimm as f64 / 100.0;
        let mut seam = residue - (residue / module).ceil() * module;
        while seam < len + module - EPS {
            seams.push(seam);
            seam += module;
        }
        return Some(seams);
    }

    // A custom profile uses the selected physical joints.
    for block in blocks.iter().filter(|b| b.course_index == course.index) {
        for (a, b) in std::iter::once((block.start, block.end))
            .chain(block.arms.iter().map(|arm| (arm.start, arm.end)))
        {
            let a = point(a).sub(o);
            let b = point(b).sub(o);
            if a.dot(v).abs() < EPS && b.dot(v).abs() < EPS {
                seams.extend([a.dot(u), b.dot(u)]);
            }
        }
    }
    (!seams.is_empty()).then_some(seams)
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
    let course_height = profile.index_centimm as f64 / 100.0;
    crate::effective_geometry::normalize_openings(&mut effective, course_height);
    let building = crate::topology::normalize_building(effective.raw_building())
        .map_err(|e| ApiFailure::new("LAMELLA_TOPOLOGY_FAILED", e, None))?;
    let topology = crate::topology::build_topology(building)
        .map_err(|e| ApiFailure::new("LAMELLA_TOPOLOGY_FAILED", e, None))?;
    let widths = topology
        .walls
        .iter()
        .map(|w| (w.id.as_str(), w.thickness as f64 / 100.0))
        .collect();
    let mut out = Vec::new();
    let mut warnings = Vec::new();
    for course in &topology.courses {
        let bottom = course.z as f64 / 100.0;
        let course_start = out.len();
        let mut end_covers = Vec::new();
        for facade in facade::facades(course, &widths) {
            let run = &facade.run;
            let (o, u, v, len) = frame(run);
            let mut coverage = course_coverage(&effective, &facade, bottom, course_height);
            if coverage.regions.is_empty() {
                continue;
            }
            let available = facing_spans(facade.low, facade.high, &coverage.stops);
            if available.is_empty() {
                continue;
            }
            let side = facade.side.name();
            let Some(mut seams) = course_seams(run, course, profile, blocks, saved) else {
                let (code, message) = if saved.is_some() {
                    (
                        "LAMELLA_SAVED_GRID_UNAVAILABLE",
                        "Для ламелей нет сохранённой сетки блоков в этом венце; блоки не изменены",
                    )
                } else {
                    ("LAMELLA_GRID_UNAVAILABLE", "Ламели покрывают не всю область балки: для пользовательского профиля в этом венце нет выбранных швов сетки")
                };
                let mut warning =
                    ApiFailure::new(code, message, Some(run.sources[0].wall_id.clone()));
                warning.course_index = Some(course.index);
                warning.source_ids = coverage.regions.iter().map(|r| r.beam.clone()).collect();
                if !warnings.iter().any(|previous: &ApiFailure| {
                    previous.course_index == warning.course_index
                        && previous.source_id == warning.source_id
                        && previous.source_ids == warning.source_ids
                }) {
                    warnings.push(warning);
                }
                continue;
            };
            seams.sort_by(f64::total_cmp);
            seams.dedup_by(|a, b| (*a - *b).abs() < EPS);
            let snap_edges: Vec<_> = seams
                .iter()
                .copied()
                .chain(available.iter().flat_map(|span| [span.low, span.high]))
                .collect();
            for area in &mut coverage.regions {
                (area.low, area.high) = snap_interval(area.low, area.high, &snap_edges);
            }
            let offset = facade.side.sign() * (facade.half_width - THICKNESS / 2.0);
            for span in available {
                let cells =
                    std::iter::once(span.low)
                        .chain(seams.iter().copied().filter(|&s| {
                            s > span.low.max(0.0) + EPS && s < span.high.min(len) - EPS
                        }))
                        .chain(std::iter::once(span.high));
                let mut cells = cells.peekable();
                while let Some(a) = cells.next() {
                    let Some(&b) = cells.peek() else { break };
                    let selected = coverage
                        .regions
                        .iter()
                        .filter(|r| r.low < b - EPS && r.high > a + EPS);
                    let beam_ids: BTreeSet<_> = selected.clone().map(|r| r.beam.clone()).collect();
                    if beam_ids.is_empty() {
                        continue;
                    }
                    let mut wall_ids: BTreeSet<_> = run
                        .sources
                        .iter()
                        .filter(|s| {
                            s.start_offset as f64 / 100.0 < b - EPS
                                && s.end_offset as f64 / 100.0 > a + EPS
                        })
                        .map(|s| s.wall_id.clone())
                        .collect();
                    if wall_ids.is_empty() {
                        let source = if b <= EPS {
                            run.sources.first()
                        } else {
                            run.sources.last()
                        };
                        wall_ids.extend(source.map(|s| s.wall_id.clone()));
                    }
                    let source_ids = wall_ids.union(&beam_ids).cloned().collect();
                    let center = o.add(u.scale((a + b) / 2.0)).add(v.scale(offset));
                    let grid_point = |t| {
                        let V(x, y) = o.add(u.scale(t));
                        [x, y, bottom]
                    };
                    let lamella = Lamella {
                        id: format!(
                            "lamella:{}:{}:{}:{:016x}:{:016x}",
                            course.index,
                            run.id,
                            side,
                            clean(a).to_bits(),
                            clean(bottom).to_bits()
                        ),
                        placement: Placement {
                            center_mm: [
                                clean(center.0),
                                clean(center.1),
                                bottom + course_height / 2.0,
                            ],
                            x_axis: [clean(u.0), clean(u.1), 0.0],
                            y_axis: [clean(v.0), clean(v.1), 0.0],
                            z_axis: [0.0, 0.0, 1.0],
                        },
                        length_mm: b - a,
                        thickness_mm: THICKNESS,
                        height_mm: course_height,
                        course_index: course.index,
                        side,
                        wall_ids,
                        beam_ids,
                        source_ids,
                        grid_start_mm: grid_point(a.clamp(0.0, len)),
                        grid_end_mm: grid_point(b.clamp(0.0, len)),
                        recess_limits: span.stops.map(|stop| stop.map(|s| o.dot(u) + s)),
                        recess_block_ids: BTreeSet::new(),
                    };
                    if selected.into_iter().all(|r| r.end_cover) {
                        end_covers.push(lamella);
                    } else {
                        out.push(lamella);
                    }
                }
            }
        }
        let covers: Vec<_> = end_covers
            .into_iter()
            .filter_map(|cover| fit_end_cover(cover, &out[course_start..]))
            .collect();
        out.extend(covers);
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
    let lc = l.placement.center_mm;
    let ll = l.length_mm;
    let lh = l.height_mm;
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
                    let trim = json!({"kind":"lamella_recess","lamella_id":l.id,
                "placement":l.placement,"length_mm":l.length_mm,"thickness_mm":l.thickness_mm,"height_mm":l.height_mm});
                    trims.push(trim);
                }
            }
        }
    }
}

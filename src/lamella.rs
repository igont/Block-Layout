//! Physical facade strips use the same canonical run and course grid as blocks.
//! The finished outer face stays on the wall contour; all frames are centered.
use crate::api::{ApiFailure, LayoutRequest};
use crate::domain::WallRun;
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
    #[serde(skip_serializing_if = "BTreeSet::is_empty")]
    pub recess_block_ids: BTreeSet<String>,
}

pub struct LamellaPlan {
    pub lamellas: Vec<Lamella>,
    pub warnings: Vec<ApiFailure>,
}

// Beam intervals select complete facade cells; they never cut the product.
struct Region {
    low: f64,
    high: f64,
    beam: String,
    side: &'static str,
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

fn regions(
    request: &LayoutRequest,
    run: &WallRun,
    low: f64,
    high: f64,
    half_width: f64,
    bottom: f64,
    top: f64,
    z0: f64,
    course_height: f64,
) -> Vec<Region> {
    let (o, u, v, _) = frame(run);
    let mut result = Vec::new();
    for b in &request.beams {
        let g = &b.geometry;
        if (b.start_zmm - b.end_zmm).abs() > EPS
            || g.height_direction_x.abs() > EPS
            || g.height_direction_y.abs() > EPS
            || (g.height_direction_z.abs() - 1.0).abs() > EPS
        {
            continue;
        }
        let a = V(b.start_xmm, b.start_ymm).sub(o);
        let e = V(b.end_xmm, b.end_ymm).sub(o);
        let d = e.sub(a);
        let length = d.0.hypot(d.1);
        if length <= EPS || d.scale(1.0 / length).dot(u).abs() < 1.0 - EPS {
            continue;
        }
        let bz = b.start_zmm + g.height_mm * g.height_direction_z;
        let zlow = b.start_zmm.min(bz);
        let zhigh = b.start_zmm.max(bz);
        let zgrid = [
            z0 + ((zlow - z0) / course_height).round() * course_height,
            z0 + ((zhigh - z0) / course_height).round() * course_height,
        ];
        let (zlow, zhigh) = snap_interval(zlow, zhigh, &zgrid);
        if zlow >= top - EPS || zhigh <= bottom + EPS {
            continue;
        }
        let begin = a.dot(u).min(e.dot(u)).max(low);
        let end = a.dot(u).max(e.dot(u)).min(high);
        let transverse = a.dot(v);
        if end <= begin + EPS || transverse.abs() >= half_width + g.width_mm / 2.0 - EPS {
            continue;
        }
        let gap = (half_width - g.width_mm / 2.0 - THICKNESS).max(0.0);
        let inner = half_width - THICKNESS - gap;
        for (side, sign) in [("left", 1.0), ("right", -1.0)] {
            if sign * transverse + g.width_mm / 2.0 < inner - EPS {
                continue;
            }
            result.push(Region {
                low: begin,
                high: end,
                beam: b.guid.clone(),
                side,
            });
        }
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
                let same_section = run.sources.iter().all(|s| {
                    effective
                        .wall_volumes
                        .iter()
                        .find(|w| w.guid == s.wall_id)
                        .is_some_and(|w| (w.thickness_mm - wall.thickness_mm).abs() < EPS)
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
                let bottom = course_bottom;
                let top = course_top;
                let mut areas = regions(
                    &effective,
                    run,
                    low,
                    high,
                    wall.thickness_mm / 2.0,
                    bottom,
                    top,
                    effective.z0_mm,
                    profile.index_centimm as f64 / 100.0,
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
                for area in &mut areas {
                    (area.low, area.high) = snap_interval(area.low, area.high, &seams);
                }
                for (side, sign) in [("left", 1.0), ("right", -1.0)] {
                    let offset = wall.thickness_mm / 2.0 - THICKNESS / 2.0;
                    for pair in seams.windows(2) {
                        // Only terminal cells reach the physical end beyond the wall axis.
                        let a = if pair[0] <= low + EPS {
                            low - wall.thickness_mm / 2.0
                        } else {
                            pair[0]
                        };
                        let b = if pair[1] >= high - EPS {
                            high + wall.thickness_mm / 2.0
                        } else {
                            pair[1]
                        };
                        if b <= a + EPS {
                            continue;
                        }
                        {
                            let z = [bottom, top];
                            let selected: Vec<_> = areas
                                .iter()
                                .filter(|r| {
                                    r.side == side
                                        && r.low < pair[1] - EPS
                                        && r.high > pair[0] + EPS
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
                            let lamella = Lamella {
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
                            };
                            out.push(lamella);
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

//! Канонизация осей и построение графа венцов.

use std::collections::{BTreeMap, BTreeSet};

use crate::domain::{
    Course, CourseEdge, CourseVertex, Edge, NormalizedBuilding, NormalizedWall, Point, RawBuilding,
    RawWall, Ray, RunSource, Topology, Vertex, VerticalLink, WallRun,
};

const LIMIT: i64 = 100_000_000;
const COURSE_STEP: i64 = 6_300;
const MAX_WALLS: usize = 10_000;
const MAX_COURSES: i64 = 20_000;

fn scaled(value: f64, name: &str) -> Result<i64, String> {
    if !value.is_finite() || value.abs() > LIMIT as f64 / 100.0 {
        return Err(format!("{name}: координата вне допустимого диапазона"));
    }
    let result = crate::precision::centimm(value) as i64;
    if result.abs() > LIMIT {
        return Err(format!("{name}: координата вне допустимого диапазона"));
    }
    Ok(result)
}

fn point(raw: crate::domain::RawPoint) -> Result<Point, String> {
    Ok(Point {
        x: scaled(raw.x_mm, "x")?,
        y: scaled(raw.y_mm, "y")?,
    })
}

fn direction(a: Point, b: Point) -> Result<u16, String> {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    if dx == 0 && dy == 0 {
        return Err("нулевая ось".into());
    }
    if dx != 0 && dy != 0 && dx.abs() != dy.abs() {
        return Err("неподдерживаемый угол оси".into());
    }
    Ok(match (dx.signum(), dy.signum()) {
        (1, 0) => 0,
        (1, 1) => 45,
        (0, 1) => 90,
        (-1, 1) => 135,
        (-1, 0) => 180,
        (-1, -1) => 225,
        (0, -1) => 270,
        (1, -1) => 315,
        _ => unreachable!(),
    })
}

fn normalize_wall(raw: RawWall) -> Result<NormalizedWall, String> {
    if raw.id.trim().is_empty() {
        return Err("пустой идентификатор стены".into());
    }
    let mut wall = NormalizedWall {
        id: raw.id,
        start: point(raw.start)?,
        end: point(raw.end)?,
        bottom_start: scaled(raw.bottom_start_mm, "bottom_start")?,
        top_start: scaled(raw.top_start_mm, "top_start")?,
        bottom_end: scaled(raw.bottom_end_mm, "bottom_end")?,
        top_end: scaled(raw.top_end_mm, "top_end")?,
        thickness: scaled(raw.thickness_mm, "thickness")?,
    };
    direction(wall.start, wall.end).map_err(|e| format!("{}: {e}", wall.id))?;
    if wall.thickness <= 0 || wall.bottom_start >= wall.top_start || wall.bottom_end >= wall.top_end
    {
        return Err(format!(
            "{}: несовместимая толщина или высотный профиль",
            wall.id
        ));
    }
    if wall.end < wall.start {
        std::mem::swap(&mut wall.start, &mut wall.end);
        std::mem::swap(&mut wall.bottom_start, &mut wall.bottom_end);
        std::mem::swap(&mut wall.top_start, &mut wall.top_end);
    }
    Ok(wall)
}

pub fn normalize_building(raw: RawBuilding) -> Result<NormalizedBuilding, String> {
    if raw.walls.len() > MAX_WALLS {
        return Err("слишком много стен".into());
    }
    let z0 = scaled(raw.z0_mm, "z0")?;
    // Coordinate transforms can leave floating-point noise even at zero.
    // Bound it by a few f64 roundoffs at the scale of the source geometry,
    // rather than treating the entire 0.01 mm rounding cell as one point.
    let coordinate_scale = raw
        .walls
        .iter()
        .flat_map(|wall| {
            [
                wall.start.x_mm,
                wall.start.y_mm,
                wall.end.x_mm,
                wall.end.y_mm,
            ]
        })
        .map(f64::abs)
        .fold(1.0, f64::max);
    let roundoff_mm = 8.0 * f64::EPSILON * coordinate_scale;
    let mut original_points: BTreeMap<Point, crate::domain::RawPoint> = BTreeMap::new();
    for wall in &raw.walls {
        for original in [wall.start, wall.end] {
            let normalized = point(original)?;
            if let Some(previous) = original_points.get(&normalized) {
                if (previous.x_mm - original.x_mm).abs() > roundoff_mm
                    || (previous.y_mm - original.y_mm).abs() > roundoff_mm
                {
                    return Err(format!(
                        "разные точки схлопнулись при округлении: {}, {}",
                        normalized.x, normalized.y
                    ));
                }
            } else {
                original_points.insert(normalized, original);
            }
        }
    }
    let mut walls = raw
        .walls
        .into_iter()
        .map(normalize_wall)
        .collect::<Result<Vec<_>, _>>()?;
    walls.sort_by(|a, b| a.id.cmp(&b.id));
    for pair in walls.windows(2) {
        if pair[0].id == pair[1].id {
            return Err(format!("повторный id стены: {}", pair[0].id));
        }
    }
    Ok(NormalizedBuilding { z0, walls })
}

fn cross(ax: i128, ay: i128, bx: i128, by: i128) -> i128 {
    ax * by - ay * bx
}
fn orient(a: Point, b: Point, c: Point) -> i128 {
    cross(
        (b.x - a.x) as i128,
        (b.y - a.y) as i128,
        (c.x - a.x) as i128,
        (c.y - a.y) as i128,
    )
}
fn on_segment(a: Point, b: Point, p: Point) -> bool {
    orient(a, b, p) == 0
        && p.x >= a.x.min(b.x)
        && p.x <= a.x.max(b.x)
        && p.y >= a.y.min(b.y)
        && p.y <= a.y.max(b.y)
}
fn round_ratio(mut numerator: i128, mut denominator: i128) -> i128 {
    if denominator < 0 {
        numerator = -numerator;
        denominator = -denominator;
    }
    let q = numerator.div_euclid(denominator);
    let r = numerator.rem_euclid(denominator);
    if r * 2 > denominator || (r * 2 == denominator && q % 2 != 0) {
        q + 1
    } else {
        q
    }
}

fn crossing(a: &NormalizedWall, b: &NormalizedWall) -> Result<Option<Point>, String> {
    let p = a.start;
    let q = b.start;
    let rx = (a.end.x - p.x) as i128;
    let ry = (a.end.y - p.y) as i128;
    let sx = (b.end.x - q.x) as i128;
    let sy = (b.end.y - q.y) as i128;
    let den = cross(rx, ry, sx, sy);
    if den == 0 {
        if orient(p, a.end, q) != 0 {
            return Ok(None);
        }
        let common = [a.start, a.end, b.start, b.end]
            .into_iter()
            .filter(|v| on_segment(a.start, a.end, *v) && on_segment(b.start, b.end, *v))
            .collect::<BTreeSet<_>>();
        if common.len() > 1 {
            return Err(format!(
                "{} и {}: неоднозначное коллинеарное наложение",
                a.id, b.id
            ));
        }
        return Ok(common.into_iter().next());
    }
    let qx = (q.x - p.x) as i128;
    let qy = (q.y - p.y) as i128;
    let t = cross(qx, qy, sx, sy);
    let u = cross(qx, qy, rx, ry);
    let inside = |n: i128| {
        if den > 0 {
            n >= 0 && n <= den
        } else {
            n <= 0 && n >= den
        }
    };
    if !inside(t) || !inside(u) {
        return Ok(None);
    }
    let x = round_ratio(p.x as i128 * den + rx * t, den);
    let y = round_ratio(p.y as i128 * den + ry * t, den);
    let candidate = Point {
        x: x as i64,
        y: y as i64,
    };
    if !on_segment(a.start, a.end, candidate) || !on_segment(b.start, b.end, candidate) {
        return Err(format!(
            "{} и {}: пересечение неоднозначно после округления",
            a.id, b.id
        ));
    }
    Ok(Some(candidate))
}

fn axis_len(w: &NormalizedWall) -> i64 {
    (w.end.x - w.start.x).abs().max((w.end.y - w.start.y).abs())
}
fn position(w: &NormalizedWall, p: Point) -> i64 {
    (p.x - w.start.x).abs().max((p.y - w.start.y).abs())
}
fn at(w: &NormalizedWall, t: i64) -> Point {
    Point {
        x: w.start.x + (w.end.x - w.start.x).signum() * t,
        y: w.start.y + (w.end.y - w.start.y).signum() * t,
    }
}
fn vertex_id(p: Point) -> String {
    format!("v:{}:{}", p.x, p.y)
}

fn profile_le(w: &NormalizedWall, t: i64, z: i64, bottom: bool) -> bool {
    let length = axis_len(w) as i128;
    let (a, b) = if bottom {
        (w.bottom_start, w.bottom_end)
    } else {
        (w.top_start, w.top_end)
    };
    let value = a as i128 * length + (b - a) as i128 * t as i128;
    if bottom {
        value <= z as i128 * length
    } else {
        value > z as i128 * length
    }
}

fn allowed_range(w: &NormalizedWall, z: i64, bottom: bool) -> Option<(i64, i64)> {
    let len = axis_len(w);
    let valid = |t| profile_le(w, t, z, bottom);
    let first = valid(0);
    let last = valid(len);
    match (first, last) {
        (true, true) => Some((0, len)),
        (false, false) => None,
        (false, true) => {
            let (mut lo, mut hi) = (0, len);
            while lo + 1 < hi {
                let mid = lo + (hi - lo) / 2;
                if valid(mid) {
                    hi = mid
                } else {
                    lo = mid
                }
            }
            Some((hi, len))
        }
        (true, false) => {
            let (mut lo, mut hi) = (0, len);
            while lo + 1 < hi {
                let mid = lo + (hi - lo) / 2;
                if valid(mid) {
                    lo = mid
                } else {
                    hi = mid
                }
            }
            Some((0, lo))
        }
    }
}

fn wall_interval(w: &NormalizedWall, z: i64) -> Option<(i64, i64)> {
    let (a, b) = allowed_range(w, z, true)?;
    let (c, d) = allowed_range(w, z, false)?;
    let lo = a.max(c);
    let hi = b.min(d);
    (lo < hi).then_some((lo, hi))
}

fn floor_div(a: i64, b: i64) -> i64 {
    a.div_euclid(b)
}
fn ceil_div(a: i64, b: i64) -> i64 {
    -(-a).div_euclid(b)
}

fn integer_sqrt(value: u128) -> u128 {
    let (mut lo, mut hi) = (0_u128, 400_000_001_u128);
    while lo + 1 < hi {
        let mid = lo + (hi - lo) / 2;
        if mid * mid <= value {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

fn metric_distance(a: Point, b: Point) -> i64 {
    let dx = a.x.abs_diff(b.x) as u128;
    let dy = a.y.abs_diff(b.y) as u128;
    let squared = dx * dx + dy * dy;
    let floor = integer_sqrt(squared);
    let rounded = if 4 * squared >= (2 * floor + 1).pow(2) {
        floor + 1
    } else {
        floor
    };
    rounded as i64
}

fn run_root(parent: &mut [usize], index: usize) -> usize {
    let mut cursor = index;
    while parent[cursor] != cursor {
        let next = parent[cursor];
        parent[cursor] = parent[next];
        cursor = next;
    }
    cursor
}

fn wall_runs(edges: &[CourseEdge], walls_by_id: &BTreeMap<&str, &NormalizedWall>) -> Vec<WallRun> {
    let mut parent: Vec<usize> = (0..edges.len()).collect();
    let mut incidence: BTreeMap<Point, Vec<(usize, (i128, i128))>> = BTreeMap::new();
    for (index, edge) in edges.iter().enumerate() {
        let dx = (edge.end.x - edge.start.x) as i128;
        let dy = (edge.end.y - edge.start.y) as i128;
        incidence
            .entry(edge.start)
            .or_default()
            .push((index, (dx, dy)));
        incidence
            .entry(edge.end)
            .or_default()
            .push((index, (-dx, -dy)));
    }
    for arms in incidence.values() {
        if arms.len() != 2 {
            continue;
        }
        let a = arms[0].1;
        let b = arms[1].1;
        let first_wall = walls_by_id[edges[arms[0].0].wall_id.as_str()];
        let second_wall = walls_by_id[edges[arms[1].0].wall_id.as_str()];
        if first_wall.thickness == second_wall.thickness
            && cross(a.0, a.1, b.0, b.1) == 0
            && a.0 * b.0 + a.1 * b.1 < 0
        {
            let first = run_root(&mut parent, arms[0].0);
            let second = run_root(&mut parent, arms[1].0);
            parent[second] = first;
        }
    }
    let mut groups: BTreeMap<usize, Vec<&CourseEdge>> = BTreeMap::new();
    for (index, edge) in edges.iter().enumerate() {
        groups
            .entry(run_root(&mut parent, index))
            .or_default()
            .push(edge);
    }
    let mut runs = Vec::new();
    for group in groups.into_values() {
        let start = group.iter().map(|edge| edge.start).min().unwrap();
        let end = group.iter().map(|edge| edge.end).max().unwrap();
        let mut sources = group
            .into_iter()
            .map(|edge| RunSource {
                wall_id: edge.wall_id.clone(),
                edge_id: edge.edge_id.clone(),
                start: edge.start,
                end: edge.end,
                start_offset: metric_distance(start, edge.start),
                end_offset: metric_distance(start, edge.end),
            })
            .collect::<Vec<_>>();
        sources.sort_by(|a, b| {
            (a.start_offset, &a.wall_id, &a.edge_id).cmp(&(b.start_offset, &b.wall_id, &b.edge_id))
        });
        runs.push(WallRun {
            id: format!("run:{}:{}:{}:{}", start.x, start.y, end.x, end.y),
            start,
            end,
            length: metric_distance(start, end),
            sources,
        });
    }
    runs.sort_by(|a, b| (&a.start, &a.end, &a.id).cmp(&(&b.start, &b.end, &b.id)));
    runs
}

fn course_vertices(runs: &[WallRun]) -> Vec<CourseVertex> {
    let mut map: BTreeMap<Point, Vec<Ray>> = BTreeMap::new();
    for e in runs {
        let forward = direction(e.start, e.end).unwrap_or(0);
        map.entry(e.start).or_default().push(Ray {
            edge_id: e.id.clone(),
            direction_deg: forward,
        });
        map.entry(e.end).or_default().push(Ray {
            edge_id: e.id.clone(),
            direction_deg: (forward + 180) % 360,
        });
    }
    map.into_iter()
        .map(|(point, mut rays)| {
            rays.sort_by(|a, b| (a.direction_deg, &a.edge_id).cmp(&(b.direction_deg, &b.edge_id)));
            CourseVertex {
                id: vertex_id(point),
                point,
                rays,
            }
        })
        .collect()
}

pub fn build_topology(building: NormalizedBuilding) -> Result<Topology, String> {
    let mut splits: BTreeMap<String, BTreeSet<Point>> = building
        .walls
        .iter()
        .map(|w| (w.id.clone(), BTreeSet::from([w.start, w.end])))
        .collect();
    for (i, a) in building.walls.iter().enumerate() {
        for b in building.walls.iter().skip(i + 1) {
            if let Some(p) = crossing(a, b)? {
                splits
                    .get_mut(&a.id)
                    .ok_or("потерян источник стены")?
                    .insert(p);
                splits
                    .get_mut(&b.id)
                    .ok_or("потерян источник стены")?
                    .insert(p);
            }
        }
    }
    let mut vertex_points = BTreeSet::new();
    let mut edges = Vec::new();
    for w in &building.walls {
        let mut points = splits
            .remove(&w.id)
            .ok_or("потерян источник стены")?
            .into_iter()
            .collect::<Vec<_>>();
        points.sort_by_key(|p| position(w, *p));
        for (index, pair) in points.windows(2).enumerate() {
            if pair[0] == pair[1] {
                return Err(format!("{}: схлопнувшийся участок", w.id));
            }
            vertex_points.insert(pair[0]);
            vertex_points.insert(pair[1]);
            edges.push(Edge {
                id: format!("{}:{}", w.id, index),
                wall_id: w.id.clone(),
                start_vertex_id: vertex_id(pair[0]),
                end_vertex_id: vertex_id(pair[1]),
                start: pair[0],
                end: pair[1],
            });
        }
    }
    let vertices = vertex_points
        .into_iter()
        .map(|point| Vertex {
            id: vertex_id(point),
            point,
        })
        .collect();
    let walls_by_id: BTreeMap<_, _> = building.walls.iter().map(|w| (w.id.as_str(), w)).collect();
    let mut courses = Vec::new();
    if let (Some(min_bottom), Some(max_top)) = (
        building
            .walls
            .iter()
            .map(|w| w.bottom_start.min(w.bottom_end))
            .min(),
        building
            .walls
            .iter()
            .map(|w| w.top_start.max(w.top_end))
            .max(),
    ) {
        let first = ceil_div(min_bottom - building.z0, COURSE_STEP);
        let last = floor_div(max_top - 1 - building.z0, COURSE_STEP);
        if last - first + 1 > MAX_COURSES || first.abs() > 1_000_000 || last.abs() > 1_000_000 {
            return Err("число или индекс венцов превышает предел".into());
        }
        for index in first..=last {
            let z = building.z0 + index * COURSE_STEP;
            let mut course_edges = Vec::new();
            for w in &building.walls {
                let Some((lo, hi)) = wall_interval(w, z) else {
                    continue;
                };
                for edge in edges.iter().filter(|e| e.wall_id == w.id) {
                    let start = lo.max(position(w, edge.start));
                    let end = hi.min(position(w, edge.end));
                    if start < end {
                        course_edges.push(CourseEdge {
                            edge_id: edge.id.clone(),
                            wall_id: w.id.clone(),
                            start: at(w, start),
                            end: at(w, end),
                        });
                    }
                }
            }
            if !course_edges.is_empty() {
                let runs = wall_runs(&course_edges, &walls_by_id);
                let vertices = course_vertices(&runs);
                courses.push(Course {
                    index,
                    z,
                    edges: course_edges,
                    runs,
                    vertices,
                });
            }
        }
    }
    let mut vertical_links = Vec::new();
    for pair in courses.windows(2) {
        if pair[1].index != pair[0].index + 1 {
            continue;
        }
        for lower in &pair[0].edges {
            let w = walls_by_id[lower.wall_id.as_str()];
            for upper in pair[1].edges.iter().filter(|e| e.wall_id == lower.wall_id) {
                let lo = position(w, lower.start).max(position(w, upper.start));
                let hi = position(w, lower.end).min(position(w, upper.end));
                if lo < hi {
                    vertical_links.push(VerticalLink {
                        lower_index: pair[0].index,
                        upper_index: pair[1].index,
                        wall_id: w.id.clone(),
                        start: at(w, lo),
                        end: at(w, hi),
                    });
                }
            }
        }
    }
    Ok(Topology {
        z0: building.z0,
        walls: building.walls,
        vertices,
        edges,
        courses,
        vertical_links,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{RawPoint, RawWall};

    fn wall(id: &str, start: (f64, f64), end: (f64, f64), bottom: f64, top: f64) -> RawWall {
        RawWall {
            id: id.into(),
            start: RawPoint {
                x_mm: start.0,
                y_mm: start.1,
            },
            end: RawPoint {
                x_mm: end.0,
                y_mm: end.1,
            },
            bottom_start_mm: bottom,
            top_start_mm: top,
            bottom_end_mm: bottom,
            top_end_mm: top,
            thickness_mm: 200.0,
        }
    }
    fn topology(walls: Vec<RawWall>) -> Topology {
        build_topology(normalize_building(RawBuilding { z0_mm: 0.0, walls }).unwrap()).unwrap()
    }

    #[test]
    fn reversed_and_permuted_input_has_same_topology() {
        let a = wall("a", (0.0, 0.0), (640.0, 0.0), 0.0, 126.0);
        let b = wall("b", (320.0, -100.0), (320.0, 100.0), 0.0, 126.0);
        let mut ar = a.clone();
        ar.start = a.end;
        ar.end = a.start;
        let mut br = b.clone();
        br.start = b.end;
        br.end = b.start;
        assert_eq!(topology(vec![a, b]), topology(vec![br, ar]));
    }

    #[test]
    fn second_floor_uses_global_course_index() {
        let t = topology(vec![wall(
            "upper",
            (0.0, 0.0),
            (640.0, 0.0),
            2520.0,
            2646.0,
        )]);
        assert_eq!(
            t.courses.iter().map(|c| c.index).collect::<Vec<_>>(),
            vec![40, 41]
        );
        assert_eq!(t.courses[0].z, 252_000);
        assert_eq!(t.vertical_links.len(), 1);
    }

    #[test]
    fn diagonal_crossing_is_split_once() {
        let t = topology(vec![
            wall("diag", (0.0, 0.0), (100.0, 100.0), 0.0, 63.0),
            wall("straight", (50.0, -50.0), (50.0, 150.0), 0.0, 63.0),
        ]);
        assert_eq!(t.edges.len(), 4);
        let joint = t.courses[0]
            .vertices
            .iter()
            .find(|v| v.point == Point { x: 5000, y: 5000 })
            .unwrap();
        assert_eq!(
            joint
                .rays
                .iter()
                .map(|r| r.direction_deg)
                .collect::<Vec<_>>(),
            vec![45, 90, 225, 270]
        );
    }

    #[test]
    fn rounding_uses_nearest_even_hundredth_mm() {
        let t = topology(vec![wall("round", (0.005, 0.0), (1.015, 0.0), 0.0, 63.0)]);
        assert_eq!(t.walls[0].start.x, 0);
        assert_eq!(t.walls[0].end.x, 102);
    }

    #[test]
    fn overlapping_collinear_walls_are_rejected() {
        let n = normalize_building(RawBuilding {
            z0_mm: 0.0,
            walls: vec![
                wall("a", (0.0, 0.0), (100.0, 0.0), 0.0, 63.0),
                wall("b", (50.0, 0.0), (150.0, 0.0), 0.0, 63.0),
            ],
        })
        .unwrap();
        assert!(build_topology(n).unwrap_err().contains("наложение"));
    }

    #[test]
    fn sloped_top_shortens_higher_course() {
        let mut w = wall("slope", (0.0, 0.0), (126.0, 0.0), 0.0, 126.0);
        w.top_end_mm = 63.0;
        let t = topology(vec![w]);
        assert_eq!(t.courses.len(), 2);
        assert_eq!(t.courses[0].edges[0].end.x, 12_600);
        assert!(t.courses[1].edges[0].end.x < 12_600);
    }

    #[test]
    fn rounded_collision_between_distinct_sources_is_rejected() {
        let raw = RawBuilding {
            z0_mm: 0.0,
            walls: vec![
                wall("a", (0.0, 0.0), (100.0, 0.0), 0.0, 63.0),
                wall("b", (0.004, 0.0), (0.004, 100.0), 0.0, 63.0),
            ],
        };
        assert!(normalize_building(raw).unwrap_err().contains("схлопнулись"));
    }

    #[test]
    fn shared_endpoint_with_floating_point_noise_has_same_topology() {
        let bottom = wall("bottom", (0.0, 0.0), (9920.0, 0.0), 0.0, 63.0);
        let right = wall("right", (9920.0, 0.0), (9920.0, 5120.0), 0.0, 63.0);
        let expected = topology(vec![bottom.clone(), right.clone()]);
        let mut noisy = right;
        // Actual endpoint exported for Beresta 44 after coordinate transforms.
        noisy.start.y_mm = -1.7763568394002505e-12;
        assert_eq!(topology(vec![bottom.clone(), noisy.clone()]), expected);
        assert_eq!(topology(vec![noisy, bottom]), expected);
    }

    #[test]
    fn small_distinct_points_at_large_coordinates_are_rejected() {
        let raw = RawBuilding {
            z0_mm: 0.0,
            walls: vec![
                wall("bottom", (0.0, 0.0), (9920.0, 0.0), 0.0, 63.0),
                wall("right", (9920.0, 0.000001), (9920.0, 5120.0), 0.0, 63.0),
            ],
        };
        assert!(normalize_building(raw).unwrap_err().contains("схлопнулись"));
    }

    #[test]
    fn arbitrary_angle_is_rejected() {
        let raw = RawBuilding {
            z0_mm: 0.0,
            walls: vec![wall("bad", (0.0, 0.0), (100.0, 50.0), 0.0, 63.0)],
        };
        assert!(normalize_building(raw).unwrap_err().contains("угол"));
    }

    #[test]
    fn inactive_upper_cross_wall_does_not_split_lower_run() {
        let t = topology(vec![
            wall("main", (0.0, 0.0), (640.0, 0.0), 0.0, 2_646.0),
            wall("upper", (320.0, 0.0), (320.0, 320.0), 2_520.0, 2_646.0),
        ]);
        let lower = t.courses.iter().find(|course| course.index == 0).unwrap();
        assert_eq!(lower.edges.len(), 2);
        assert_eq!(lower.runs.len(), 1);
        assert_eq!(lower.runs[0].length, 64_000);
        assert_eq!(lower.runs[0].sources.len(), 2);
        assert_eq!(lower.vertices.len(), 2);
        assert!(lower
            .vertices
            .iter()
            .all(|v| v.point != Point { x: 32_000, y: 0 }));
        let upper = t.courses.iter().find(|course| course.index == 40).unwrap();
        assert_eq!(upper.runs.len(), 3);
        assert!(upper
            .vertices
            .iter()
            .any(|v| v.point == Point { x: 32_000, y: 0 } && v.rays.len() == 3));
    }

    #[test]
    fn collinear_walls_with_different_guids_keep_both_sources() {
        let t = topology(vec![
            wall("b", (320.0, 0.0), (640.0, 0.0), 0.0, 63.0),
            wall("a", (0.0, 0.0), (320.0, 0.0), 0.0, 63.0),
        ]);
        let run = &t.courses[0].runs[0];
        assert_eq!(t.courses[0].runs.len(), 1);
        assert_eq!(run.length, 64_000);
        assert_eq!(
            run.sources
                .iter()
                .map(|s| s.wall_id.as_str())
                .collect::<Vec<_>>(),
            vec!["a", "b"]
        );
        assert_eq!(
            run.sources
                .iter()
                .map(|s| (s.start_offset, s.end_offset))
                .collect::<Vec<_>>(),
            vec![(0, 32_000), (32_000, 64_000)]
        );
    }

    #[test]
    fn diagonal_offsets_are_measured_from_run_start_once() {
        let t = topology(vec![
            wall("first", (0.0, 0.0), (100.0, 100.0), 0.0, 63.0),
            wall("second", (100.0, 100.0), (200.0, 200.0), 0.0, 63.0),
        ]);
        let run = &t.courses[0].runs[0];
        assert_eq!(run.length, 28_284);
        assert_eq!(run.sources[0].end_offset, 14_142);
        assert_eq!(run.sources[1].start_offset, 14_142);
    }

    #[test]
    fn thickness_transition_remains_a_visible_vertex() {
        let first = wall("thin", (0.0, 0.0), (320.0, 0.0), 0.0, 63.0);
        let mut second = wall("thick", (320.0, 0.0), (640.0, 0.0), 0.0, 63.0);
        second.thickness_mm = 300.0;
        let t = topology(vec![first, second]);
        let course = &t.courses[0];
        assert_eq!(course.runs.len(), 2);
        assert!(course
            .vertices
            .iter()
            .any(|v| v.point == Point { x: 32_000, y: 0 } && v.rays.len() == 2));
    }
}

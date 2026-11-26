use super::{frame, EPS, V};
use crate::domain::{Course, Point, WallRun};
use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy)]
pub(super) enum Side {
    Left,
    Right,
}

impl Side {
    pub(super) fn sign(self) -> f64 {
        match self {
            Self::Left => 1.0,
            Self::Right => -1.0,
        }
    }

    pub(super) fn name(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }
}

pub(super) struct Facade<'a> {
    pub run: Cow<'a, WallRun>,
    pub side: Side,
    pub half_width: f64,
    pub low: f64,
    pub high: f64,
}

pub(super) fn facades<'a>(course: &'a Course, widths: &BTreeMap<&str, f64>) -> Vec<Facade<'a>> {
    let runs: BTreeMap<_, _> = course.runs.iter().map(|r| (r.id.as_str(), r)).collect();
    let vertices: BTreeMap<_, _> = course.vertices.iter().map(|v| (v.point, v)).collect();
    let half_width = |run: &WallRun| widths[run.sources[0].wall_id.as_str()] / 2.0;
    let neighbor = |run: &WallRun, endpoint: Point, turn_left: bool| {
        let rays = &vertices[&endpoint].rays;
        let i = rays.iter().position(|r| r.edge_id == run.id)?;
        if rays.len() == 1 {
            return None;
        }
        let step = if turn_left { 1 } else { rays.len() - 1 };
        let next = &rays[(i + step) % rays.len()];
        let angle = if turn_left {
            next.direction_deg + 360 - rays[i].direction_deg
        } else {
            rays[i].direction_deg + 360 - next.direction_deg
        } % 360;
        Some((runs[next.edge_id.as_str()], angle))
    };
    let retreat = |run: &WallRun, endpoint, turn_left| {
        let own = half_width(run);
        match neighbor(run, endpoint, turn_left) {
            None => -own,
            Some((_, 180)) => 0.0,
            Some((other, angle)) => {
                // Intersection of the two offset face lines, measured into this run.
                let (sin, cos) = f64::from(angle).to_radians().sin_cos();
                (half_width(other) + own * cos) / sin
            }
        }
    };

    let mut result = Vec::new();
    for side in [Side::Left, Side::Right] {
        let mut used = BTreeSet::new();
        let left = matches!(side, Side::Left);
        for first in &course.runs {
            if !used.insert(&first.id) {
                continue;
            }
            let width = half_width(first);
            let mut last = first;
            let mut run = Cow::Borrowed(first);
            // A Tee can split the axis into two block runs while its far face is continuous.
            while let Some((next, 180)) = neighbor(last, last.end, !left) {
                if next.start != last.end || (half_width(next) - width).abs() > EPS {
                    break;
                }
                used.insert(&next.id);
                let joined = run.to_mut();
                let (o, u, _, _) = frame(joined);
                joined
                    .sources
                    .extend(next.sources.iter().cloned().map(|mut source| {
                        let offset =
                            |p| (super::point(p).sub(o).dot(u) * 100.0).round_ties_even() as i64;
                        source.start_offset = offset(source.start);
                        source.end_offset = offset(source.end);
                        source
                    }));
                joined.end = next.end;
                last = next;
            }
            let (_, _, _, length) = frame(&run);
            if last.id != first.id {
                let joined = run.to_mut();
                joined.length = (length * 100.0).round_ties_even() as i64;
                let (a, b) = (joined.start, joined.end);
                joined.id = format!("run:{}:{}:{}:{}", a.x, a.y, b.x, b.y);
            }
            result.push(Facade {
                run,
                side,
                half_width: width,
                low: retreat(first, first.start, left),
                high: length - retreat(last, last.end, !left),
            });
        }
    }
    result
}

/// Along-wall projection of an oriented beam's footprint inside a facing strip.
pub(super) fn blocked_span(corners: [V; 4], inner: f64, outer: f64) -> Option<(f64, f64)> {
    if corners.iter().all(|p| p.1 <= inner + EPS) || corners.iter().all(|p| p.1 >= outer - EPS) {
        return None;
    }
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    let mut include = |x: f64| {
        low = low.min(x);
        high = high.max(x);
    };
    for i in 0..4 {
        let a = corners[i];
        let b = corners[(i + 1) % 4];
        if a.1 >= inner && a.1 <= outer {
            include(a.0);
        }
        for y in [inner, outer] {
            if a.1.min(b.1) < y && y < a.1.max(b.1) {
                include(a.0 + (b.0 - a.0) * (y - a.1) / (b.1 - a.1));
            }
        }
    }
    (high > low + EPS).then_some((low, high))
}

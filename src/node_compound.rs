//! Joint stock assembly for neighbouring nodes on one straight wall run.
//! The same assembly predicate is used before selection and before filling.
use crate::domain::Point;
use crate::end_state::EndStates;
use crate::layout::{Block, BlockArm};
use std::collections::{BTreeMap, BTreeSet};

fn axis(rotation: u16) -> Option<(i64, i64)> {
    match rotation % 360 {
        0 => Some((1, 0)),
        90 => Some((0, 1)),
        180 => Some((-1, 0)),
        270 => Some((0, -1)),
        _ => None,
    }
}
fn project(point: Point, origin: Point, direction: (i64, i64)) -> i64 {
    (point.x - origin.x) * direction.0 + (point.y - origin.y) * direction.1
}
fn on_axis(point: Point, origin: Point, direction: (i64, i64)) -> bool {
    (point.x - origin.x) * direction.1 == (point.y - origin.y) * direction.0
}

/// A shared product exists only when all covered arms form one continuous
/// axial stock, and every original manufacturing cut fits that stock.
pub(crate) fn compose(
    left: &Block,
    right: &Block,
    run_start: Point,
    run_end: Point,
    maximum_stock: i64,
) -> Option<Block> {
    let delta = (run_end.x - run_start.x, run_end.y - run_start.y);
    let direction = match delta {
        (x, 0) if x > 0 => (1, 0),
        (x, 0) if x < 0 => (-1, 0),
        (0, y) if y > 0 => (0, 1),
        (0, y) if y < 0 => (0, -1),
        _ => return None,
    };
    let run_length = delta.0.abs() + delta.1.abs();
    let rotation = match direction {
        (1, 0) => 0,
        (0, 1) => 90,
        (-1, 0) => 180,
        _ => 270,
    };
    let mut spans = Vec::new();
    for part in [left, right] {
        let part_axis = axis(part.rotation_deg)?;
        if part_axis.0 * direction.1 != part_axis.1 * direction.0 || part.arms.is_empty() {
            return None;
        }
        for arm in &part.arms {
            if !on_axis(arm.start, run_start, direction) || !on_axis(arm.end, run_start, direction)
            {
                return None;
            }
            let a = project(arm.start, run_start, direction);
            let b = project(arm.end, run_start, direction);
            spans.push((a.min(b), a.max(b)));
        }
    }
    spans.sort();
    let from = spans.first()?.0;
    let mut to = spans.first()?.1;
    for &(a, b) in &spans[1..] {
        if a > to {
            return None;
        }
        to = to.max(b);
    }
    let length = to - from;
    if from > 0 || to < run_length || length <= 0 || length > maximum_stock {
        return None;
    }
    let origin = Point {
        x: run_start.x + direction.0 * from,
        y: run_start.y + direction.1 * from,
    };
    let mut cuts: BTreeMap<(String, i64, u8), BTreeSet<String>> = BTreeMap::new();
    for part in [left, right] {
        let part_axis = axis(part.rotation_deg)?;
        let sign = part_axis.0 * direction.0 + part_axis.1 * direction.1;
        for cut in &part.cuts {
            if !cut.starts_with("Type") {
                continue;
            }
            let mut fields = cut.splitn(4, ':');
            let key = fields.next()?;
            let location = fields.next()?;
            let face = fields.next()?.strip_prefix('y')?.parse::<u8>().ok()?;
            let sources = fields.next()?;
            let local = if let Some(x) = location.strip_prefix('x') {
                (x.parse::<i64>().ok()? - 1) * 32_000
            } else {
                location.strip_prefix('p')?.parse::<i64>().ok()?
            };
            let position = project(part.start, origin, direction) + sign * local;
            if position < 0 || position > length || !(1..=3).contains(&face) {
                return None;
            }
            let face = if sign < 0 { 4 - face } else { face };
            cuts.entry((key.to_owned(), position, face))
                .or_default()
                .extend(sources.split(',').map(str::to_owned));
        }
    }
    if cuts.len() < 2 {
        return None;
    }
    let cuts = cuts
        .into_iter()
        .map(|((key, position, face), sources)| {
            format!(
                "{key}:p{position}:y{face}:{}",
                sources.into_iter().collect::<Vec<_>>().join(",")
            )
        })
        .collect();
    let mut result = left.clone();
    let mut ids = [left.id.clone(), right.id.clone()];
    ids.sort();
    result.id = format!("{}+{}", ids[0], ids[1]);
    result.kind = "node_compound".into();
    result.start = origin;
    result.end = origin;
    result.rotation_deg = rotation;
    result.local_origin = None;
    result.local_rotation_deg = None;
    result.length_centimm = length;
    result.catalog_nominal_centimm = Some(length);
    result.cuts = cuts;
    // Distal wall-boundary cuts at an internal join disappear with that join.
    // Actual opening/beam end cuts are reapplied by the finalisation owner.
    result.ends = EndStates::factory();
    result.source_ids.extend(right.source_ids.iter().cloned());
    result.source_ids.sort();
    result.source_ids.dedup();
    let mut arms = left.arms.clone();
    arms.extend(right.arms.iter().cloned());
    arms.sort_by_key(|a| {
        (
            a.edge_id.clone(),
            project(a.start, origin, direction).min(project(a.end, origin, direction)),
        )
    });
    let mut merged: Vec<BlockArm> = Vec::new();
    for arm in arms {
        let a = project(arm.start, origin, direction).min(project(arm.end, origin, direction));
        let b = project(arm.start, origin, direction).max(project(arm.end, origin, direction));
        if let Some(last) = merged.last_mut() {
            let previous = project(last.end, origin, direction);
            if last.edge_id == arm.edge_id && a <= previous {
                let end = previous.max(b);
                last.end = Point {
                    x: origin.x + direction.0 * end,
                    y: origin.y + direction.1 * end,
                };
                last.length_centimm = end - project(last.start, origin, direction);
                continue;
            }
        }
        let mut arm = arm;
        arm.start = Point {
            x: origin.x + direction.0 * a,
            y: origin.y + direction.1 * a,
        };
        arm.end = Point {
            x: origin.x + direction.0 * b,
            y: origin.y + direction.1 * b,
        };
        arm.length_centimm = b - a;
        merged.push(arm);
    }
    result.arms = merged;
    // Validate the real multi-cut stock before it becomes an allowed CSP pair.
    let shape = crate::node_shapes::node_stock(&result, 193.0, 63.0).ok()?;
    if shape.is_empty() {
        return None;
    }
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn product(id: &str, origin: i64, rotation: u16, from: i64, to: i64, cut: &str) -> Block {
        serde_json::from_value(json!({"id":id,"wall_id":"wall","edge_id":"shared",
            "course_index":0,"z_centimm":0,"start":{"x":origin,"y":0},"end":{"x":origin,"y":0},
            "length_centimm":64000,"kind":"node_T","product_key":"Type8","catalog_status":"KnownPattern",
            "rotation_deg":rotation,"local_origin":null,"local_rotation_deg":null,"is_bridge":false,
            "hide_spikes_left":false,"hide_spikes_right":false,"cuts":[cut],"source_ids":[id],
            "catalog_nominal_centimm":64000,"arms":[{"wall_id":"wall","edge_id":"shared",
                "start":{"x":from,"y":0},"end":{"x":to,"y":0},"length_centimm":(to-from).abs()}]})).unwrap()
    }

    #[test]
    fn shared_stock_rejects_gaps_oversize_and_a_lost_manufacturing_cut() {
        let left = product("left", 0, 0, 0, 32000, "Type8:x1:y3:left-run");
        let right = product("right", 64000, 180, 64000, 32000, "Type1:x1:y1:right-run");
        let from = Point { x: 0, y: 0 };
        let to = Point { x: 64000, y: 0 };
        let combined = compose(&left, &right, from, to, 64000).unwrap();
        assert_eq!(combined.arms.len(), 1);
        assert_eq!(combined.arms[0].length_centimm, 64000);
        assert!(compose(&left, &right, from, to, 32000).is_none());
        let mut gap = right.clone();
        gap.arms[0].end.x = 40000;
        assert!(compose(&left, &gap, from, to, 64000).is_none());
        let mut lost = right.clone();
        lost.cuts = vec!["Type1:p96000:y1:right-run".into()];
        assert!(compose(&left, &lost, from, to, 64000).is_none());
        let mut bent = right.clone();
        bent.arms[0].end.y = 100;
        assert!(compose(&left, &bent, from, to, 64000).is_none());
        assert_eq!(left.start.x, 0);
        assert_eq!(right.start.x, 64000);
        assert_eq!(left.cuts, vec!["Type8:x1:y3:left-run"]);
    }
}

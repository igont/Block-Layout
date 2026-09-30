//! Кандидат перемычек по венцам; компоненты сохраняют исходную физическую геометрию.
//! Источник слоёв: ForestBrick/WallAndOpeningProcessor.java:228, LintelProcessor.java:173.
use crate::api::{ApiFailure, LayoutRequest, Volume};
use crate::domain::Point;
use crate::layout::{Block, BlockArm, Profile};

const EPS: f64 = 0.01;
type P = [f64; 2];

fn point(p: Point) -> P {
    [p.x as f64 / 100., p.y as f64 / 100.]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn projection(p: P, origin: P, axis: P) -> f64 {
    dot([p[0] - origin[0], p[1] - origin[1]], axis)
}
fn collinear(p: P, origin: P, axis: P) -> bool {
    ((p[0] - origin[0]) * axis[1] - (p[1] - origin[1]) * axis[0]).abs() <= EPS
}

fn layer_offsets(opening: &Volume, width: f64) -> Option<&'static [i64]> {
    if (opening.start_top_zmm - opening.end_top_zmm).abs() > EPS {
        return None;
    }
    match opening.opening_type.as_str() {
        "CONSOLE" => Some(&[0, 2, 4, 6, 8]),
        "OPENING" if width <= 1000. + EPS => Some(&[0, 2, 4]),
        "OPENING" if width <= 1500. + EPS => Some(&[0, 2, 4, 6, 8]),
        _ => None,
    }
}

fn interval(block: &Block, origin: P, axis: P) -> Option<(f64, f64)> {
    let (s, c) = (block.rotation_deg as f64).to_radians().sin_cos();
    if (c * axis[1] - s * axis[0]).abs() > 1e-7 {
        return None;
    }
    let segments: Vec<_> = if block.arms.is_empty() {
        let start = point(block.start);
        vec![(
            start,
            [
                start[0] + c * block.length_centimm as f64 / 100.,
                start[1] + s * block.length_centimm as f64 / 100.,
            ],
        )]
    } else {
        block
            .arms
            .iter()
            .map(|a| (point(a.start), point(a.end)))
            .collect()
    };
    if segments
        .iter()
        .any(|&(a, b)| !collinear(a, origin, axis) || !collinear(b, origin, axis))
    {
        return None;
    }
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for (a, b) in segments {
        for p in [a, b] {
            let t = projection(p, origin, axis);
            lo = lo.min(t);
            hi = hi.max(t);
        }
    }
    (hi > lo + EPS).then_some((lo, hi))
}

fn support(intervals: &[(f64, f64)], edge: f64, left: bool) -> f64 {
    let mut intervals = intervals.to_vec();
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for (a, b) in intervals {
        if let Some(last) = merged.last_mut().filter(|v| a <= v.1 + EPS) {
            last.1 = last.1.max(b);
        } else {
            merged.push((a, b));
        }
    }
    merged
        .iter()
        .find(|&&(a, b)| a <= edge + EPS && b >= edge - EPS)
        .map_or(0., |&(a, b)| {
            if left {
                (edge - a).max(0.)
            } else {
                (b - edge).max(0.)
            }
        })
}

fn warning(code: &str, message: String, opening: &Volume, course: Option<i64>) -> ApiFailure {
    let mut result = ApiFailure::new(code, message, Some(opening.guid.clone()));
    result.source_ids.push(format!("opening:{}", opening.guid));
    result.course_index = course;
    result
}

/// Объединяет целые коллинеарные детали; физические тела остаются в components.
pub fn assemble_lintel_layers(
    request: &LayoutRequest,
    mut blocks: Vec<Block>,
    profile: &Profile,
) -> (Vec<Block>, Vec<ApiFailure>) {
    let mut diagnostics = Vec::new();
    let mut derived = 0;
    for opening in &request.opening_volumes {
        let origin = [opening.start_xmm, opening.start_ymm];
        let delta = [opening.end_xmm - origin[0], opening.end_ymm - origin[1]];
        let width = dot(delta, delta).sqrt();
        if width <= EPS {
            continue;
        }
        let Some(offsets) = layer_offsets(opening, width) else {
            diagnostics.push(warning("LINTEL_DESIGN_REQUIRED",
                format!("Проём шириной {width:.2} мм либо с наклонным верхом требует проекта перемычки; существующая геометрия сохранена"),opening,None));
            continue;
        };
        let axis = [delta[0] / width, delta[1] / width];
        let base = ((opening.start_top_zmm - request.z0_mm) / 63. - 1e-9).ceil() as i64;
        let required = profile.lintel_support_mm.max(200.);
        for (layer, &offset) in offsets.iter().enumerate() {
            let course = base + offset;
            let selected: Vec<_> = blocks
                .iter()
                .enumerate()
                .filter_map(|(i, b)| {
                    if b.course_index != course {
                        return None;
                    }
                    let span = interval(b, origin, axis)?;
                    (span.1 > -required + EPS && span.0 < width + required - EPS)
                        .then_some((i, span))
                })
                .collect();
            let spans: Vec<_> = selected.iter().map(|v| v.1).collect();
            let (left, right) = (support(&spans, 0., true), support(&spans, width, false));
            if left + EPS < required || right + EPS < required {
                diagnostics.push(warning("LINTEL_SUPPORT_INCOMPLETE",format!(
                    "Венец {course}: доступное опирание слева {left:.2} мм, справа {right:.2} мм; требуется {required:.2} мм; физическая геометрия сохранена"),opening,Some(course)));
            }
            if selected.is_empty() {
                continue;
            }
            let lo = spans.iter().map(|s| s.0).fold(f64::INFINITY, f64::min);
            let hi = spans.iter().map(|s| s.1).fold(f64::NEG_INFINITY, f64::max);
            let length = ((hi - lo) * 100.).round() as i64;
            let nominal = profile
                .bridge_nominal_lengths_centimm
                .iter()
                .copied()
                .filter(|&n| n >= length && n <= 275000)
                .min();
            let Some(nominal) = nominal else {
                diagnostics.push(warning("LINTEL_DESIGN_REQUIRED",format!(
                    "Венец {course}: полный пролёт деталей {:.2} мм не входит в каталог до 2750 мм; детали сохранены",hi-lo),opening,Some(course)));
                continue;
            };
            let indexes: std::collections::BTreeSet<_> = selected.iter().map(|s| s.0).collect();
            let components: Vec<_> = selected.iter().map(|&(i, _)| blocks[i].clone()).collect();
            let mut bridge = components[0].clone();
            bridge.id = format!("c{course}:lintel:{}:layer{layer}", opening.guid);
            bridge.kind = "bridge".into();
            bridge.is_bridge = true;
            bridge.product_key = None;
            bridge.catalog_status = "derived_lintel_product_required".into();
            bridge.catalog_nominal_centimm = Some(nominal);
            bridge.length_centimm = length;
            let at = |t: f64| Point {
                x: ((origin[0] + axis[0] * t) * 100.).round() as i64,
                y: ((origin[1] + axis[1] * t) * 100.).round() as i64,
            };
            bridge.start = at(lo);
            bridge.end = at(hi);
            bridge.rotation_deg =
                (axis[1].atan2(axis[0]).to_degrees().round() as i64).rem_euclid(360) as u16;
            bridge.local_origin = None;
            bridge.local_rotation_deg = None;
            bridge.arms = components
                .iter()
                .flat_map(|b| {
                    if b.arms.is_empty() {
                        vec![BlockArm {
                            wall_id: b.wall_id.clone(),
                            edge_id: b.edge_id.clone(),
                            start: b.start,
                            end: b.end,
                            length_centimm: b.length_centimm,
                        }]
                    } else {
                        b.arms.clone()
                    }
                })
                .collect();
            bridge.cuts = components.iter().flat_map(|b| b.cuts.clone()).collect();
            bridge.source_ids = components
                .iter()
                .flat_map(|b| b.source_ids.clone())
                .chain(
                    components
                        .iter()
                        .filter(|b| !b.wall_id.is_empty())
                        .map(|b| format!("wall:{}", b.wall_id)),
                )
                .chain([format!("opening:{}", opening.guid)])
                .collect();
            bridge.source_ids.sort();
            bridge.source_ids.dedup();
            bridge.components = components;
            blocks = blocks
                .into_iter()
                .enumerate()
                .filter_map(|(i, b)| (!indexes.contains(&i)).then_some(b))
                .collect();
            blocks.push(bridge);
            derived += 1;
        }
    }
    if derived > 0 {
        let mut warning=ApiFailure::new("LINTEL_CATALOG_UNAPPROVED",format!(
            "Собрано {derived} перемычек из исходных физических деталей без изменения их геометрии; объединённые изделия и запилы не подтверждены производственным каталогом"),None);
        warning.occurrences = derived;
        diagnostics.push(warning);
    }
    blocks.sort_by(|a, b| {
        a.course_index
            .cmp(&b.course_index)
            .then_with(|| a.id.cmp(&b.id))
    });
    (blocks, diagnostics)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn request() -> LayoutRequest {
        serde_json::from_value(serde_json::json!({
        "schema_version":1,"z0_mm":0,"wall_volumes":[],"opening_volumes":[{
        "guid":"opening","startXmm":0,"startYmm":0,"endXmm":960,"endYmm":0,
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":100,"endTopZmm":100,
        "openingType":"OPENING"}]}))
        .unwrap()
    }
    fn block(id: &str, start: i64, length: i64, course: i64) -> Block {
        serde_json::from_value(serde_json::json!({"id":id,"wall_id":"wall","edge_id":"run",
        "course_index":course,"z_centimm":course*6300,"start":{"x":start*100,"y":0},
        "end":{"x":(start+length)*100,"y":0},"length_centimm":length*100,"kind":"ordinary",
        "catalog_status":"approved","rotation_deg":0,"is_bridge":false,
        "hide_spikes_left":false,"hide_spikes_right":false,"cuts":[],"source_ids":["wall:wall"]}))
        .unwrap()
    }
    fn profile() -> Profile {
        serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
    }
    #[test]
    fn width_categories_and_course_offsets_are_independent() {
        let mut r = request();
        let o = &mut r.opening_volumes[0];
        assert_eq!(layer_offsets(o, 960.), Some(&[0, 2, 4][..]));
        assert_eq!(layer_offsets(o, 1280.), Some(&[0, 2, 4, 6, 8][..]));
        assert_eq!(layer_offsets(o, 1920.), None);
        o.end_top_zmm = 110.;
        assert_eq!(layer_offsets(o, 960.), None);
        let r = request();
        let blocks = (0..8)
            .map(|c| block(&format!("b{c}"), -320, 1600, c))
            .collect();
        let (blocks, _) = assemble_lintel_layers(&r, blocks, &profile());
        let courses: Vec<_> = blocks
            .iter()
            .filter(|b| b.is_bridge)
            .map(|b| b.course_index)
            .collect();
        assert_eq!(courses, vec![2, 4, 6]);
    }
    #[test]
    fn t_axis_parts_preserve_cuts_and_physical_components() {
        let r = request();
        let mut node = block("node", 320, 640, 2);
        node.kind = "node_T".into();
        node.product_key = Some("Type6".into());
        node.cuts = vec!["Type6:x2:y1:wall".into()];
        let left = block("left", -320, 640, 2);
        let right = block("right", 960, 640, 2);
        let mut stem = block("stem", 320, 640, 2);
        stem.rotation_deg = 90;
        let (blocks, _) = assemble_lintel_layers(
            &r,
            vec![left.clone(), node.clone(), right.clone(), stem.clone()],
            &profile(),
        );
        let bridge = blocks.iter().find(|b| b.is_bridge).unwrap();
        assert_eq!(bridge.components, vec![left, node, right]);
        assert_eq!(bridge.cuts, vec!["Type6:x2:y1:wall"]);
        assert!(blocks.contains(&stem));
        assert_eq!(bridge.catalog_nominal_centimm, Some(192000));
        assert!(bridge.source_ids.contains(&"opening:opening".into()));
    }
}

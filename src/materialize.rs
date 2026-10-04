//! Геометрия готовых деталей после осевых разрезов; снятие шипов зависит от происхождения торца.
use crate::api::{ApiFailure, LayoutRequest, Volume};
use crate::layout::{Block, Profile};
use crate::solid_geometry::{self as solid, Mesh, Plane};
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::BTreeSet;

type V = [f64; 3];
fn dot(a: V, b: V) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}
fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn plane(n: V, point: V) -> Plane {
    Plane {
        normal: n,
        offset: dot(n, point),
    }
}
struct Obstacle { planes: Vec<Plane> }
fn prism(v: &Volume, width: f64, extend_ends: f64) -> Obstacle {
    let len = (v.end_xmm - v.start_xmm).hypot(v.end_ymm - v.start_ymm);
    let x = [
        (v.end_xmm - v.start_xmm) / len,
        (v.end_ymm - v.start_ymm) / len,
        0.0,
    ];
    let y = [-x[1], x[0], 0.0];
    let start = [v.start_xmm, v.start_ymm, 0.0];
    let bottom_slope = (v.end_bottom_zmm - v.start_bottom_zmm) / len;
    let top_slope = (v.end_top_zmm - v.start_top_zmm) / len;
    let mut planes = vec![
        plane(scale(x, -1.0), add(start, scale(x, -extend_ends))),
        plane(x, add(start, scale(x, len + extend_ends))),
        plane(scale(y, -1.0), add(start, scale(y, -width / 2.0))),
        plane(y, add(start, scale(y, width / 2.0))),
        plane(
            [x[0] * bottom_slope, x[1] * bottom_slope, -1.0],
            [v.start_xmm, v.start_ymm, v.start_bottom_zmm],
        ),
        plane(
            [-x[0] * top_slope, -x[1] * top_slope, 1.0],
            [v.start_xmm, v.start_ymm, v.start_top_zmm],
        ),
    ];
    Obstacle { planes: std::mem::take(&mut planes) }
}
fn clip_all(mut mesh: Mesh, planes: &[Plane]) -> Option<Mesh> {
    for p in planes {
        mesh = solid::clip(&mesh, p)?;
    }
    Some(mesh)
}
fn combine(parts: &[Mesh]) -> Mesh {
    let mut result = Mesh {
        vertices: Vec::new(),
        faces: Vec::new(),
    };
    for part in parts {
        let offset = result.vertices.len();
        result.vertices.extend_from_slice(&part.vertices);
        result.faces.extend(
            part.faces
                .iter()
                .map(|f| f.iter().map(|i| i + offset).collect()),
        );
    }
    result
}
fn local_mesh(mesh: &Mesh, origin: V, axes: [V; 3]) -> Mesh {
    Mesh {
        vertices: mesh
            .vertices
            .iter()
            .map(|p| {
                let q = sub(*p, origin);
                [dot(q, axes[0]), dot(q, axes[1]), dot(q, axes[2])]
            })
            .collect(),
        faces: mesh.faces.clone(),
    }
}
fn frame_json(origin: V, axes: [V; 3]) -> Value {
    json!({"origin_mm":origin,"x_axis":axes[0],"y_axis":axes[1],"z_axis":axes[2]})
}
fn local_plane_cut(p: Plane, origin: V, axes: [V; 3], ids: &[String]) -> Value {
    let normal = [
        dot(p.normal, axes[0]),
        dot(p.normal, axes[1]),
        dot(p.normal, axes[2]),
    ];
    let offset = p.offset - dot(p.normal, origin);
    let point = scale(normal, offset / dot(normal, normal));
    json!({"kind":"plane_cut","point_local_mm":point,"normal_local":normal,"keep_side":"negative","source_ids":ids})
}

#[derive(Serialize)]
pub struct PhysicalBlock {
    pub id: String,
    pub kind: String,
    pub course_index: i64,
    pub source_ids: Vec<String>,
    pub bodies: Vec<Mesh>,
    pub cut: bool,
    pub is_dobor: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dobor_group: Option<crate::offcut_plan::DoborGroup>,
    pub cut_from_block_id: Option<String>,
}
#[derive(Serialize)]
pub struct PhysicalResult {
    pub format: &'static str,
    pub project_name: String,
    pub snapshot_hash: String,
    pub detail_level: &'static str,
    pub blocks: Vec<PhysicalBlock>,
}
pub struct Materialized {
    pub physical: PhysicalResult,
    pub exchange_blocks: Vec<Value>,
    pub diagnostics: Vec<ApiFailure>,
}

pub fn materialize(
    request: &LayoutRequest,
    blocks: &[Block],
    profile: &Profile,
) -> Result<Materialized, ApiFailure> {
    let prepared = crate::offcut_plan::prepare(request, profile, blocks)?;
    let mut effective = request.clone();
    crate::effective_geometry::normalize_beams(&mut effective);
    let mut materialized = materialize_parts(&effective, &prepared.blocks)?;
    for block in &mut materialized.exchange_blocks { prepared.annotate(block); }
    for block in &mut materialized.physical.blocks {
        block.cut_from_block_id = prepared.parents.get(&block.id).cloned();
        block.dobor_group = prepared.dobor_groups.get(&block.id).cloned();
        block.is_dobor = block.dobor_group.is_some();
    }
    Ok(materialized)
}

fn materialize_parts(
    request: &LayoutRequest,
    blocks: &[Block],
) -> Result<Materialized, ApiFailure> {
    let mut physical = Vec::new();
    let mut exchange = Vec::new();
    let mut diagnostics = Vec::new();
    let mut profile_warning: Option<ApiFailure> = None;
    for block in blocks {
        crate::code_result::validate_end_states(block)?;
        if let Some(warning) = crate::node_shapes::node_profile_warning(block) {
            if let Some(existing) = profile_warning.as_mut() {
                existing.occurrences += 1;
                existing.source_ids.extend(warning.source_ids);
            } else {
                let mut warning = warning;
                warning.course_index = None;
                warning.source_id = None;
                profile_warning = Some(warning);
            }
        }
        if !block.components.is_empty() {
            let components = materialize_parts(request, &block.components)?;
            diagnostics.extend(components.diagnostics);
            let cut = components.physical.blocks.iter().any(|b| b.cut) || !block.cuts.is_empty();
            let bodies: Vec<Mesh> = components
                .physical
                .blocks
                .into_iter()
                .flat_map(|b| b.bodies)
                .collect();
            if bodies.is_empty() {
                continue;
            }
            let origin = [
                block.start.x as f64 / 100.0,
                block.start.y as f64 / 100.0,
                block.z_centimm as f64 / 100.0,
            ];
            let a = (block.rotation_deg as f64).to_radians();
            let axes = [
                [a.cos(), a.sin(), 0.0],
                [-a.sin(), a.cos(), 0.0],
                [0.0, 0.0, 1.0],
            ];
            let shape = local_mesh(&combine(&bodies), origin, axes);
            let source_ids: Vec<String> = components
                .exchange_blocks
                .iter()
                .flat_map(|v| {
                    v["source_ids"]
                        .as_array()
                        .into_iter()
                        .flatten()
                        .filter_map(Value::as_str)
                })
                .map(str::to_owned)
                .chain(
                    block
                        .source_ids
                        .iter()
                        .filter_map(|id| id.strip_prefix("opening:").map(str::to_owned)),
                )
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect();
            let wall_ids: Vec<String> = components
                .exchange_blocks
                .iter()
                .flat_map(|v| {
                    v["wall_ids"]
                        .as_array()
                        .into_iter()
                        .flatten()
                        .filter_map(Value::as_str)
                })
                .map(str::to_owned)
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect();
            let width = request
                .wall_volumes
                .iter()
                .find(|w| wall_ids.contains(&w.guid))
                .map(|w| w.thickness_mm)
                .unwrap_or(193.0);
            let nominal = block.length_centimm as f64 / 100.0;
            exchange.push(json!({"id":block.id,"source_ids":source_ids,"wall_ids":wall_ids,"course_index":block.course_index,
                "product":{"category":"lintel","type_id":"DERIVED_LINTEL","code":format!("DERIVED-LINTEL-{nominal}"),"nominal_size_mm":[nominal,width,63.0]},
                "placement":frame_json(origin,axes),"stock_shape":{"kind":"mesh","vertices_mm":shape.vertices,"faces":shape.faces},"cuts":[],
                "spikes_removed":{"left":block.hide_spikes_left(),"right":block.hide_spikes_right()},"natural_end_left":block.natural_end_left(),"natural_end_right":block.natural_end_right(),"left_end":block.ends.left(),"right_end":block.ends.right()}));
            physical.push(PhysicalBlock {
                is_dobor: false,
                dobor_group: None,
                cut_from_block_id: None,
                id: block.id.clone(),
                kind: "bridge".into(),
                course_index: block.course_index,
                source_ids: block.source_ids.clone(),
                bodies,
                cut,
            });
            continue;
        }
        let walls: Vec<_> = request
            .wall_volumes
            .iter()
            .filter(|w| block.source_ids.contains(&format!("wall:{}", w.guid)))
            .collect();
        let Some(primary) = walls.first() else {
            return Err(ApiFailure::new(
                "MISSING_WALL_GEOMETRY",
                "Нет исходной стены детали",
                Some(block.id.clone()),
            ));
        };
        let width = primary.thickness_mm;
        let height = 63.0;
        let length = block.length_centimm as f64 / 100.0;
        let origin = [
            block.start.x as f64 / 100.0,
            block.start.y as f64 / 100.0,
            block.z_centimm as f64 / 100.0,
        ];
        let a = (block.rotation_deg as f64).to_radians();
        let axes = [
            [a.cos(), a.sin(), 0.0],
            [-a.sin(), a.cos(), 0.0],
            [0.0, 0.0, 1.0],
        ];
        let nominal = length;
        let has_node_profile = block.kind.starts_with("node_")
            || block.cuts.iter().any(|cut| cut.starts_with("Type"));
        let stocks = if has_node_profile {
            crate::node_shapes::node_stock(block, width, height)
                .map_err(|m| ApiFailure::new("UNSUPPORTED_NODE_SHAPE", m, Some(block.id.clone())))?
        } else {
            vec![solid::box_mesh(
                add(origin, scale(axes[1], -width / 2.0)),
                axes,
                [nominal, width, height],
            )]
        };
        let stock = combine(&stocks);
        let mut bodies = stocks;
        let mut cuts = Vec::new();
        let sources: Vec<String> = block
            .source_ids
            .iter()
            .filter(|id| !id.starts_with("edge:"))
            .map(|id| {
                id.split_once(':')
                    .filter(|(p, _)| matches!(*p, "wall" | "beam" | "opening"))
                    .map_or_else(|| id.clone(), |(_, v)| v.to_owned())
            })
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        for (natural, hidden, at_left) in [
            (block.natural_end_left(), block.hide_spikes_left(), true),
            (block.natural_end_right(), block.hide_spikes_right(), false),
        ] {
            if natural && hidden {
                let p = if at_left {
                    plane(scale(axes[0], -1.0), add(origin, scale(axes[0], 5.0)))
                } else {
                    plane(axes[0], add(origin, scale(axes[0], length - 5.0)))
                };
                bodies = bodies.into_iter().filter_map(|m| solid::clip(&m, &p)).collect();
                cuts.push(local_plane_cut(p, origin, axes, &sources));
            }
        }
        // Union of the source wall volumes preserves sloping roof surfaces and floor starts.
        let mut in_walls = Vec::new();
        let mut remaining = bodies;
        for w in &walls {
            let shape = prism(
                w,
                w.thickness_mm,
                if block.kind.starts_with("node_") {
                    width / 2.0
                } else {
                    0.0
                },
            );
            let mut rest = Vec::new();
            for m in remaining {
                if let Some(inside) = clip_all(m.clone(), &shape.planes) {
                    in_walls.push(inside);
                }
                rest.extend(solid::subtract(&m, &shape.planes));
            }
            remaining = rest;
        }
        bodies = in_walls;
        // A per-wall cap is expressible as a plane only for one source wall.
        if walls.len() == 1 {
            let p = prism(
                primary,
                width,
                if block.kind.starts_with("node_") {
                    width / 2.0
                } else {
                    0.0
                },
            );
            for cap in &p.planes[4..] {
                if stock
                    .vertices
                    .iter()
                    .any(|v| dot(cap.normal, *v) > cap.offset + 1e-6)
                {
                    cuts.push(local_plane_cut(*cap, origin, axes, &sources));
                }
            }
        }
        if bodies.is_empty() {
            continue;
        }
        let cut = !cuts.is_empty() || !block.cuts.is_empty();
        let product = if block.is_bridge {
            "lintel"
        } else if length > 640.0 {
            "special"
        } else {
            "standard"
        };
        let code = block.product_key.clone().unwrap_or_else(|| {
            if block.is_bridge {
                format!("LINTEL-{nominal}")
            } else {
                format!("P{nominal}")
            }
        });
        let local = local_mesh(&stock, origin, axes);
        let mut value = json!({"id":block.id,"source_ids":sources,"wall_ids":walls.iter().map(|w|w.guid.clone()).collect::<Vec<_>>(),"course_index":block.course_index,
            "product":{"category":product,"type_id":code,"code":code,"nominal_size_mm":[nominal,width,height]},"placement":frame_json(origin,axes),
            "stock_shape":{"kind":"mesh","vertices_mm":local.vertices,"faces":local.faces},"cuts":cuts,"spikes_removed":{"left":block.hide_spikes_left(),"right":block.hide_spikes_right()},"natural_end_left":block.natural_end_left(),"natural_end_right":block.natural_end_right(),"left_end":block.ends.left(),"right_end":block.ends.right()});
        if product == "special" {
            value["special_origin"] = json!({"rule_id":"minimum_ordinary_span_merge","component_ids":value["source_ids"]});
        }
        exchange.push(value);
        physical.push(PhysicalBlock {
            is_dobor: false,
            dobor_group: None,
            cut_from_block_id: None,
            id: block.id.clone(),
            kind: block.kind.clone(),
            course_index: block.course_index,
            source_ids: block.source_ids.clone(),
            bodies,
            cut,
        });
    }
    if let Some(mut warning) = profile_warning {
        warning.source_ids.sort();
        warning.source_ids.dedup();
        diagnostics.push(warning);
    }
    Ok(Materialized {
        physical: PhysicalResult {
            format: "physical_body_v1",
            project_name: request.project_name.clone(),
            snapshot_hash: request.snapshot_hash.clone(),
            detail_level: "finished_parts_with_node_profiles_without_spike_mesh",
            blocks: physical,
        },
        exchange_blocks: exchange,
        diagnostics,
    })
}

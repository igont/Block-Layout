//! Геометрия просмотра из выбранных деталей; номиналы и реальные вычеты раздельны.
use crate::api::{ApiFailure, LayoutRequest, Volume};
use crate::layout::Block;
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
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn unit(a: V) -> V {
    scale(a, 1.0 / dot(a, a).sqrt())
}
fn plane(n: V, point: V) -> Plane {
    Plane {
        normal: n,
        offset: dot(n, point),
    }
}
fn overlap(a: (V, V), b: (V, V)) -> bool {
    (0..3).all(|i| a.0[i] < b.1[i] - 1e-7 && b.0[i] < a.1[i] - 1e-7)
}

#[derive(Clone)]
struct Obstacle {
    id: String,
    planes: Vec<Plane>,
    bounds: (V, V),
    frame: Option<(V, [V; 3], V)>,
}
fn box_obstacle(id: String, origin: V, axes: [V; 3], size: V) -> Obstacle {
    let mut planes = Vec::new();
    for i in 0..3 {
        planes.push(plane(scale(axes[i], -1.0), origin));
        planes.push(plane(axes[i], add(origin, scale(axes[i], size[i]))));
    }
    let mesh = solid::box_mesh(origin, axes, size);
    Obstacle {
        id,
        planes,
        bounds: solid::bounds(&mesh).expect("проверенная коробка"),
        frame: Some((origin, axes, size)),
    }
}
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
    let low_z = v.start_bottom_zmm.min(v.end_bottom_zmm) - bottom_slope.abs() * extend_ends;
    let hi_z = v.start_top_zmm.max(v.end_top_zmm) + top_slope.abs() * extend_ends;
    let mesh = solid::box_mesh(
        add(add(start, scale(x, -extend_ends)), [0.0, 0.0, low_z]),
        [x, y, [0.0, 0.0, 1.0]],
        [len + 2.0 * extend_ends, width, hi_z - low_z],
    );
    // The box above is only a broad bound, not the shape of a sloping opening.
    let mut bounds = solid::bounds(&mesh).expect("проверенная призма");
    bounds.0[0] -= width;
    bounds.0[1] -= width;
    bounds.1[0] += width;
    bounds.1[1] += width;
    let frame = if bottom_slope.abs() < 1e-9 && top_slope.abs() < 1e-9 {
        Some((
            add(
                add(start, scale(x, -extend_ends)),
                add(scale(y, -width / 2.0), [0.0, 0.0, low_z]),
            ),
            [x, y, [0.0, 0.0, 1.0]],
            [len + 2.0 * extend_ends, width, hi_z - low_z],
        ))
    } else {
        None
    };
    Obstacle {
        id: v.guid.clone(),
        planes: std::mem::take(&mut planes),
        bounds,
        frame,
    }
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

pub fn materialize(request: &LayoutRequest, blocks: &[Block]) -> Result<Materialized, ApiFailure> {
    let mut obstacles = Vec::new();
    for b in &request.beams {
        let start = [b.start_xmm, b.start_ymm, b.start_zmm];
        let end = [b.end_xmm, b.end_ymm, b.end_zmm];
        let x = unit(sub(end, start));
        let z = unit([
            b.geometry.height_direction_x,
            b.geometry.height_direction_y,
            b.geometry.height_direction_z,
        ]);
        let y = unit(cross(z, x));
        obstacles.push(box_obstacle(
            b.guid.clone(),
            add(start, scale(y, -b.geometry.width_mm / 2.0)),
            [x, y, z],
            [
                (dot(sub(end, start), sub(end, start))).sqrt(),
                b.geometry.width_mm,
                b.geometry.height_mm,
            ],
        ));
    }
    for o in &request.opening_volumes {
        if o.opening_type == "OPENING" || o.opening_type == "CONSOLE" {
            obstacles.push(prism(o, 100_000.0, 0.0));
        }
    }
    let mut physical = Vec::new();
    let mut exchange = Vec::new();
    let mut diagnostics = Vec::new();
    let mut profile_warning: Option<ApiFailure> = None;
    for block in blocks {
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
            let components = materialize(request, &block.components)?;
            diagnostics.extend(components.diagnostics);
            let bodies: Vec<Mesh> = components
                .physical
                .blocks
                .into_iter()
                .flat_map(|b| b.bodies)
                .collect();
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
            let nominal = block
                .catalog_nominal_centimm
                .unwrap_or(block.length_centimm) as f64
                / 100.0;
            exchange.push(json!({"id":block.id,"source_ids":source_ids,"wall_ids":wall_ids,"course_index":block.course_index,
                "product":{"category":"lintel","type_id":"DERIVED_LINTEL","code":format!("DERIVED-LINTEL-{nominal}"),"nominal_size_mm":[nominal,width,63.0]},
                "placement":frame_json(origin,axes),"stock_shape":{"kind":"mesh","vertices_mm":shape.vertices,"faces":shape.faces},"cuts":[],
                "spikes_removed":{"left":block.hide_spikes_left,"right":block.hide_spikes_right}}));
            physical.push(PhysicalBlock {
                id: block.id.clone(),
                kind: "bridge".into(),
                course_index: block.course_index,
                source_ids: block.source_ids.clone(),
                bodies,
                cut: !block.cuts.is_empty(),
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
        let nominal = block
            .catalog_nominal_centimm
            .unwrap_or(block.length_centimm) as f64
            / 100.0;
        let stocks = if block.kind.starts_with("node_") {
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
        if !block.kind.starts_with("node_") && length < nominal - 1e-6 {
            let p = plane(axes[0], add(origin, scale(axes[0], length)));
            bodies = bodies
                .into_iter()
                .filter_map(|m| solid::clip(&m, &p))
                .collect();
            cuts.push(local_plane_cut(p, origin, axes, &sources));
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
        let Some(bound) = solid::bounds(&stock) else {
            return Err(ApiFailure::new(
                "EMPTY_STOCK",
                "Пустая геометрия заготовки",
                Some(block.id.clone()),
            ));
        };
        for obstacle in &obstacles {
            if !overlap(bound, obstacle.bounds) {
                continue;
            }
            // Openings attach to their own wall axis; a thin projection is not a global tunnel.
            if let Some(o) = request
                .opening_volumes
                .iter()
                .find(|o| o.guid == obstacle.id)
            {
                let aligned = walls.iter().any(|w| {
                    let wx = w.end_xmm - w.start_xmm;
                    let wy = w.end_ymm - w.start_ymm;
                    let ox = o.end_xmm - o.start_xmm;
                    let oy = o.end_ymm - o.start_ymm;
                    (wx * oy - wy * ox).abs() < 1e-5
                        && ((o.start_xmm - w.start_xmm) * wy - (o.start_ymm - w.start_ymm) * wx)
                            .abs()
                            < 0.02 * wx.hypot(wy)
                });
                if !aligned {
                    continue;
                }
            }
            let before: f64 = bodies.iter().map(solid::volume).sum();
            let mut next = Vec::new();
            for m in bodies {
                next.extend(solid::subtract(&m, &obstacle.planes));
            }
            bodies = next;
            let after: f64 = bodies.iter().map(solid::volume).sum();
            if before - after > 1e-5 {
                if let Some((o, ax, size)) = obstacle.frame {
                    let loc = sub(o, origin);
                    let local_origin = [dot(loc, axes[0]), dot(loc, axes[1]), dot(loc, axes[2])];
                    let local_axes =
                        ax.map(|d| [dot(d, axes[0]), dot(d, axes[1]), dot(d, axes[2])]);
                    cuts.push(json!({"kind":"box_cut","frame_local":frame_json(local_origin,local_axes),"size_mm":size,"source_ids":[obstacle.id]}));
                } else {
                    let planes:Vec<Value>=obstacle.planes.iter().map(|p|json!({"normal":[dot(p.normal,axes[0]),dot(p.normal,axes[1]),dot(p.normal,axes[2])],"offset_mm":p.offset-dot(p.normal,origin)})).collect();
                    cuts.push(json!({"kind":"convex_cut","planes_local":planes,"source_ids":[obstacle.id]}));
                }
            }
            if bodies.len() > 256 {
                return Err(ApiFailure::new(
                    "GEOMETRY_LIMIT",
                    "Более 256 тел в одной детали",
                    Some(block.id.clone()),
                ));
            }
        }
        if bodies.is_empty() {
            return Err(ApiFailure::new(
                "EMPTY_FINISHED_PART",
                "Выбранная деталь целиком удалена препятствием",
                Some(block.id.clone()),
            ));
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
            "stock_shape":{"kind":"mesh","vertices_mm":local.vertices,"faces":local.faces},"cuts":cuts,"spikes_removed":{"left":block.hide_spikes_left||cut,"right":block.hide_spikes_right||cut}});
        if product == "special" {
            value["special_origin"] = json!({"rule_id":"minimum_ordinary_span_merge","component_ids":value["source_ids"]});
        }
        exchange.push(value);
        physical.push(PhysicalBlock {
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
            detail_level: "nominal_with_node_profiles_and_obstacle_cuts_without_spike_mesh",
            blocks: physical,
        },
        exchange_blocks: exchange,
        diagnostics,
    })
}

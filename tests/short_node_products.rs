use fb_layout::{api::LayoutRequest, layout::Profile};
use serde_json::json;

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}

fn request(distance: i64, rotation: u16) -> LayoutRequest {
    let turn = |p: (i64, i64)| match rotation {
        0 => p,
        90 => (-p.1, p.0),
        180 => (-p.0, -p.1),
        270 => (p.1, -p.0),
        _ => unreachable!(),
    };
    let wall = |id: &str, a: (i64, i64), b: (i64, i64)| {
        let a = turn(a);
        let b = turn(b);
        json!({"guid":id,"startXmm":a.0+15680,"startYmm":a.1+7360,
            "endXmm":b.0+15680,"endYmm":b.1+7360,"startBottomZmm":0,"endBottomZmm":0,
            "startTopZmm":126,"endTopZmm":126,"thicknessMm":193,"purposeType":1})
    };
    serde_json::from_value(json!({"schema_version":1,"request_id":"short-nodes",
        "snapshot_hash":"a".repeat(64),"z0_mm":0,"wall_volumes":[
            wall("through",(-2560,0),(distance,0)),wall("tee",(0,0),(0,-1600)),
            wall("corner",(distance,0),(distance,1920))]}))
    .unwrap()
}

fn intersection(a: &fb_layout::solid_geometry::Mesh, b: &fb_layout::solid_geometry::Mesh) -> f64 {
    use fb_layout::solid_geometry::{clip, volume, Plane};
    // Compute near the shared product to avoid cancellation at House coordinates.
    let origin = a.vertices[0];
    let local = |mesh: &fb_layout::solid_geometry::Mesh| {
        let mut mesh = mesh.clone();
        for p in &mut mesh.vertices {
            for i in 0..3 {
                p[i] -= origin[i];
            }
        }
        mesh
    };
    let a = local(a);
    let b = local(b);
    let mut body = Some(a);
    for face in &b.faces {
        let p = b.vertices[face[0]];
        let q = b.vertices[face[1]];
        let r = b.vertices[face[2]];
        let u = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
        let v = [r[0] - p[0], r[1] - p[1], r[2] - p[2]];
        let normal = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        body = body.and_then(|body| {
            clip(
                &body,
                &Plane {
                    normal,
                    offset: normal[0] * p[0] + normal[1] * p[1] + normal[2] * p[2],
                },
            )
        });
    }
    body.as_ref().map_or(0.0, volume)
}

#[test]
fn shared_640_stock_changes_corner_phase_and_exports_two_cuts_once() {
    let request = request(640, 0);
    let profile = profile();
    let result = fb_layout::inspect_request(&request, &profile);
    assert!(result.diagnostics.is_empty(), "{:?}", result.diagnostics);
    let combined: Vec<_> = result
        .blocks
        .iter()
        .filter(|b| b.kind == "node_compound")
        .collect();
    assert_eq!(combined.len(), 1);
    let part = combined[0];
    assert_eq!(part.course_index, 1);
    assert_eq!(part.length_centimm, 64000);
    assert!(part.cuts.iter().any(|c| c.starts_with("Type8:p0:")));
    assert!(part.cuts.iter().any(|c| c.starts_with("Type1:p64000:")));
    assert_eq!(part.arms.len(), 1);
    assert_eq!(part.arms[0].length_centimm, 64000);
    for source in ["through", "tee", "corner"] {
        assert!(
            part.source_ids.iter().any(|id| id.ends_with(source)),
            "{source}: {:?}",
            part.source_ids
        );
    }
    let standard = fb_layout::exchange::from_layout_request(&request, &profile).unwrap();
    let codes =
        fb_layout::code_result::export_codes(&standard, &request, &profile, &result.blocks, &[])
            .unwrap();
    let exported = codes["blocks"]
        .as_array()
        .unwrap()
        .iter()
        .find(|v| v["id"] == part.id)
        .unwrap();
    assert_eq!(exported["product_type"], "Комбинированный");
    assert_eq!(exported["code1"], "П640 [Н-НЧ-Тип 8] [В-КН-Тип 1]");
    assert_eq!(exported["length_mm"], 736.5);
    let even: Vec<_> = result
        .blocks
        .iter()
        .filter(|b| b.course_index == 0 && b.kind == "node_L")
        .collect();
    assert!(even
        .iter()
        .any(|b| b.product_key.as_deref() == Some("Type3")));
    assert!(even
        .iter()
        .any(|b| b.product_key.as_deref() == Some("Type4")));
    let rendered = fb_layout::materialize::materialize(&request, &result.blocks, &profile).unwrap();
    let physical = rendered
        .physical
        .blocks
        .iter()
        .find(|b| b.id == part.id)
        .unwrap();
    assert_eq!(
        rendered
            .physical
            .blocks
            .iter()
            .filter(|b| b.id == part.id)
            .count(),
        1
    );
    for other in rendered
        .physical
        .blocks
        .iter()
        .filter(|b| b.course_index == part.course_index && b.id != part.id)
    {
        let overlap: f64 = physical
            .bodies
            .iter()
            .flat_map(|a| other.bodies.iter().map(move |b| intersection(a, b)))
            .sum();
        // The sampled catalog curves meet within a cubic millimetre.
        assert!(
            overlap < 1.0,
            "{} overlaps combined stock: {overlap}",
            other.id
        );
    }
}

#[test]
fn pier_keeps_two_half_blocks_alternating_with_one_stock_above_course_66() {
    for rotation in [0, 90, 180, 270] {
        let mut request = request(640, rotation);
        let direction = match rotation { 0 => (1.0,0.0), 90 => (0.0,1.0),
            180 => (-1.0,0.0), _ => (0.0,-1.0) };
        request.wall_volumes[0].start_xmm = 15680.0;
        request.wall_volumes[0].start_ymm = 7360.0;
        let mut second_corner = request.wall_volumes[1].clone();
        second_corner.guid = "second-corner".into();
        second_corner.start_xmm += direction.0 * 640.0;
        second_corner.end_xmm += direction.0 * 640.0;
        second_corner.start_ymm += direction.1 * 640.0;
        second_corner.end_ymm += direction.1 * 640.0;
        request.wall_volumes[2] = second_corner;
        for wall in &mut request.wall_volumes {
            wall.start_bottom_zmm = 66.0 * 63.0;
            wall.end_bottom_zmm = 66.0 * 63.0;
            wall.start_top_zmm = 70.0 * 63.0;
            wall.end_top_zmm = 70.0 * 63.0;
        }
        let profile = profile();
        let result = fb_layout::inspect_request(&request, &profile);
        assert!(result.diagnostics.is_empty(), "rotation={rotation}: {:?}", result.diagnostics);
        let along = |p: fb_layout::domain::Point| {
            ((p.x-1568000) as f64 * direction.0 + (p.y-736000) as f64 * direction.1) as i64
        };
        let across = |p: fb_layout::domain::Point| {
            ((p.x-1568000) as f64 * direction.1 - (p.y-736000) as f64 * direction.0) as i64
        };
        let mut counts = Vec::new();
        for course in 66..70 {
            let parts: Vec<_> = result.blocks.iter().filter(|b| b.course_index == course
                && across(b.start) == 0 && across(b.end) == 0
                && along(b.start).min(along(b.end)) < 64000
                && along(b.start).max(along(b.end)) > 0).collect();
            let mut lengths: Vec<_> = parts.iter().map(|b| b.length_centimm).collect();
            lengths.sort();
            assert!(lengths == vec![64000] || lengths == vec![32000,32000],
                "rotation={rotation} course={course}: {lengths:?} parts={parts:?}");
            counts.push(parts.len());
        }
        assert!(counts == vec![1,2,1,2] || counts == vec![2,1,2,1],
            "rotation={rotation}: {counts:?}");
        let standard = fb_layout::exchange::from_layout_request(&request, &profile).unwrap();
        let exported = fb_layout::code_result::export_codes(&standard, &request, &profile, &result.blocks, &[]).unwrap();
        assert_eq!(exported["blocks"].as_array().unwrap().len(), result.blocks.len());
    }
}

#[test]
fn short_shared_node_stock_has_one_body_for_all_axis_orientations() {
    for distance in [640, 320, 200, 160, 100] {
        for rotation in [0, 90, 180, 270] {
            let request = request(distance, rotation);
            let mut profile = profile();
            // Non-grid node positions still need the same physical stock rule.
            if distance % 320 != 0 {
                profile.world_joint_policy = None;
            }
            let result = fb_layout::inspect_request(&request, &profile);
            assert!(
                !result.blocks.is_empty(),
                "distance={distance} rotation={rotation}: {:?}",
                result.diagnostics
            );
            let parts: Vec<_> = result
                .blocks
                .iter()
                .filter(|b| b.kind == "node_compound")
                .collect();
            assert!(!parts.is_empty(), "distance={distance} rotation={rotation}");
            if distance % 320 == 0 {
                assert!(
                    !result
                        .diagnostics
                        .iter()
                        .any(|d| d.code == "VERTICAL_JOINT_CONFLICT"),
                    "{:?}",
                    result.diagnostics
                );
                let corner_types = |course| {
                    let mut types: Vec<_> = result
                        .blocks
                        .iter()
                        .filter(|b| b.course_index == course)
                        .flat_map(|b| b.cuts.iter())
                        .filter_map(|c| c.split(':').next())
                        .filter(|key| matches!(*key, "Type1" | "Type2" | "Type3" | "Type4"))
                        .collect();
                    types.sort();
                    types
                };
                assert_ne!(
                    corner_types(0),
                    corner_types(1),
                    "distance={distance} rotation={rotation}"
                );
            }
            let standard = fb_layout::exchange::from_layout_request(&request, &profile).unwrap();
            let codes = fb_layout::code_result::export_codes(
                &standard,
                &request,
                &profile,
                &result.blocks,
                &[],
            )
            .unwrap();
            let rendered =
                fb_layout::materialize::materialize(&request, &result.blocks, &profile).unwrap();
            for part in parts {
                assert!(part.length_centimm <= 64000);
                assert!(part.cuts.len() >= 2);
                let bodies = fb_layout::node_shapes::node_stock(part, 193.0, 63.0).unwrap();
                assert!(
                    bodies
                        .iter()
                        .map(fb_layout::solid_geometry::volume)
                        .sum::<f64>()
                        > 0.0
                );
                let products: Vec<_> = codes["blocks"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .filter(|b| b["id"] == part.id)
                    .collect();
                assert_eq!(products.len(), 1);
                assert_eq!(products[0]["product_type"], "Комбинированный");
                assert_eq!(
                    products[0]["node_cuts"].as_array().unwrap().len(),
                    part.cuts.len()
                );
                assert_eq!(
                    rendered
                        .physical
                        .blocks
                        .iter()
                        .filter(|b| b.id == part.id)
                        .count(),
                    1
                );
            }
        }
    }
}

use fb_layout::{
    api::LayoutRequest,
    code_result::export_codes,
    exchange,
    lamella::{self, Lamella},
    layout::{Block, Profile},
};
use serde_json::json;

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}
fn wall(id: &str, a: [f64; 2], b: [f64; 2]) -> serde_json::Value {
    json!({"guid":id,"startXmm":a[0],"startYmm":a[1],"endXmm":b[0],"endYmm":b[1],
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":126,"endTopZmm":126,"thicknessMm":193,"purposeType":1})
}
fn beam(id: &str, a: [f64; 2], b: [f64; 2]) -> serde_json::Value {
    json!({"guid":id,"startXmm":a[0],"startYmm":a[1],"endXmm":b[0],"endYmm":b[1],
        "startZmm":0,"endZmm":0,"geometry":{"widthMm":160,"heightMm":320,"heightDirectionZ":1}})
}
fn request(walls: Vec<serde_json::Value>, beams: Vec<serde_json::Value>) -> LayoutRequest {
    serde_json::from_value(
        json!({"schema_version":1,"request_id":"lamella","snapshot_hash":"a".repeat(64),
        "z0_mm":0,"wall_volumes":walls,"beams":beams}),
    )
    .unwrap()
}
fn bounds(l: &Lamella) -> [f64; 4] {
    let c = l.placement.center_mm;
    let u = l.placement.x_axis;
    let v = l.placement.y_axis;
    let ex = (u[0].abs() * l.length_mm + v[0].abs() * l.thickness_mm) / 2.0;
    let ey = (u[1].abs() * l.length_mm + v[1].abs() * l.thickness_mm) / 2.0;
    [c[0] - ex, c[0] + ex, c[1] - ey, c[1] + ey]
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-6, "{a} != {b}");
}

#[test]
fn gable_source_boundary_below_both_slopes_does_not_split_selected_cells() {
    let mut a = wall("gable-a", [0.0, 0.0], [960.75, 0.0]);
    let mut b = wall("gable-b", [960.75, 0.0], [1920.0, 0.0]);
    a["endTopZmm"] = json!(315.0);
    b["startTopZmm"] = json!(315.0);
    let r = request(vec![a, b], vec![beam("beam", [800.0, 0.0], [1100.0, 0.0])]);
    let result = lamella::calculate(&r, &profile(), &[]).unwrap();
    for l in result.lamellas.iter().filter(|l| l.course_index < 2) {
        close(l.length_mm, 640.0);
        let bb = bounds(l);
        if bb[0] < 960.75 && bb[1] > 960.75 {
            assert_eq!(l.wall_ids.len(), 2);
        }
    }
    assert!(result
        .lamellas
        .iter()
        .any(|l| l.course_index == 0 && l.wall_ids.len() == 2));
}

#[test]
fn beam_region_preserves_actual_course_grid_without_any_reference_block() {
    let r = request(
        vec![wall("wall", [0.0, 0.0], [1920.0, 0.0])],
        vec![beam("beam", [600.0, 0.0], [1000.0, 0.0])],
    );
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    assert_eq!(ls.len(), 8);
    for (course, expected) in [
        (0, vec![(320.0, 960.0), (960.0, 1600.0)]),
        (1, vec![(0.0, 640.0), (640.0, 1280.0)]),
    ] {
        for side in ["left", "right"] {
            let parts: Vec<_> = ls
                .iter()
                .filter(|l| l.course_index == course && l.side == side)
                .collect();
            let mut actual: Vec<_> = parts
                .iter()
                .map(|l| {
                    let b = bounds(l);
                    (b[0], b[1])
                })
                .collect();
            actual.sort_by(|a, b| a.0.total_cmp(&b.0));
            assert_eq!(actual, expected);
            for l in parts {
                close(
                    l.placement.center_mm[1],
                    if side == "left" { 89.0 } else { -89.0 },
                );
                close(l.height_mm, 63.0);
                close(l.thickness_mm, 15.0);
                assert_eq!(l.beam_ids.len(), 1);
            }
        }
    }
    // An 80mm beam half-width leaves a real 1.5mm gap to the skin inner face.
    close(
        bounds(ls.iter().find(|l| l.side == "left").unwrap())[2] - 80.0,
        1.5,
    );
    let standard = exchange::from_layout_request(&r, &profile()).unwrap();
    let result = export_codes(&standard, &r, &profile(), &[], &[]).unwrap();
    assert_eq!(result["lamellas"].as_array().unwrap().len(), 8);
}

#[test]
fn short_beam_is_not_lost_and_normalized_height_does_not_add_sixth_course() {
    let mut r = request(
        vec![wall("wall", [-1280.0, 0.0], [1280.0, 0.0])],
        vec![beam("short", [-7.0, 0.0], [7.0, 0.0])],
    );
    r.wall_volumes[0].start_top_zmm = 630.0;
    r.wall_volumes[0].end_top_zmm = 630.0;
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    assert!(ls.iter().all(|l| l.course_index < 5));
    assert!(ls.iter().any(|l| l.course_index == 4));
    assert!(ls.iter().all(|l| l.length_mm <= 640.0 + 1e-7));
    for course in 0..5 {
        for side in ["left", "right"] {
            let mut spans: Vec<_> = ls
                .iter()
                .filter(|l| l.course_index == course && l.side == side)
                .map(bounds)
                .collect();
            spans.sort_by(|a, b| a[0].total_cmp(&b[0]));
            assert!(spans.first().unwrap()[0] <= -7.0 && spans.last().unwrap()[1] >= 7.0);
            for pair in spans.windows(2) {
                close(pair[0][1], pair[1][0]);
            }
        }
    }
    r.beams[0].geometry.height_direction_z = -1.0;
    r.beams[0].start_zmm = 320.0;
    r.beams[0].end_zmm = 320.0;
    assert_eq!(
        serde_json::to_value(lamella::calculate(&r, &profile(), &[]).unwrap().lamellas).unwrap(),
        serde_json::to_value(ls).unwrap()
    );
}

#[test]
fn l_corner_internal_clip_external_extension_and_finite_butt_are_independent_of_input_order() {
    let r = request(
        vec![
            wall("x", [0.0, 0.0], [1280.0, 0.0]),
            wall("y", [0.0, 0.0], [0.0, 1280.0]),
        ],
        vec![
            beam("bx", [0.0, 0.0], [1280.0, 0.0]),
            beam("by", [0.0, 0.0], [0.0, 1280.0]),
        ],
    );
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    let first = |wall: &str, side: &str| {
        ls.iter()
            .filter(|l| l.course_index == 0 && l.wall_ids.contains(wall) && l.side == side)
            .min_by(|a, b| {
                let ba = bounds(a);
                let bb = bounds(b);
                (ba[0] + ba[2]).total_cmp(&(bb[0] + bb[2]))
            })
            .unwrap()
    };
    let xi = bounds(first("x", "left"));
    let yi = bounds(first("y", "right"));
    close(xi[0], 96.5);
    close(yi[2], 81.5);
    let xe = bounds(first("x", "right"));
    let ye = bounds(first("y", "left"));
    close(xe[0], -81.5);
    close(ye[2], -96.5);
    close(xe[0], ye[1]);
    // Independent physical rectangle intersections: positive volume is forbidden.
    for (index, a) in ls.iter().enumerate() {
        for b in ls
            .iter()
            .skip(index + 1)
            .filter(|b| b.course_index == a.course_index)
        {
            let aa = bounds(a);
            let bb = bounds(b);
            let ix = aa[1].min(bb[1]) - aa[0].max(bb[0]);
            let iy = aa[3].min(bb[3]) - aa[2].max(bb[2]);
            assert!(
                ix <= 1e-7 || iy <= 1e-7,
                "overlap {} {}: {ix}x{iy}",
                a.id,
                b.id
            );
        }
    }
    let mut reversed = r.clone();
    reversed.wall_volumes.reverse();
    reversed.beams.reverse();
    for w in &mut reversed.wall_volumes {
        std::mem::swap(&mut w.start_xmm, &mut w.end_xmm);
        std::mem::swap(&mut w.start_ymm, &mut w.end_ymm);
    }
    for b in &mut reversed.beams {
        std::mem::swap(&mut b.start_xmm, &mut b.end_xmm);
        std::mem::swap(&mut b.start_ymm, &mut b.end_ymm);
    }
    assert_eq!(
        serde_json::to_value(
            lamella::calculate(&reversed, &profile(), &[])
                .unwrap()
                .lamellas
        )
        .unwrap(),
        serde_json::to_value(ls).unwrap()
    );
}

#[test]
fn module_extension_into_block_exports_identical_physical_subtraction() {
    let r = request(
        vec![wall("wall", [0.0, 0.0], [1920.0, 0.0])],
        vec![beam("beam", [600.0, 0.0], [1000.0, 0.0])],
    );
    let block: Block = serde_json::from_value(json!({"id":"part","wall_id":"wall","edge_id":"run",
        "course_index":0,"z_centimm":0,"start":{"x":32000,"y":0},"end":{"x":58000,"y":0},
        "length_centimm":26000,"kind":"ordinary","product_key":null,"catalog_status":"KnownPattern",
        "rotation_deg":0,"local_origin":null,"local_rotation_deg":null,"is_bridge":false,
        "hide_spikes_left":false,"hide_spikes_right":false,"cuts":[],"source_ids":["wall:wall"],
        "catalog_nominal_centimm":64000,"arms":[]}))
    .unwrap();
    let standard = exchange::from_layout_request(&r, &profile()).unwrap();
    let blocks_only =
        fb_layout::code_result::export_blocks(&standard, &r, &profile(), &[block.clone()], &[])
            .unwrap();
    assert_eq!(blocks_only["lamellas"], json!([]));
    assert!(!blocks_only["blocks"][0]["trims"]
        .as_array()
        .unwrap()
        .iter()
        .any(|t| t["kind"] == "lamella_recess"));
    let result = export_codes(&standard, &r, &profile(), &[block], &[]).unwrap();
    let trims = result["blocks"][0]["trims"].as_array().unwrap();
    let recesses: Vec<_> = trims
        .iter()
        .filter(|t| t["kind"] == "lamella_recess")
        .collect();
    assert_eq!(recesses.len(), 2);
    for recess in recesses {
        let l = result["lamellas"]
            .as_array()
            .unwrap()
            .iter()
            .find(|l| l["id"] == recess["lamella_id"])
            .unwrap();
        for field in ["placement", "length_mm", "thickness_mm", "height_mm"] {
            assert_eq!(recess[field], l[field]);
        }
        assert!(recess["placement"].get("origin_mm").is_none());
    }
}

#[test]
fn no_relevant_beams_always_exports_empty_array() {
    for beams in [vec![], vec![beam("remote", [0.0, 1000.0], [640.0, 1000.0])]] {
        let r = request(vec![wall("wall", [0.0, 0.0], [1280.0, 0.0])], beams);
        let standard = exchange::from_layout_request(&r, &profile()).unwrap();
        assert_eq!(
            export_codes(&standard, &r, &profile(), &[], &[]).unwrap()["lamellas"],
            json!([])
        );
    }
}

#[test]
fn collinear_source_boundary_keeps_full_grid_module_and_all_native_ids() {
    let r = request(
        vec![
            wall("a", [0.0, 0.0], [640.0, 0.0]),
            wall("b", [640.0, 0.0], [1920.0, 0.0]),
        ],
        vec![beam("beam", [600.0, 0.0], [700.0, 0.0])],
    );
    let plan = lamella::calculate(&r, &profile(), &[]).unwrap();
    let l = plan
        .lamellas
        .iter()
        .find(|l| l.course_index == 0 && l.side == "left")
        .unwrap();
    close(bounds(l)[0], 320.0);
    close(bounds(l)[1], 960.0);
    assert_eq!(
        l.wall_ids,
        std::collections::BTreeSet::from(["a".into(), "b".into()])
    );
    assert_eq!(
        l.source_ids,
        std::collections::BTreeSet::from(["a".into(), "b".into(), "beam".into()])
    );
}

#[test]
fn diagonal_run_uses_shared_local_grid_and_world_centered_frame() {
    let r = request(
        vec![wall("diag", [-1280.0, -1280.0], [1280.0, 1280.0])],
        vec![beam("beam", [-700.0, -700.0], [-600.0, -600.0])],
    );
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    let l = ls
        .iter()
        .find(|l| l.course_index == 0 && l.side == "left")
        .unwrap();
    let axis = l.placement.x_axis;
    close(axis[0], 1.0 / 2.0_f64.sqrt());
    close(axis[1], 1.0 / 2.0_f64.sqrt());
    let c = l.placement.center_mm;
    close((c[1] - c[0]) / 2.0_f64.sqrt(), 89.0);
    let center_along = (c[0] + 1280.0) * axis[0] + (c[1] + 1280.0) * axis[1];
    close(center_along - l.length_mm / 2.0, 320.0);
    close(l.length_mm, 640.0);
}

#[test]
fn custom_profile_without_selected_grid_reports_partial_coverage() {
    let r = request(
        vec![wall("wall", [0.0, 0.0], [1280.0, 0.0])],
        vec![beam("beam", [0.0, 0.0], [1280.0, 0.0])],
    );
    let mut p = profile();
    p.world_joint_policy = None;
    let standard = exchange::from_layout_request(&r, &p).unwrap();
    let result = export_codes(&standard, &r, &p, &[], &[]).unwrap();
    assert_eq!(result["status"], "success");
    assert_eq!(result["lamellas"], json!([]));
    let warnings = result["warnings"].as_array().unwrap();
    assert_eq!(warnings.len(), 2);
    assert!(warnings
        .iter()
        .all(|w| w["code"] == "LAMELLA_GRID_UNAVAILABLE"
            && w["source_ids"] == json!(["beam", "wall"])));
}

#[test]
fn t_branch_keeps_finite_corner_butts_without_physical_overlap() {
    let r = request(
        vec![
            wall("trunk", [-1280.0, 0.0], [1280.0, 0.0]),
            wall("branch", [0.0, 0.0], [0.0, 1280.0]),
        ],
        vec![
            beam("trunk-beam", [-1280.0, 0.0], [1280.0, 0.0]),
            beam("branch-beam", [0.0, 0.0], [0.0, 1280.0]),
        ],
    );
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    for side in ["left", "right"] {
        let l = ls
            .iter()
            .filter(|l| l.course_index == 0 && l.side == side && l.wall_ids.contains("branch"))
            .min_by(|a, b| bounds(a)[2].total_cmp(&bounds(b)[2]))
            .unwrap();
        assert!((bounds(l)[2] - 81.5).abs() < 1e-7 || (bounds(l)[2] - 96.5).abs() < 1e-7);
    }
    for (index, a) in ls.iter().enumerate() {
        for b in ls
            .iter()
            .skip(index + 1)
            .filter(|b| b.course_index == a.course_index)
        {
            let aa = bounds(a);
            let bb = bounds(b);
            assert!(
                aa[1].min(bb[1]) - aa[0].max(bb[0]) <= 1e-7
                    || aa[3].min(bb[3]) - aa[2].max(bb[2]) <= 1e-7,
                "overlap {} {}",
                a.id,
                b.id
            );
        }
    }
}

#[test]
fn sloped_wall_emits_exact_partial_course_solid_and_interior_center() {
    let mut r = request(
        vec![wall("slope", [0.0, 0.0], [1280.0, 0.0])],
        vec![beam("beam", [0.0, 0.0], [1280.0, 0.0])],
    );
    r.wall_volumes[0].start_top_zmm = 31.5;
    r.wall_volumes[0].end_top_zmm = 157.5;
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    for side in ["left", "right"] {
        let mut volume = 0.0;
        for l in ls.iter().filter(|l| l.course_index == 1 && l.side == side) {
            if let Some(mesh) = &l.solid {
                assert!(mesh
                    .vertices
                    .iter()
                    .all(|p| p[2] <= 31.5 + p[0] * 126.0 / 1280.0 + 1e-7 && p[2] >= 63.0 - 1e-7));
                let c = l.placement.center_mm;
                assert!(c[2] <= 31.5 + c[0] * 126.0 / 1280.0 + 1e-7 && c[2] >= 63.0 - 1e-7);
                volume += fb_layout::solid_geometry::volume(mesh);
            } else {
                volume += l.length_mm * l.thickness_mm * l.height_mm;
            }
        }
        // Independent trapezoid/rectangle integral of the sloped 63mm band.
        assert!((volume - 604800.0).abs() < 0.01, "{volume}");
    }
    assert!(ls.iter().any(|l| l.course_index == 1 && l.solid.is_some()));
    let mut blocks = vec![
        json!({"id":"partial","placement":{"origin_mm":[320.0,0.0,63.0],
        "x_axis":[1.0,0.0,0.0],"y_axis":[0.0,1.0,0.0],"z_axis":[0.0,0.0,1.0]},
        "length_mm":640.0,"width_mm":193.0,"height_mm":63.0,"trims":[]}),
    ];
    lamella::add_recesses(&mut blocks, &[], &ls, &r);
    let solid_trim = blocks[0]["trims"]
        .as_array()
        .unwrap()
        .iter()
        .find(|t| t.get("solid").is_some())
        .unwrap();
    let exact = ls
        .iter()
        .find(|l| l.id == solid_trim["lamella_id"].as_str().unwrap())
        .unwrap();
    assert_eq!(
        solid_trim["solid"],
        serde_json::to_value(exact.solid.as_ref().unwrap()).unwrap()
    );
    let mut reversed = r.clone();
    let wall = &mut reversed.wall_volumes[0];
    std::mem::swap(&mut wall.start_xmm, &mut wall.end_xmm);
    std::mem::swap(&mut wall.start_ymm, &mut wall.end_ymm);
    std::mem::swap(&mut wall.start_top_zmm, &mut wall.end_top_zmm);
    std::mem::swap(&mut wall.start_bottom_zmm, &mut wall.end_bottom_zmm);
    assert_eq!(
        serde_json::to_value(
            lamella::calculate(&reversed, &profile(), &[])
                .unwrap()
                .lamellas
        )
        .unwrap(),
        serde_json::to_value(ls).unwrap()
    );
}

#[test]
fn real_banya_transverse_beam_above_lower_wall_endpoint_keeps_five_courses() {
    let mut w = wall(
        "3373C02A-E845-4697-955E-E13E5248610D",
        [13120.0, 1920.0],
        [13120.0, 5760.0],
    );
    w["startTopZmm"] = json!(3297.323);
    w["endTopZmm"] = json!(4848.784);
    w["startBottomZmm"] = json!(-252.0);
    w["endBottomZmm"] = json!(-252.0);
    let mut b = beam(
        "D8CAC7EA-74E2-4078-BEAE-0FF8F30C15CF",
        [11103.5, 3520.0],
        [15175.0, 3520.0],
    );
    b["startZmm"] = json!(3465.0);
    b["endZmm"] = json!(3465.0);
    let mut r = request(vec![w], vec![b]);
    r.z0_mm = -252.0;
    let plan = lamella::calculate(&r, &profile(), &[]).unwrap();
    assert!(plan.warnings.is_empty());
    let courses: std::collections::BTreeSet<_> =
        plan.lamellas.iter().map(|l| l.course_index).collect();
    assert_eq!(
        courses,
        std::collections::BTreeSet::from([59, 60, 61, 62, 63])
    );
    for l in &plan.lamellas {
        close(l.height_mm, 63.0);
        assert_eq!(l.beam_ids.len(), 1);
    }
}

fn physical_mesh(l: &Lamella) -> fb_layout::solid_geometry::Mesh {
    if let Some(mesh) = &l.solid {
        return mesh.clone();
    }
    let p = &l.placement;
    let axes = [p.x_axis, p.y_axis, p.z_axis];
    let size = [l.length_mm, l.thickness_mm, l.height_mm];
    let origin = std::array::from_fn(|i| {
        p.center_mm[i] - (0..3).map(|j| axes[j][i] * size[j] / 2.0).sum::<f64>()
    });
    fb_layout::solid_geometry::box_mesh(origin, axes, size)
}
fn intersection_volume(
    a: &fb_layout::solid_geometry::Mesh,
    b: &fb_layout::solid_geometry::Mesh,
) -> f64 {
    let centroid: [f64; 3] = std::array::from_fn(|i| {
        b.vertices.iter().map(|p| p[i]).sum::<f64>() / b.vertices.len() as f64
    });
    let mut mesh = a.clone();
    for face in &b.faces {
        let p = b.vertices[face[0]];
        let q = b.vertices[face[1]];
        let r = b.vertices[face[2]];
        let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
        let e = [r[0] - p[0], r[1] - p[1], r[2] - p[2]];
        let mut n = [
            d[1] * e[2] - d[2] * e[1],
            d[2] * e[0] - d[0] * e[2],
            d[0] * e[1] - d[1] * e[0],
        ];
        if (0..3).map(|i| n[i] * (centroid[i] - p[i])).sum::<f64>() > 0.0 {
            for x in &mut n {
                *x = -*x;
            }
        }
        let plane = fb_layout::solid_geometry::Plane {
            normal: n,
            offset: (0..3).map(|i| n[i] * p[i]).sum(),
        };
        let Some(clipped) = fb_layout::solid_geometry::clip(&mesh, &plane) else {
            return 0.0;
        };
        mesh = clipped;
    }
    fb_layout::solid_geometry::volume(&mesh)
}

#[test]
fn oblique_corner_emits_exact_butt_meshes_without_positive_volume_overlap() {
    for x in [-1280.0, 1280.0] {
        let r = request(
            vec![
                wall("x", [0.0, 0.0], [1280.0, 0.0]),
                wall("diag", [0.0, 0.0], [x, 1280.0]),
            ],
            vec![
                beam("x-beam", [0.0, 0.0], [1280.0, 0.0]),
                beam("diag-beam", [0.0, 0.0], [x, 1280.0]),
            ],
        );
        let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
        assert!(ls.iter().any(|l| l.solid.is_some()));
        for (index, a) in ls.iter().enumerate() {
            for b in ls
                .iter()
                .skip(index + 1)
                .filter(|b| b.course_index == a.course_index)
            {
                let volume = intersection_volume(&physical_mesh(a), &physical_mesh(b));
                assert!(volume < 1e-5, "overlap {} {} = {volume}", a.id, b.id);
            }
        }
    }
}

#[test]
fn transverse_beam_ending_inside_wall_selects_only_reached_facade() {
    for (start, end, expected) in [(-1000.0, 76.5, "right"), (-76.5, 1000.0, "left")] {
        let r = request(
            vec![wall("wall", [0.0, 0.0], [1920.0, 0.0])],
            vec![beam("partial", [1000.0, start], [1000.0, end])],
        );
        let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
        assert!(!ls.is_empty());
        assert!(ls.iter().all(|l| l.side == expected));
        for l in &ls {
            let b = bounds(l);
            assert!(b[0] < 1080.0 && b[1] > 920.0);
        }
        let mut reversed = r.clone();
        let b = &mut reversed.beams[0];
        std::mem::swap(&mut b.start_xmm, &mut b.end_xmm);
        std::mem::swap(&mut b.start_ymm, &mut b.end_ymm);
        assert_eq!(
            serde_json::to_value(ls).unwrap(),
            serde_json::to_value(
                lamella::calculate(&reversed, &profile(), &[])
                    .unwrap()
                    .lamellas
            )
            .unwrap()
        );
    }
    let r = request(
        vec![wall("wall", [0.0, 0.0], [1920.0, 0.0])],
        vec![beam("cross", [1000.0, -1000.0], [1000.0, 81.5])],
    );
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    assert!(ls.iter().any(|l| l.side == "left"));
    assert!(ls.iter().any(|l| l.side == "right"));
}

#[test]
fn longitudinal_beam_preserves_centered_skin_gap_but_offset_selects_near_facade() {
    for y in [-20.0, 0.0, 20.0] {
        let r = request(
            vec![wall("wall", [0.0, 0.0], [1920.0, 0.0])],
            vec![beam("parallel", [600.0, y], [1000.0, y])],
        );
        let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
        let sides: std::collections::BTreeSet<_> = ls.iter().map(|l| l.side).collect();
        let expected = if y < 0.0 {
            std::collections::BTreeSet::from(["right"])
        } else if y > 0.0 {
            std::collections::BTreeSet::from(["left"])
        } else {
            std::collections::BTreeSet::from(["left", "right"])
        };
        assert_eq!(sides, expected, "offset {y}");
    }
}

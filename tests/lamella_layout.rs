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
        assert!(l.length_mm > 0.0);
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
        (1, vec![(-96.5, 640.0), (640.0, 1280.0)]),
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

#[test]
fn partial_height_and_sloped_wall_keep_whole_bonded_cells() {
    let mut w = wall("slope", [0.0, 0.0], [1280.0, 0.0]);
    w["startTopZmm"] = json!(31.5);
    w["endTopZmm"] = json!(157.5);
    let mut b = beam("beam", [500.0, 0.0], [700.0, 0.0]);
    b["startZmm"] = json!(70.0);
    b["endZmm"] = json!(70.0);
    b["geometry"]["heightMm"] = json!(20.0);
    let r = request(vec![w], vec![b]);
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    assert!(!ls.is_empty());
    for l in &ls {
        assert!(serde_json::to_value(l).unwrap().get("solid").is_none());
        close(l.height_mm, 63.0);
        assert!(l.length_mm > 0.0);
        assert_eq!(l.course_index, 1);
        close(l.placement.center_mm[2], 94.5);
    }
}

#[test]
fn transverse_skew_and_parallel_outside_wall_do_not_create_lamellas() {
    for (a, b) in [
        ([600.0, -500.0], [600.0, 500.0]),
        ([0.0, -300.0], [1200.0, 300.0]),
        ([400.0, 500.0], [900.0, 500.0]),
    ] {
        let r = request(
            vec![wall("wall", [0.0, 0.0], [1920.0, 0.0])],
            vec![beam("beam", a, b)],
        );
        assert!(lamella::calculate(&r, &profile(), &[])
            .unwrap()
            .lamellas
            .is_empty());
    }
}

#[test]
fn overlapping_longitudinal_beams_share_cells_and_keep_all_sources() {
    let w = wall("wall", [0.0, 0.0], [1920.0, 0.0]);
    let a = beam("a", [500.0, 0.0], [700.0, 0.0]);
    let b = beam("b", [500.0, 0.0], [700.0, 0.0]);
    let single =
        lamella::calculate(&request(vec![w.clone()], vec![a.clone()]), &profile(), &[]).unwrap();
    let both = lamella::calculate(
        &request(vec![w.clone()], vec![a.clone(), b.clone()]),
        &profile(),
        &[],
    )
    .unwrap();
    let reversed = lamella::calculate(&request(vec![w], vec![b, a]), &profile(), &[]).unwrap();
    assert!(!single.lamellas.is_empty());
    assert_eq!(single.lamellas.len(), both.lamellas.len());
    assert_eq!(
        serde_json::to_value(&both.lamellas).unwrap(),
        serde_json::to_value(&reversed.lamellas).unwrap()
    );
    assert!(both
        .lamellas
        .iter()
        .all(|l| l.beam_ids.len() == 2 && serde_json::to_value(l).unwrap().get("solid").is_none()));
}

#[test]
fn oblique_wall_corner_does_not_cut_grid_rectangles() {
    let r = request(
        vec![
            wall("a", [0.0, 0.0], [1280.0, 0.0]),
            wall("b", [0.0, 0.0], [900.0, 900.0]),
        ],
        vec![
            beam("a", [0.0, 0.0], [500.0, 0.0]),
            beam("b", [0.0, 0.0], [400.0, 400.0]),
        ],
    );
    let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
    assert!(!ls.is_empty());
    for l in ls {
        assert!(l.length_mm > 0.0);
        close(l.height_mm, 63.0);
        assert!(serde_json::to_value(l).unwrap().get("solid").is_none());
    }
}

#[test]
fn terminal_cells_extend_or_shorten_to_axis_plus_half_section() {
    for reversed in [false, true] {
        let (a, b) = if reversed {
            ([1280.0, 0.0], [0.0, 0.0])
        } else {
            ([0.0, 0.0], [1280.0, 0.0])
        };
        let r = request(vec![wall("wall", a, b)], vec![beam("beam", a, b)]);
        let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
        for course in [0, 1] {
            for side in ["left", "right"] {
                let mut spans: Vec<_> = ls
                    .iter()
                    .filter(|l| l.course_index == course && l.side == side)
                    .map(|l| {
                        let b = bounds(l);
                        (b[0], b[1])
                    })
                    .collect();
                spans.sort_by(|a, b| a.0.total_cmp(&b.0));
                let expected = if course == 0 {
                    vec![(-96.5, 320.0), (320.0, 960.0), (960.0, 1376.5)]
                } else {
                    vec![(-96.5, 640.0), (640.0, 1376.5)]
                };
                assert_eq!(spans, expected);
            }
        }
    }
}

#[test]
fn five_mm_beam_allowance_does_not_select_neighbor_cells_or_lower_course() {
    for allowance in [0.0, 5.0] {
        for reversed in [false, true] {
            let (a, b) = if reversed {
                (960.0 + allowance, 320.0 - allowance)
            } else {
                (320.0 - allowance, 960.0 + allowance)
            };
            let mut beam = beam("beam", [a, 0.0], [b, 0.0]);
            beam["startZmm"] = json!(0.0);
            beam["endZmm"] = json!(0.0);
            beam["geometry"]["heightMm"] = json!(63.0);
            let r = request(vec![wall("wall", [0.0, 0.0], [1920.0, 0.0])], vec![beam]);
            let ls = lamella::calculate(&r, &profile(), &[]).unwrap().lamellas;
            assert_eq!(ls.len(), 2);
            for l in ls {
                assert_eq!(&bounds(&l)[..2], &[320.0, 960.0]);
            }
        }
    }
}

#[test]
fn perpendicular_corner_uses_the_inner_and_outer_face_lengths() {
    for reversed in [false, true] {
        let endpoints = if reversed {
            ([1920.0, 0.0], [0.0, 0.0])
        } else {
            ([0.0, 0.0], [1920.0, 0.0])
        };
        let r = request(
            vec![
                wall("through", endpoints.0, endpoints.1),
                wall("return", [0.0, 0.0], [0.0, 900.0]),
            ],
            vec![beam("beam", [0.0, 0.0], [500.0, 0.0])],
        );
        let plan = lamella::calculate(&r, &profile(), &[]).unwrap();
        for (side, expected) in [("left", (96.5, 320.0)), ("right", (-96.5, 320.0))] {
            let first = plan
                .lamellas
                .iter()
                .filter(|l| l.course_index == 0 && l.side == side)
                .map(bounds)
                .min_by(|a, b| a[0].total_cmp(&b[0]))
                .unwrap();
            close(first[0], expected.0);
            close(first[1], expected.1);
        }
    }
}

#[test]
fn tee_splits_the_near_face_at_the_stem_but_keeps_the_far_face_grid() {
    let mut stem = wall("stem", [800.0, 0.0], [800.0, 900.0]);
    stem["thicknessMm"] = json!(250);
    let r = request(
        vec![wall("through", [0.0, 0.0], [1920.0, 0.0]), stem],
        vec![beam("beam", [650.0, 0.0], [710.0, 0.0])],
    );
    let plan = lamella::calculate(&r, &profile(), &[]).unwrap();
    let spans: Vec<_> = plan
        .lamellas
        .iter()
        .filter(|l| l.course_index == 0)
        .map(|l| (l.side, bounds(l)))
        .collect();
    assert_eq!(spans.len(), 2);
    for (side, expected) in [("left", (320.0, 675.0)), ("right", (320.0, 960.0))] {
        let (_, part) = spans.iter().find(|(name, _)| *name == side).unwrap();
        close(part[0], expected.0);
        close(part[1], expected.1);
    }
}

#[test]
fn transverse_beam_blocks_only_the_side_and_courses_it_reaches() {
    let mut facade_wall = wall("wall", [0.0, 0.0], [1920.0, 0.0]);
    facade_wall["startTopZmm"] = json!(189);
    facade_wall["endTopZmm"] = json!(189);
    let mut stop = beam("transverse", [900.0, 500.0], [900.0, 50.0]);
    stop["startZmm"] = json!(63);
    stop["endZmm"] = json!(63);
    stop["geometry"]["widthMm"] = json!(120);
    stop["geometry"]["heightMm"] = json!(63);
    let r = request(
        vec![facade_wall],
        vec![beam("longitudinal", [700.0, 0.0], [850.0, 0.0]), stop],
    );
    let plan = lamella::calculate(&r, &profile(), &[]).unwrap();
    assert_eq!(plan.lamellas.len(), 6);
    for part in plan.lamellas {
        let expected = match (part.course_index, part.side) {
            (1, "left") => (640.0, 840.0),
            (1, "right") => (640.0, 1280.0),
            _ => (320.0, 960.0),
        };
        assert_eq!(
            part.beam_ids,
            std::collections::BTreeSet::from(["longitudinal".to_owned()])
        );
        close(bounds(&part)[0], expected.0);
        close(bounds(&part)[1], expected.1);
    }
}

#[test]
fn opening_jambs_clip_the_step_only_in_intersected_courses() {
    let mut main = wall("wall", [0.0, 0.0], [1920.0, 0.0]);
    main["startTopZmm"] = json!(189);
    main["endTopZmm"] = json!(189);
    let mut r = request(vec![main], vec![beam("beam", [0.0, 0.0], [1920.0, 0.0])]);
    r.opening_volumes = serde_json::from_value(json!([
        {"guid":"sloped-opening","openingType":"OPENING","isOutside":true,
         "startXmm":1090,"startYmm":0,"endXmm":850,"endYmm":0,
         "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":94.5,"endTopZmm":31.5},
        {"guid":"other-wall","openingType":"WINDOW","isOutside":true,
         "startXmm":550,"startYmm":450,"endXmm":800,"endYmm":450,
         "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":189,"endTopZmm":189}
    ]))
    .unwrap();
    let plan = lamella::calculate(&r, &profile(), &[]).unwrap();
    for side in ["left", "right"] {
        for (course, opening_start) in [(0, 850.0), (1, 970.0)] {
            let spans: Vec<_> = plan
                .lamellas
                .iter()
                .filter(|l| l.side == side && l.course_index == course)
                .map(bounds)
                .collect();
            assert!(spans
                .iter()
                .all(|b| b[1] <= opening_start || b[0] >= 1090.0));
            assert!(spans.iter().any(|b| b[1] == opening_start));
            assert!(spans.iter().any(|b| b[0] == 1090.0));
            assert!(spans.iter().any(|b| b[0] <= 550.0 && b[1] > 550.0));
        }
        assert!(plan
            .lamellas
            .iter()
            .any(|l| l.side == side && l.course_index == 2 && bounds(l)[0] == 960.0));
    }
}

#[test]
fn blind_beam_end_covers_the_far_face_and_meets_the_longitudinal_strip() {
    for (end_x, has_cover) in [(-80.0, true), (-70.0, false), (-200.0, false)] {
        let r = request(
            vec![
                wall("through", [0.0, 0.0], [1920.0, 0.0]),
                wall("return", [0.0, -960.0], [0.0, 0.0]),
            ],
            vec![beam("beam", [end_x, 0.0], [900.0, 0.0])],
        );
        let plan = lamella::calculate(&r, &profile(), &[]).unwrap();
        let covers: Vec<_> = plan
            .lamellas
            .iter()
            .filter(|l| l.wall_ids.contains("return"))
            .collect();
        if has_cover {
            assert_eq!(covers.len(), 2);
            for cover in covers {
                let b = bounds(cover);
                close(b[0], -96.5);
                close(b[1], -81.5);
                close(b[3], 81.5);
                assert!(b[2] < -80.0);
                let mate = plan
                    .lamellas
                    .iter()
                    .filter(|l| {
                        l.course_index == cover.course_index
                            && l.wall_ids.contains("through")
                            && l.side == "left"
                    })
                    .map(bounds)
                    .min_by(|a, b| a[0].total_cmp(&b[0]))
                    .unwrap();
                close(mate[2], b[3]);
            }
        } else {
            assert!(covers.is_empty());
        }
    }
}

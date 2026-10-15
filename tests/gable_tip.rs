use fb_layout::{
    code_result::export_blocks,
    exchange::{parse_request, ExchangeRequest},
    layout::Profile,
    topology::{build_topology, normalize_building},
};
use serde_json::{json, Value};

const REQUEST: &str =
    include_str!("../Документация/Граничные контракты/examples/layout-request.v1.json");

fn source(start: [f64; 2], end: [f64; 2], bottom: f64, tops: [f64; 2]) -> Value {
    let mut value: Value = serde_json::from_str(REQUEST).unwrap();
    value["coordinate_system"]["z0_mm"] = json!(bottom);
    value["model"]["wall_volumes"][0]["volume"] = json!({
        "start_xy_mm":start,"end_xy_mm":end,
        "bottom_start_mm":bottom,"bottom_end_mm":bottom,
        "top_start_mm":tops[0],"top_end_mm":tops[1],
        "left_thickness_mm":96.5,"right_thickness_mm":96.5
    });
    value
}

#[test]
fn zero_tip_gable_passes_exchange_topology_and_codes_in_both_directions() {
    let profile: Profile =
        serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    for end in [[640., 0.], [0., 640.], [-640., 0.], [0., -640.]] {
        for tops in [[0., 252.], [252., 0.]] {
            let original = source([0., 0.], end, 0., tops);
            let mut expected_topology = None;
            for reverse in [false, true] {
                let value = if reverse {
                    source(end, [0., 0.], 0., [tops[1], tops[0]])
                } else {
                    original.clone()
                };
                let neutral = parse_request(&value.to_string()).unwrap();
                let request = neutral.to_layout_request().unwrap();
                let before = request.raw_building();
                let topology =
                    build_topology(normalize_building(request.raw_building()).unwrap()).unwrap();
                assert_eq!(
                    topology.courses.iter().map(|c| c.index).collect::<Vec<_>>(),
                    vec![0, 1, 2, 3]
                );
                // The affine top admits exactly the positive-height part of each course.
                for course in &topology.courses {
                    assert!(!course.runs.is_empty());
                    for run in &course.runs {
                        for point in [run.start, run.end] {
                            let distance = if end[0] != 0. {
                                point.x * end[0].signum() as i64
                            } else {
                                point.y * end[1].signum() as i64
                            };
                            let from_tip = if tops[0] == 0. {
                                distance
                            } else {
                                64000 - distance
                            };
                            assert!(from_tip > course.index * 16000);
                            assert!(from_tip <= 64000);
                        }
                    }
                }
                if let Some(expected) = &expected_topology {
                    assert_eq!(&topology, expected);
                } else {
                    expected_topology = Some(topology);
                }
                let result = fb_layout::inspect_request(&request, &profile);
                assert!(result.diagnostics.is_empty(), "{:?}", result.diagnostics);
                assert!(!result.blocks.is_empty());
                let codes =
                    export_blocks(&neutral, &request, &profile, &result.blocks, &[]).unwrap();
                let blocks = codes["blocks"].as_array().unwrap();
                assert!(!blocks.is_empty());
                for block in blocks {
                    assert_eq!(block["height_mm"], 63.0);
                    assert!(block["length_mm"].as_f64().unwrap() > 0.);
                    assert!(block["trims"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(|trim| trim["kind"] != "wall_boundary"
                            && trim["kind"] != "opening_slope"));
                }
                // Calculation must not shift the tip or raise its source height.
                assert_eq!(request.raw_building(), before);
                assert_eq!(serde_json::to_value(&neutral).unwrap(), value);
            }
        }
    }
}

#[test]
fn actual_project_96_gable_coordinates_survive_canonical_conversion() {
    // Original accepted current_entity payloads, retained before SQL migration.
    for (guid, start_x, end_x) in [
        ("8C6713DF-9799-42A2-A893-F48FDDD5A985", -2230., 527.5),
        ("A20BD88C-7C12-43B9-9E1B-01EAF43F4E32", 3510., 752.5),
    ] {
        let mut value = source([start_x, -7360.], [end_x, -7360.], 3339., [3339., 4468.001]);
        value["model"]["wall_volumes"][0]["id"] = json!(guid);
        let neutral = parse_request(&value.to_string()).unwrap();
        let request = neutral.to_layout_request().unwrap();
        let wall = &request.wall_volumes[0];
        assert_eq!(wall.guid, guid);
        assert_eq!((wall.start_xmm, wall.end_xmm), (start_x, end_x));
        assert_eq!((wall.start_top_zmm, wall.start_bottom_zmm), (3339., 3339.));
        assert_eq!(wall.end_top_zmm, 4468.);
        assert_eq!(neutral.model.wall_volumes[0].volume.top_end_mm, 4468.001);
        let topology = build_topology(normalize_building(request.raw_building()).unwrap()).unwrap();
        assert!(!topology.courses.is_empty());
        assert_eq!(topology.walls[0].id, guid);
    }
}

#[test]
fn collapsed_and_inverted_profiles_fail_at_exchange_and_direct_topology_boundaries() {
    for tops in [[0., 0.], [0.004, 0.004], [-0.01, 252.], [252., -0.01]] {
        let value = source([0., 0.], [640., 0.], 0., tops);
        assert_eq!(
            parse_request(&value.to_string()).unwrap_err().code,
            "INVALID_GEOMETRY"
        );
        // Direct callers bypass exchange, so topology owns the same invariant.
        let valid: ExchangeRequest =
            parse_request(&source([0., 0.], [640., 0.], 0., [63., 252.]).to_string()).unwrap();
        let mut raw = valid.to_layout_request().unwrap().raw_building();
        raw.walls[0].top_start_mm = tops[0];
        raw.walls[0].top_end_mm = tops[1];
        assert!(normalize_building(raw)
            .unwrap_err()
            .contains("wall-example"));
    }
}

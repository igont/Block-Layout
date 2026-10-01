use fb_layout::{api::LayoutRequest, code_result::export_codes, exchange, layout::{Block, Profile},
    materialize::materialize, offcut_plan::prepare, solid_geometry::volume};
use serde_json::{json, Value};

fn fixture(specs: &[(i64, bool, bool)]) -> (LayoutRequest, Profile, Vec<Block>) {
    let profile: Profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let mut walls = Vec::new();
    let mut openings = Vec::new();
    let mut blocks = Vec::new();
    for (i, &(length, hide_left, hide_right)) in specs.iter().enumerate() {
        let y = i as i64 * 1000;
        let z = i as i64 * 630;
        walls.push(json!({"guid":format!("wall-{i}"),"startXmm":-1000,"endXmm":2000,
            "startYmm":y,"endYmm":y,"startBottomZmm":0,"endBottomZmm":0,
            "startTopZmm":10000,"endTopZmm":10000,"thicknessMm":193,"purposeType":1}));
        for (side, start, end) in [("left", -320, 0), ("right", length / 100, length / 100 + 320)] {
            if (side == "left" && hide_left) || (side == "right" && hide_right) {
                openings.push(json!({"guid":format!("opening-{i}-{side}"),"startXmm":start,"endXmm":end,
                    "startYmm":y,"endYmm":y,"startBottomZmm":z,"endBottomZmm":z,
                    "startTopZmm":z+63,"endTopZmm":z+63,"thicknessMm":193,"openingType":"WINDOW"}));
            }
        }
        blocks.push(serde_json::from_value(json!({"id":format!("block-{i}"),"wall_id":format!("wall-{i}"),
            "edge_id":format!("run-{i}"),"course_index":i*10,"z_centimm":z*100,
            "start":{"x":0,"y":y*100},"end":{"x":length,"y":y*100},"length_centimm":length,
            "kind":"ordinary","product_key":null,"catalog_status":"length_only","rotation_deg":0,
            "local_origin":null,"local_rotation_deg":null,"is_bridge":false,
            "hide_spikes_left":hide_left,"hide_spikes_right":hide_right,
            "natural_end_left":true,"natural_end_right":true,"cuts":[],
            "source_ids":[format!("wall:wall-{i}")],"catalog_nominal_centimm":length})).unwrap());
    }
    let request = serde_json::from_value(json!({"schema_version":1,"request_id":"offcut-test",
        "snapshot_hash":"a".repeat(64),"z0_mm":0,"wall_volumes":walls,"opening_volumes":openings})).unwrap();
    (request, profile, blocks)
}

fn codes(request: &LayoutRequest, profile: &Profile, blocks: &[Block]) -> Value {
    let standard = exchange::from_layout_request(request, profile).unwrap();
    export_codes(&standard, request, profile, blocks, &[]).unwrap()
}

#[test]
fn opposite_profiles_across_walls_and_floors_share_one_stock_without_moving_the_bodies() {
    let (request, profile, blocks) = fixture(&[(32_000, false, true), (32_000, true, false)]);
    let result = codes(&request, &profile, &blocks);
    let a = &result["blocks"][0]; let b = &result["blocks"][1];
    assert_eq!(a["is_dobor"], false);
    assert!(a["cut_from_block_id"].is_null());
    assert_eq!(b["is_dobor"], true);
    assert_eq!(b["cut_from_block_id"], "block-0");
    assert_eq!(a["code1"], "П315"); assert_eq!(b["code1"], "П315");
    assert_eq!(b["placement"]["origin_mm"], json!([5.0,1000.0,630.0]));
    assert_eq!(a["natural_end_right"], false); assert_eq!(b["natural_end_left"], false);
    let plan = &a["stock_cut_plan"];
    assert_eq!(plan["parts"][0]["interval_mm"], json!([0.0,315.0]));
    assert_eq!(plan["parts"][1]["interval_mm"], json!([325.0,640.0]));
    assert_eq!(plan["saw_cuts_mm"], json!([[315.0,320.0],[320.0,325.0]]));
    assert_eq!(plan["waste_intervals_mm"], json!([]));
    let combined = materialize(&request, &blocks, &profile).unwrap();
    for (i, block) in blocks.iter().enumerate() {
        let original = materialize(&request, &[block.clone()], &profile).unwrap();
        let expected: f64 = original.physical.blocks[0].bodies.iter().map(volume).sum();
        let actual: f64 = combined.physical.blocks[i].bodies.iter().map(volume).sum();
        assert!((actual - expected).abs() < 1e-4);
        for axis in 0..3 {
            let bounds = |bodies: &[fb_layout::solid_geometry::Mesh]| {
                bodies.iter().flat_map(|m| &m.vertices).fold((f64::INFINITY, f64::NEG_INFINITY),
                    |(min,max), v| (min.min(v[axis]),max.max(v[axis])))
            };
            assert_eq!(bounds(&combined.physical.blocks[i].bodies), bounds(&original.physical.blocks[0].bodies));
        }
    }
    assert_eq!(combined.physical.blocks[1].cut_from_block_id.as_deref(), Some("block-0"));
    assert_eq!(combined.exchange_blocks[1]["cut_from_block_id"], "block-0");
    let prepared = prepare(&request, &profile, &blocks).unwrap();
    assert_eq!(codes(&request, &profile, &prepared.blocks), result);
    let mut reversed = blocks.clone(); reversed.reverse();
    assert_eq!(codes(&request, &profile, &reversed), result);
}

#[test]
fn same_profile_and_a_missing_saw_allowance_never_create_a_dobor() {
    for specs in [[(32_000,false,true),(32_000,false,true)],
        [(32_300,false,true),(32_300,true,false)]] {
        let (request, profile, blocks) = fixture(&specs);
        let result = codes(&request,&profile,&blocks);
        assert!(result["blocks"].as_array().unwrap().iter().all(|b| b["is_dobor"] == false));
    }
}

#[test]
fn each_stock_and_each_spike_are_consumed_once_and_the_choice_is_stable() {
    let (request,profile,blocks) = fixture(&[(32_000,false,true),(32_000,true,false),(32_000,true,false)]);
    let result = codes(&request,&profile,&blocks);
    let rows = result["blocks"].as_array().unwrap();
    assert_eq!(rows.iter().filter(|b| b["is_dobor"] == true).count(),1);
    assert_eq!(rows.iter().filter(|b| b.get("stock_cut_plan").is_some()).count(),1);
    let mut reversed = blocks; reversed.reverse();
    assert_eq!(codes(&request,&profile,&reversed),result);
}

#[test]
fn unrelated_short_blocks_nodes_and_gable_shapes_are_excluded() {
    let (mut request,profile,blocks) = fixture(&[(32_000,false,true),(32_000,true,false)]);
    request.opening_volumes.clear();
    assert!(prepare(&request,&profile,&blocks).unwrap().parents.is_empty());
    let (mut request,profile,mut blocks) = fixture(&[(32_000,false,true),(32_000,true,false)]);
    blocks[0].kind = "node_L".into(); blocks[0].product_key = Some("Type4".into());
    assert!(prepare(&request,&profile,&blocks).unwrap().parents.is_empty());
    blocks[0].kind = "ordinary".into(); blocks[0].product_key = None;
    blocks[0].cuts.push("Type4:x1:y1:run-0".into());
    assert!(prepare(&request,&profile,&blocks).unwrap().parents.is_empty());
    blocks[0].cuts.clear(); request.wall_volumes[0].start_top_zmm = 70.0;
    request.wall_volumes[0].end_top_zmm = 30.0;
    assert!(prepare(&request,&profile,&blocks).unwrap().parents.is_empty());
}

#[test]
fn flat_parts_require_both_factory_profiles_to_be_removed() {
    let (request,profile,blocks) = fixture(&[(32_000,true,true),(32_000,true,true)]);
    let prepared = prepare(&request,&profile,&blocks).unwrap();
    assert_eq!(prepared.parents.len(),1);
    let plan = prepared.stock_plans.values().next().unwrap();
    assert!(plan.parts.iter().all(|p| p.stock_end == "none"));
    assert_eq!(plan.parts[0].interval_mm, [5.0,315.0]);
    assert_eq!(plan.parts[1].interval_mm, [325.0,635.0]);
    assert!(plan.saw_cuts_mm.contains(&[0.0,5.0]));
    assert!(plan.saw_cuts_mm.contains(&[635.0,640.0]));
}

#[test]
fn beam_parts_are_eligible_but_different_sections_are_not_interchangeable() {
    let (mut request,profile,blocks) = fixture(&[(32_000,false,true),(32_000,true,false)]);
    let opening = request.opening_volumes.remove(0);
    request.beams.push(serde_json::from_value(json!({"guid":"beam","startXmm":opening.start_xmm,
        "endXmm":opening.end_xmm,"startYmm":opening.start_ymm,"endYmm":opening.end_ymm,
        "startZmm":0,"endZmm":0,"geometry":{"widthMm":193,"heightMm":63,"heightDirectionZ":1}})).unwrap());
    assert_eq!(prepare(&request,&profile,&blocks).unwrap().parents.len(),1);
    request.wall_volumes[1].thickness_mm = 160.0;
    assert!(prepare(&request,&profile,&blocks).unwrap().parents.is_empty());
}

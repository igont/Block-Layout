use fb_layout::{
    api::LayoutRequest,
    code_result::export_codes,
    exchange,
    layout::{Block, Profile},
    offcut_plan::prepare,
};
use serde_json::{json, Value};

fn fixture(specs: &[(i64, bool, bool)]) -> (LayoutRequest, Profile, Vec<Block>) {
    let profile: Profile =
        serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let mut walls = Vec::new();
    let mut openings = Vec::new();
    let mut blocks = Vec::new();
    for (i, &(length, hide_left, hide_right)) in specs.iter().enumerate() {
        let y = i as i64 * 1000;
        let z = i as i64 * 630;
        walls.push(
            json!({"guid":format!("wall-{i}"),"startXmm":-1000,"endXmm":2000,
            "startYmm":y,"endYmm":y,"startBottomZmm":0,"endBottomZmm":0,
            "startTopZmm":10000,"endTopZmm":10000,"thicknessMm":193,"purposeType":1}),
        );
        for (side, start, end) in [
            ("left", -320, 0),
            ("right", length / 100, length / 100 + 320),
        ] {
            if (side == "left" && hide_left) || (side == "right" && hide_right) {
                openings.push(
                    json!({"guid":format!("opening-{i}-{side}"),"startXmm":start,"endXmm":end,
                    "startYmm":y,"endYmm":y,"startBottomZmm":z,"endBottomZmm":z,
                    "startTopZmm":z+63,"endTopZmm":z+63,"thicknessMm":193,"openingType":"WINDOW"}),
                );
            }
        }
        blocks.push(
            serde_json::from_value(
                json!({"id":format!("block-{i}"),"wall_id":format!("wall-{i}"),
            "edge_id":format!("run-{i}"),"course_index":i*10,"z_centimm":z*100,
            "start":{"x":0,"y":y*100},"end":{"x":length,"y":y*100},"length_centimm":length,
            "kind":"ordinary","product_key":null,"catalog_status":"length_only","rotation_deg":0,
            "local_origin":null,"local_rotation_deg":null,"is_bridge":false,
            "hide_spikes_left":hide_left,"hide_spikes_right":hide_right,
            "natural_end_left":true,"natural_end_right":true,"cuts":[],
            "source_ids":[format!("wall:wall-{i}")],"catalog_nominal_centimm":length}),
            )
            .unwrap(),
        );
    }
    let request = serde_json::from_value(json!({"schema_version":1,"request_id":"offcut-test",
        "snapshot_hash":"a".repeat(64),"z0_mm":0,"wall_volumes":walls,"opening_volumes":openings}))
    .unwrap();
    let blocks = fb_layout::layout::finalize_parts(&request, &profile, &blocks).unwrap();
    (request, profile, blocks)
}

fn codes(request: &LayoutRequest, profile: &Profile, blocks: &[Block]) -> Value {
    let standard = exchange::from_layout_request(request, profile).unwrap();
    let final_blocks = fb_layout::layout::finalize_parts(request, profile, blocks).unwrap();
    export_codes(&standard, request, profile, &final_blocks, &[]).unwrap()
}

#[test]
fn same_profile_and_a_missing_saw_allowance_never_create_a_dobor() {
    for specs in [
        [(32_000, false, true), (32_000, false, true)],
        [(32_300, false, true), (32_300, true, false)],
    ] {
        let (request, profile, blocks) = fixture(&specs);
        let result = codes(&request, &profile, &blocks);
        assert!(result["blocks"]
            .as_array()
            .unwrap()
            .iter()
            .all(|b| b["is_dobor"] == false));
    }
}

#[test]
fn unrelated_short_blocks_nodes_and_gable_shapes_are_excluded() {
    let (mut request, profile, blocks) = fixture(&[(32_000, false, true), (32_000, true, false)]);
    request.opening_volumes.clear();
    assert!(prepare(&request, &profile, &blocks)
        .unwrap()
        .parents
        .is_empty());
    let (mut request, profile, mut blocks) =
        fixture(&[(32_000, false, true), (32_000, true, false)]);
    blocks[0].kind = "node_L".into();
    blocks[0].product_key = Some("Type4".into());
    assert!(prepare(&request, &profile, &blocks)
        .unwrap()
        .parents
        .is_empty());
    blocks[0].kind = "ordinary".into();
    blocks[0].product_key = None;
    blocks[0].cuts.push("Type4:x1:y1:run-0".into());
    assert!(prepare(&request, &profile, &blocks)
        .unwrap()
        .parents
        .is_empty());
    blocks[0].cuts.clear();
    request.wall_volumes[0].start_top_zmm = 70.0;
    request.wall_volumes[0].end_top_zmm = 30.0;
    assert!(prepare(&request, &profile, &blocks)
        .unwrap()
        .parents
        .is_empty());
}

#[test]
fn parts_between_two_openings_do_not_break_the_single_offcut_side() {
    let (request, profile, blocks) = fixture(&[(32_000, true, true), (32_000, true, true)]);
    assert!(prepare(&request, &profile, &blocks)
        .unwrap()
        .parents
        .is_empty());
}

#[test]
fn one_opening_uses_one_side_and_its_own_opposite_course_first() {
    let specs = [
        (24_000, false, true),
        (32_000, true, false),
        (24_000, false, true),
        (32_000, true, false),
        (24_000, false, true),
        (32_000, true, false),
    ];
    let (mut request, profile, mut blocks) = fixture(&specs);
    for wall in &mut request.wall_volumes {
        wall.start_ymm = 0.0;
        wall.end_ymm = 0.0;
    }
    let mut opening = request.opening_volumes[0].clone();
    opening.guid = "shared-window".into();
    opening.start_xmm = 240.0;
    opening.end_xmm = 640.0;
    opening.start_ymm = 0.0;
    opening.end_ymm = 0.0;
    opening.start_bottom_zmm = 0.0;
    opening.end_bottom_zmm = 0.0;
    opening.start_top_zmm = 10000.0;
    opening.end_top_zmm = 10000.0;
    request.opening_volumes = vec![opening];
    for (i, block) in blocks.iter_mut().enumerate() {
        block.start.y = 0;
        block.end.y = 0;
        if i % 2 == 1 {
            block.start.x += 64_000;
            block.end.x += 64_000;
        }
        block.course_index = (i / 2) as i64;
    }
    let result = codes(&request, &profile, &blocks);
    for i in 0..3 {
        assert_eq!(result["blocks"][i * 2]["is_dobor"], false);
        assert_eq!(
            result["blocks"][i * 2 + 1]["cut_from_block_id"],
            format!("block-{}", i * 2)
        );
    }
    blocks.reverse();
    assert_eq!(codes(&request, &profile, &blocks), result);
    for block in &mut blocks {
        std::mem::swap(&mut block.start, &mut block.end);
        block.ends = block.ends.reversed(block.length_centimm);
        block.rotation_deg = 180;
    }
    let reversed_axes = codes(&request, &profile, &blocks);
    for i in 0..3 {
        assert_eq!(
            reversed_axes["blocks"][i * 2 + 1]["cut_from_block_id"],
            format!("block-{}", i * 2)
        );
    }
}


fn shared_fixture(specs: &[(i64, bool, bool)]) -> (LayoutRequest, Profile, Vec<Block>) {
    let (mut request, profile, mut blocks) = fixture(specs);
    let mut opening = request.opening_volumes[0].clone();
    opening.guid = "shared".into();
    opening.start_xmm = specs[0].0 as f64 / 100.0;
    opening.end_xmm = 640.0;
    opening.start_ymm = 0.0;
    opening.end_ymm = 0.0;
    opening.start_bottom_zmm = 0.0;
    opening.end_bottom_zmm = 0.0;
    opening.start_top_zmm = 10000.0;
    opening.end_top_zmm = 10000.0;
    request.opening_volumes = vec![opening];
    for (i, block) in blocks.iter_mut().enumerate() {
        request.wall_volumes[i].start_ymm = 0.0;
        request.wall_volumes[i].end_ymm = 0.0;
        block.start.y = 0;
        block.end.y = 0;
        if specs[i].1 {
            block.start.x += 64000;
            block.end.x += 64000;
        }
    }
    let blocks = fb_layout::layout::finalize_parts(&request, &profile, &blocks).unwrap();
    (request, profile, blocks)
}

#[test]
fn opposite_parts_of_each_opening_type_share_stock_without_moving_parts() {
    for opening_type in ["WINDOW", "DOOR", "OPENING", "BEAM"] {
        let (mut request, profile, blocks) = shared_fixture(&[(32000, false, true), (32000, true, false)]);
        if opening_type == "BEAM" {
            request.opening_volumes.clear();
            request.beams.push(serde_json::from_value(json!({"guid":"shared",
                "startXmm":320,"endXmm":640,"startYmm":0,"endYmm":0,
                "startZmm":0,"endZmm":1000,
                "geometry":{"widthMm":193,"heightMm":10000,"heightDirectionZ":1}})).unwrap());
            // Горизонтальная балка перекрывает оба венца своим сечением.
            request.beams[0].end_zmm = 0.0;
        } else {
            request.opening_volumes[0].opening_type = opening_type.into();
        }
        // The layout owner records the newly configured obstacle before classification.
        let blocks = fb_layout::layout::finalize_parts(&request, &profile, &blocks).unwrap();
        let prepared = prepare(&request, &profile, &blocks).unwrap();
        assert_eq!(prepared.parents.get("block-1").map(String::as_str), Some("block-0"), "{opening_type}");
        let plan = &prepared.stock_plans["block-0"];
        assert_eq!(plan.parts[0].interval_mm, [0.0, 315.0]);
        assert_eq!(plan.parts[1].interval_mm, [325.0, 640.0]);
        for (before, after) in blocks.iter().zip(&prepared.blocks) {
            assert_eq!(before.start, after.start);
            assert_eq!(before.end, after.end);
            assert_eq!(before.length_centimm, after.length_centimm);
        }
        request.wall_volumes[1].thickness_mm = 160.0;
        assert!(prepare(&request, &profile, &blocks).unwrap().parents.is_empty());
    }
}

#[test]
fn another_opening_and_the_same_side_cannot_supply_offcuts() {
    let (request, profile, blocks) = fixture(&[(32000, false, true), (32000, true, false)]);
    assert!(prepare(&request, &profile, &blocks).unwrap().parents.is_empty());
    let (request, profile, blocks) = shared_fixture(&[(24000, false, true), (24000, false, true)]);
    assert!(prepare(&request, &profile, &blocks).unwrap().parents.is_empty());
}

#[test]
fn inclined_opening_prefers_high_side_even_when_it_has_fewer_parts() {
    let (mut request, profile, blocks) = shared_fixture(&[
        (24000, false, true), (24000, false, true), (32000, true, false),
    ]);
    request.opening_volumes[0].start_top_zmm = 9000.0;
    let prepared = prepare(&request, &profile, &blocks).unwrap();
    assert_eq!(prepared.parents.get("block-2").map(String::as_str), Some("block-0"));
    let opening = &mut request.opening_volumes[0];
    std::mem::swap(&mut opening.start_xmm, &mut opening.end_xmm);
    std::mem::swap(&mut opening.start_top_zmm, &mut opening.end_top_zmm);
    assert_eq!(prepare(&request, &profile, &blocks).unwrap().parents, prepared.parents);
}

#[test]
fn stock_is_consumed_once_and_missing_saw_allowance_prevents_replacement() {
    let (request, profile, mut blocks) = shared_fixture(&[
        (32000, false, true), (32000, true, false), (32000, true, false),
    ]);
    let prepared = prepare(&request, &profile, &blocks).unwrap();
    assert_eq!(prepared.parents.len(), 1);
    assert_eq!(prepared.stock_plans.len(), 1);
    blocks.reverse();
    assert_eq!(prepare(&request, &profile, &blocks).unwrap().parents, prepared.parents);
    let (request, profile, blocks) = shared_fixture(&[(32300, false, true), (32300, true, false)]);
    assert!(prepare(&request, &profile, &blocks).unwrap().parents.is_empty());
}

#[test]
fn inclined_opening_without_short_parts_on_high_side_uses_low_side() {
    let (mut request, profile, blocks) = shared_fixture(&[(8000, false, true), (54000, true, false)]);
    request.opening_volumes[0].start_top_zmm = 9000.0;
    let prepared = prepare(&request, &profile, &blocks).unwrap();
    assert_eq!(prepared.parents.get("block-0").map(String::as_str), Some("block-1"));
}

#[test]
fn typed_long_stock_supplies_plain_reversed_offcut() {
    let (request, profile, mut blocks) = shared_fixture(&[(30600, false, true), (31500, true, false)]);
    blocks[0].ends = fb_layout::end_state::EndStates::from_flags(false, true, true, false);
    blocks[0].ends.record_cut(false, 30600, fb_layout::end_state::CutSource::Opening("shared".into())).unwrap();
    let donor = &mut blocks[1];
    donor.kind = "node_T".into(); donor.product_key = Some("Type8_1".into());
    donor.start.x = 96000; donor.end.x = 64500; donor.rotation_deg = 180;
    donor.ends = fb_layout::end_state::EndStates::from_flags(false, true, true, false);
    donor.cuts = vec!["Type8_1:p0:y1:run-1".into()];
    donor.ends.record_cut(false, 31500, fb_layout::end_state::CutSource::Opening("shared".into())).unwrap();
    let prepared = prepare(&request, &profile, &blocks).unwrap();
    assert_eq!(prepared.parents.get("block-0").map(String::as_str), Some("block-1"));
    let plan = &prepared.stock_plans["block-1"];
    assert_eq!(plan.stock_code, "П640 [В-НЧ-Тип 8.1]");
    assert_eq!(plan.parts[1].interval_mm, [334.0,640.0]);
    assert!(plan.parts[1].reversed);
    let result = codes(&request, &profile, &blocks);
    assert_eq!(result["blocks"][0]["is_dobor"], true);
    assert_eq!(result["blocks"][1]["product_type"], "Тип 8.1");
    let mut odd_receiver = blocks[0].clone();
    odd_receiver.id = "block-2".into();
    odd_receiver.course_index = 20;
    odd_receiver.z_centimm = 126000;
    blocks.push(odd_receiver);
    let odd = prepare(&request, &profile, &blocks).unwrap();
    assert_eq!(odd.dobor_groups.len(), 2);
    assert_eq!(odd.parents.len(), 1);
    assert!(!odd.parents.contains_key("block-2"));
    assert_eq!(serde_json::to_value(&odd.blocks).unwrap(), serde_json::to_value(&blocks).unwrap());
    // Перенос запила в остаток делает такой донор непригодным.
    blocks[1].cuts = vec!["Type6:p30000:y1:run-1".into()];
    blocks[1].product_key = Some("Type6".into());
    assert!(prepare(&request, &profile, &blocks).unwrap().parents.is_empty());
}

#[test]
fn odd_receiver_remains_on_the_stable_side_without_inventing_stock() {
    let (request, profile, blocks) = shared_fixture(&[
        (32000, false, true), (32000, true, false), (32000, true, false),
    ]);
    let before = serde_json::to_value(&blocks).unwrap();
    let prepared = prepare(&request, &profile, &blocks).unwrap();
    assert_eq!(serde_json::to_value(&prepared.blocks).unwrap(), before);
    assert_eq!(prepared.dobor_groups.len(), 2);
    assert_eq!(prepared.parents.len(), 1);
    assert_eq!(prepared.stock_plans.len(), 1);
    let result = codes(&request, &profile, &blocks);
    for index in [1, 2] {
        let block = &result["blocks"][index];
        assert_eq!(block["is_dobor"], true);
        assert_eq!(block["dobor_group"], json!({"source_id":"shared", "side":"positive"}));
    }
    assert!(result["blocks"][2]["cut_from_block_id"].is_null());
    assert!(result["blocks"][2].get("stock_cut_plan").is_none());
    assert_eq!(result["blocks"][0]["cut_zone_ids"], json!(["shared"]));
    let physical = fb_layout::materialize::materialize(&request, &blocks, &profile).unwrap();
    let unmatched = physical.physical.blocks.iter().find(|block| block.id == "block-2").unwrap();
    assert!(unmatched.is_dobor);
    assert!(unmatched.cut_from_block_id.is_none());
    assert_eq!(serde_json::to_value(&unmatched.dobor_group).unwrap(),
        json!({"source_id":"shared", "side":"positive"}));
}

#[test]
fn receiver_side_does_not_follow_changes_in_course_counts() {
    let (request, profile, blocks) = shared_fixture(&[
        (24000, false, true), (32000, true, false),
        (24000, false, true), (24000, false, true),
    ]);
    let prepared = prepare(&request, &profile, &blocks).unwrap();
    assert!(prepared.dobor_groups.contains_key("block-1"));
    assert!(!prepared.dobor_groups.contains_key("block-0"));
    let reduced = prepare(&request, &profile, &blocks[..2]).unwrap();
    assert_eq!(reduced.dobor_groups, prepared.dobor_groups);
}

#[test]
fn incompatible_sections_and_profiles_never_establish_membership() {
    let (mut request, profile, blocks) = shared_fixture(&[(32000, false, true), (32000, true, false)]);
    request.wall_volumes[1].thickness_mm = 160.0;
    assert!(prepare(&request, &profile, &blocks).unwrap().dobor_groups.is_empty());
    let (request, profile, blocks) = shared_fixture(&[(32300, false, true), (32300, true, false)]);
    assert!(prepare(&request, &profile, &blocks).unwrap().dobor_groups.is_empty());
}

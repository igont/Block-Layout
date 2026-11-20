use fb_layout::{
    api::LayoutRequest,
    code_result::export_blocks,
    exchange, inspect_request,
    layout::{finalize_parts, Block, Profile},
    materialize::materialize,
    solid_geometry::volume,
};
use serde_json::json;

fn fixture() -> (LayoutRequest, Profile) {
    let request = serde_json::from_value(json!({
        "schema_version": 1, "request_id": "rectangular-beam-cups",
        "snapshot_hash": "c".repeat(64), "z0_mm": 0,
        "wall_volumes": [{
            "guid": "wall", "startXmm": 0, "startYmm": 0, "endXmm": 2560, "endYmm": 0,
            "startBottomZmm": 0, "endBottomZmm": 0, "startTopZmm": 189, "endTopZmm": 189,
            "thicknessMm": 193, "purposeType": 1
        }]
    }))
    .unwrap();
    let profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    (request, profile)
}

fn add_beam(request: &mut LayoutRequest, x: f64, far_side_y: f64) {
    request.beams.push(
        serde_json::from_value(json!({
            "guid": "beam", "startXmm": x, "startYmm": -500, "startZmm": 52,
            "endXmm": x, "endYmm": far_side_y, "endZmm": 52,
            "geometry": {"widthMm": 160, "heightMm": 20, "heightDirectionZ": 1}
        }))
        .unwrap(),
    );
}

fn layout(request: &LayoutRequest, profile: &Profile) -> Vec<Block> {
    let result = inspect_request(request, profile);
    assert!(!result.blocks.is_empty(), "{:?}", result.diagnostics);
    result.blocks
}

fn wide_interior_block(blocks: &[Block]) -> &Block {
    blocks
        .iter()
        .find(|block| {
            block.course_index == 0
                && block.start.x >= 32_000
                && block.end.x <= 224_000
                && block.length_centimm >= 32_000
                && block.rotation_deg == 0
        })
        .unwrap()
}

#[test]
fn central_cup_preserves_layout_and_stock_code_and_removes_actual_volume() {
    let (mut request, profile) = fixture();
    let baseline = layout(&request, &profile);
    let blank = wide_interior_block(&baseline);
    let center = (blank.start.x + blank.end.x) as f64 / 200.0;
    add_beam(&mut request, center, -70.0);
    let actual = layout(&request, &profile);
    let machined = actual.iter().find(|block| block.id == blank.id).unwrap();
    assert_eq!(
        actual.iter().map(|block| &block.id).collect::<Vec<_>>(),
        baseline.iter().map(|block| &block.id).collect::<Vec<_>>()
    );
    assert_eq!(
        (&machined.start, &machined.end, &machined.ends),
        (&blank.start, &blank.end, &blank.ends)
    );
    let expected = json!([{"side": "right", "offset_mm": blank.length_centimm as f64 / 200.0 - 80.0,
        "bottom_mm": 52.0, "width_mm": 160.0, "height_mm": 11.0, "depth_mm": 26.5, "source_beam_id": "beam"}]);
    assert_eq!(
        serde_json::to_value(&machined.rectangular_cuts).unwrap(),
        expected
    );
    assert!(machined.obstacle_ends().is_empty());

    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let original_codes = export_blocks(&standard, &request, &profile, &baseline, &[]).unwrap();
    let codes = export_blocks(&standard, &request, &profile, &actual, &[]).unwrap();
    let entry = |value: &serde_json::Value| {
        value["blocks"]
            .as_array()
            .unwrap()
            .iter()
            .find(|block| block["id"] == blank.id)
            .unwrap()
            .clone()
    };
    let before = entry(&original_codes);
    let after = entry(&codes);
    for field in [
        "code1",
        "code2",
        "placement",
        "nominal_length_mm",
        "is_dobor",
        "node_cuts",
    ] {
        assert_eq!(before[field], after[field], "{field}");
    }
    assert_eq!(after["rectangular_cuts"], expected);
    assert_eq!(after["source_ids"], json!(["beam", "wall"]));

    let bodies_before = materialize(&request, &baseline, &profile).unwrap();
    let bodies_after = materialize(&request, &actual, &profile).unwrap();
    let part_volume = |value: &fb_layout::materialize::Materialized| {
        value
            .physical
            .blocks
            .iter()
            .find(|block| block.id == blank.id)
            .unwrap()
            .bodies
            .iter()
            .map(volume)
            .sum::<f64>()
    };
    assert!(
        (part_volume(&bodies_before) - part_volume(&bodies_after) - 160.0 * 11.0 * 26.5).abs()
            < 1e-5
    );
    let physical_exchange = bodies_after
        .exchange_blocks
        .iter()
        .find(|block| block["id"] == blank.id)
        .unwrap();
    assert_eq!(physical_exchange["rectangular_cuts"], expected);
}

#[test]
fn off_center_joint_splits_width_and_each_course_keeps_its_own_height() {
    let (mut request, profile) = fixture();
    let baseline = layout(&request, &profile);
    let before = wide_interior_block(&baseline);
    let joint = before.end.x;
    let after = baseline
        .iter()
        .find(|block| block.course_index == 0 && block.start.x == joint)
        .unwrap();
    add_beam(&mut request, joint as f64 / 100.0 + 20.0, 15.0);
    let actual = layout(&request, &profile);

    for (original, offset, width) in [
        (before, before.length_centimm - 6000, 6000),
        (after, 0, 10_000),
    ] {
        let block = actual.iter().find(|part| part.id == original.id).unwrap();
        assert_eq!(
            (&block.start, &block.end, &block.ends),
            (&original.start, &original.end, &original.ends)
        );
        let cut = &block.rectangular_cuts[0];
        assert_eq!(
            (cut.offset_centimm, cut.width_centimm, cut.depth_centimm),
            (offset, width, 11_150)
        );
        assert_eq!((cut.bottom_centimm, cut.height_centimm), (5200, 1100));
    }
    let next: Vec<_> = actual
        .iter()
        .filter(|block| block.course_index == 1)
        .flat_map(|block| &block.rectangular_cuts)
        .collect();
    assert_eq!(
        next.iter().map(|cut| cut.width_centimm).sum::<i64>(),
        16_000
    );
    assert!(next
        .iter()
        .all(|cut| cut.bottom_centimm == 0 && cut.height_centimm == 900));
    assert!(actual
        .iter()
        .filter(|block| block.course_index >= 2)
        .all(|block| block.rectangular_cuts.is_empty()));
}

#[test]
fn flipping_block_axis_flips_side_and_measures_from_new_origin() {
    let (mut request, profile) = fixture();
    let baseline = layout(&request, &profile);
    let part = wide_interior_block(&baseline);
    let mut flipped = part.clone();
    std::mem::swap(&mut flipped.start, &mut flipped.end);
    flipped.rotation_deg = 180;
    add_beam(&mut request, part.start.x as f64 / 100.0 + 100.0, 20.0);
    let blocks = finalize_parts(&request, &profile, &[flipped]).unwrap();
    let cut = &blocks[0].rectangular_cuts[0];
    assert_eq!(cut.side, fb_layout::rectangular_cut::Side::Left);
    assert_eq!(cut.offset_centimm, part.length_centimm - 18_000);
    assert_eq!(cut.width_centimm, 16_000);
    assert_eq!(cut.depth_centimm, 11_650);
}

#[test]
fn through_beam_retains_full_height_cut_and_creates_no_cup() {
    let (mut request, profile) = fixture();
    add_beam(&mut request, 1280.0, 200.0);
    let blocks = layout(&request, &profile);
    assert!(blocks.iter().all(|block| block.rectangular_cuts.is_empty()));
    let course = |index| {
        blocks
            .iter()
            .filter(move |block| block.course_index == index)
    };
    assert!(course(0).all(|block| block.end.x <= 120_000 || block.start.x >= 136_000));
    assert!(course(1).all(|block| block.end.x <= 120_000 || block.start.x >= 136_000));
    for index in [0, 1] {
        assert!(course(index).any(|block| {
            block.end.x == 120_000
                && block
                    .obstacle_ends()
                    .iter()
                    .any(|end| end.source_id == "beam:beam")
        }));
        assert!(course(index).any(|block| {
            block.start.x == 136_000
                && block
                    .obstacle_ends()
                    .iter()
                    .any(|end| end.source_id == "beam:beam")
        }));
    }
    assert!(course(2).any(|block| block.end.x > 120_000 && block.start.x < 136_000));
}

#[test]
fn normalized_beam_height_and_inverted_source_axes_produce_the_same_cups() {
    let (mut request, profile) = fixture();
    request.wall_volumes[0].start_top_zmm = 378.0;
    request.wall_volumes[0].end_top_zmm = 378.0;
    add_beam(&mut request, 1280.0, 0.0);
    let beam = &mut request.beams[0];
    beam.start_zmm = 0.0;
    beam.end_zmm = 0.0;
    beam.geometry.height_mm = 320.0;
    let forward = layout(&request, &profile);
    assert!(forward.iter().any(|block| block.course_index == 4
        && block
            .rectangular_cuts
            .iter()
            .any(|cut| cut.height_centimm == 6300)));
    assert!(forward
        .iter()
        .filter(|block| block.course_index == 5)
        .all(|block| block.rectangular_cuts.is_empty()));

    let beam = &mut request.beams[0];
    std::mem::swap(&mut beam.start_ymm, &mut beam.end_ymm);
    beam.start_zmm = 320.0;
    beam.end_zmm = 320.0;
    beam.geometry.height_direction_z = -1.0;
    assert_eq!(layout(&request, &profile), forward);
}

#[test]
fn beam_in_the_l_extension_is_measured_from_actual_stock_origin() {
    let (mut request, profile) = fixture();
    let mut stem = request.wall_volumes[0].clone();
    stem.guid = "stem".into();
    stem.end_xmm = 0.0;
    stem.end_ymm = -1280.0;
    request.wall_volumes.push(stem);
    let baseline = layout(&request, &profile);
    let corner = baseline
        .iter()
        .find(|block| block.course_index == 0 && block.kind == "node_L" && block.rotation_deg == 0)
        .unwrap();
    assert_eq!(corner.start.x, 0);
    add_beam(&mut request, -50.0, 80.0);
    let beam = &mut request.beams[0];
    beam.start_ymm = 500.0;
    beam.geometry.width_mm = 40.0;
    let actual = layout(&request, &profile);
    let machined = actual.iter().find(|block| block.id == corner.id).unwrap();
    assert_eq!((machined.start, machined.end), (corner.start, corner.end));
    let cut = json!([{"side":"left","offset_mm":26.5,"bottom_mm":52.0,"width_mm":40.0,
        "height_mm":11.0,"depth_mm":16.5,"source_beam_id":"beam"}]);
    assert_eq!(
        serde_json::to_value(&machined.rectangular_cuts).unwrap(),
        cut
    );
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let codes = export_blocks(&standard, &request, &profile, &actual, &[]).unwrap();
    let code = codes["blocks"]
        .as_array()
        .unwrap()
        .iter()
        .find(|part| part["id"] == corner.id)
        .unwrap();
    assert_eq!(code["placement"]["origin_mm"][0], -96.5);
    assert_eq!(code["rectangular_cuts"], cut);

    let physical_before = materialize(&request, &baseline, &profile).unwrap();
    let physical_after = materialize(&request, &actual, &profile).unwrap();
    let original = physical_before
        .physical
        .blocks
        .iter()
        .find(|part| part.id == corner.id)
        .unwrap();
    let finished = physical_after
        .physical
        .blocks
        .iter()
        .find(|part| part.id == corner.id)
        .unwrap();
    let before_volume = original.bodies.iter().map(volume).sum::<f64>();
    let after_volume = finished.bodies.iter().map(volume).sum::<f64>();
    assert!((before_volume - after_volume - 40.0 * 11.0 * 16.5).abs() < 1e-5);

    let beam = &mut request.beams[0];
    beam.start_ymm = -500.0;
    beam.end_ymm = -80.0;
    let empty_side = finalize_parts(&request, &profile, &[corner.clone()]).unwrap();
    assert_eq!(empty_side[0].id, corner.id);
    assert!(empty_side[0].rectangular_cuts.is_empty());
}

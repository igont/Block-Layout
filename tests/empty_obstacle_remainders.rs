use fb_layout::{
    api::LayoutRequest, code_result::export_codes, exchange,
    layout::{finalize_parts, Block, Profile}, materialize::materialize,
    solid_geometry::volume,
};
use serde_json::json;

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}

// The real SUP failure: courses between a beam endpoint and an opening contain
// only the five millimetres removed from the natural hidden opening-side end.
fn request(gap_mm: f64) -> LayoutRequest {
    request_with_height(gap_mm, 320.0)
}

fn request_with_height(gap_mm: f64, height_mm: f64) -> LayoutRequest {
    serde_json::from_value(json!({
        "schema_version":1,"request_id":"empty-obstacle-remainder","snapshot_hash":"a".repeat(64),"z0_mm":-252,
        "wall_volumes":[{"guid":"wall","startXmm":4800,"startYmm":13440,"endXmm":4800,"endYmm":11840,
            "startBottomZmm":-252,"endBottomZmm":-252,"startTopZmm":3717,"endTopZmm":3717,"thicknessMm":193,"purposeType":1}],
        "opening_volumes":[{"guid":"opening","openingType":"OPENING","isOutside":true,
            "startXmm":4800,"startYmm":13120,"endXmm":4800,"endYmm":12160,
            "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":2961,"endTopZmm":2961,"thicknessMm":193,"purposeType":2}],
        "beams":[{"guid":"beam","startXmm":4800,"startYmm":12160.0-gap_mm,"startZmm":2457,
            "endXmm":4800,"endYmm":9595,"endZmm":2457,
            "geometry":{"widthMm":160,"heightMm":height_mm,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}]
    })).unwrap()
}

fn in_gap(block: &Block) -> bool {
    block.start.x == 480000 && block.end.x == 480000
        && block.start.y >= 1215000 && block.end.y <= 1216000
        && (43..=48).contains(&block.course_index)
}

#[test]
fn real_beam_opening_gap_exports_without_six_empty_parts() {
    let request = request(5.0);
    let profile = profile();
    let candidate = fb_layout::inspect_request(&request, &profile);
    assert!(!candidate.blocks.is_empty(), "{:?}", candidate.diagnostics);
    assert_eq!(candidate.blocks.iter().filter(|block| in_gap(block)).count(), 0);
    // The positive neighbour on the opposite side of the opening remains in each affected course.
    for course in 43..=48 {
        assert!(candidate.blocks.iter().any(|block| block.course_index == course
            && block.start.x == 480000 && block.start.y >= 1312000 && block.length_centimm > 500));
    }
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let codes = export_codes(&standard, &request, &profile, &candidate.blocks, &candidate.diagnostics).unwrap();
    assert_eq!(codes["status"], "success");
    assert_eq!(codes["blocks"].as_array().unwrap().len(), candidate.blocks.len());
    let physical = materialize(&request, &candidate.blocks, &profile).unwrap();
    assert_eq!(physical.physical.blocks.len(), candidate.blocks.len());
    assert!(physical.physical.blocks.iter().all(|block| block.bodies.iter().map(volume).sum::<f64>() > 0.0));
}

#[test]
fn positive_body_one_quantum_above_spike_removal_is_preserved() {
    let profile = profile();
    // Normalized 320 and actual 315 stop at z2772, exactly below course48.
    // Actual 321 keeps its height and intersects the sixth course as well.
    for (height, courses) in [(315.0, 5), (320.0, 5), (321.0, 6)] {
        let request = request_with_height(5.01, height);
        let candidate = fb_layout::inspect_request(&request, &profile);
        let gap: Vec<_> = candidate.blocks.iter().filter(|block| in_gap(block)).cloned().collect();
        assert_eq!(gap.len(), courses, "beam height {height}");
        for course in 43..43 + courses as i64 {
            assert_eq!(gap.iter().filter(|block| block.course_index == course).count(), 1);
        }
        assert!(gap.iter().all(|block| block.length_centimm == 501
            && !block.natural_end_left() && block.natural_end_right()
            && block.hide_spikes_left() && block.hide_spikes_right()));
        let standard = exchange::from_layout_request(&request, &profile).unwrap();
        let codes = export_codes(&standard, &request, &profile, &candidate.blocks, &candidate.diagnostics).unwrap();
        assert_eq!(codes["status"], "success");
        let physical = materialize(&request, &gap, &profile).unwrap();
        assert_eq!(physical.physical.blocks.len(), courses);
        for block in &physical.physical.blocks {
            let material: f64 = block.bodies.iter().map(volume).sum();
            assert!((material - 0.01 * 193.0 * 63.0).abs() < 1e-3, "height {height}: {material}");
        }
        if courses == 5 {
            let above = candidate.blocks.iter().find(|block| block.course_index == 48
                && block.start.y == 1184000 && block.end.y == 1216000).unwrap();
            assert_eq!(above.length_centimm, 32000);
            assert!(!above.source_ids.iter().any(|id| id == "beam:beam"));
            let physical = materialize(&request, std::slice::from_ref(above), &profile).unwrap();
            let material: f64 = physical.physical.blocks[0].bodies.iter().map(volume).sum();
            assert!((material - 310.0 * 193.0 * 63.0).abs() < 1e-3, "{material}");
        }
    }
}

#[test]
fn empty_ordinary_end_remainders_vanish_but_artificial_ends_keep_short_material() {
    let request = request(5.0);
    let profile = profile();
    for (length, natural_left, natural_right, expected) in [
        (499, false, true, 0), (500, false, true, 0), (501, false, true, 1),
        (999, true, true, 0), (1000, true, true, 0), (1001, true, true, 1),
        (1, false, false, 1),
    ] {
        // Keep this independent unit part away from all request obstacles and wall endpoints.
        let part: Block = serde_json::from_value(json!({"id":"part","wall_id":"wall","edge_id":"run",
            "course_index":0,"z_centimm":0,"start":{"x":480000,"y":1320000},
            "end":{"x":480000,"y":1320000+length},"length_centimm":length,
            "kind":"ordinary","product_key":null,"catalog_status":"length_only","rotation_deg":90,
            "local_origin":null,"local_rotation_deg":null,"is_bridge":false,
            "hide_spikes_left":true,"hide_spikes_right":true,"natural_end_left":natural_left,
            "natural_end_right":natural_right,"cuts":[],"source_ids":["wall:wall"],"arms":[]})).unwrap();
        let result = finalize_parts(&request, &profile, &[part]).unwrap();
        assert_eq!(result.len(), expected, "length={length}, natural_left={natural_left}, natural_right={natural_right}");
    }
}

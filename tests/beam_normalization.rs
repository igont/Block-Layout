use fb_layout::{api::LayoutRequest, effective_geometry::{normalize_beams, beam_adjustments}, exchange,
    layout::Profile, code_result::export_codes, materialize::materialize};
use serde_json::json;

fn request(height: f64, direction: f64) -> LayoutRequest {
    let axis_z = if direction < 0.0 { height } else { 0.0 };
    serde_json::from_value(json!({"schema_version":1,"request_id":"beam-normalization",
        "snapshot_hash":"d".repeat(64),"z0_mm":0,
        "wall_volumes":[{"guid":"wall","startXmm":0,"startYmm":0,"endXmm":3200,"endYmm":0,
            "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":630,"endTopZmm":630,"thicknessMm":193,"purposeType":1}],
        "beams":[{"guid":"beam","startXmm":640,"startYmm":0,"startZmm":axis_z,
            "endXmm":1600,"endYmm":0,"endZmm":axis_z,
            "geometry":{"widthMm":193,"heightMm":height,"heightDirectionZ":direction}}]})).unwrap()
}

#[test]
fn horizontal_320_is_virtual_315_with_same_bottom_in_both_z_directions() {
    let profile: Profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    for direction in [1.0,-1.0] {
        let original = request(320.0,direction);
        let expected = request(315.0,direction);
        let mut effective = original.clone();
        let changes = normalize_beams(&mut effective);
        assert_eq!(changes.len(),1);
        assert_eq!(effective.beams[0].geometry.height_mm,315.0);
        assert_eq!(effective.beams[0].start_zmm,expected.beams[0].start_zmm);
        assert_eq!(changes[0]["bottom_start_z_mm"],0.0);
        assert_eq!(changes[0]["bottom_end_z_mm"],0.0);
        assert!(normalize_beams(&mut effective).is_empty());
        let actual = fb_layout::inspect_request(&original,&profile);
        let reference = fb_layout::inspect_request(&expected,&profile);
        assert!(!actual.blocks.is_empty(),"{:?}",actual.diagnostics);
        assert_eq!(serde_json::to_value(&actual.blocks).unwrap(),serde_json::to_value(&reference.blocks).unwrap());
        // The discarded upper 5 mm must not void the sixth 63 mm crown.
        assert!(actual.blocks.iter().any(|b| b.course_index==5 && b.start.x<160_000 && b.end.x>64_000));
        let standard = exchange::from_layout_request(&original,&profile).unwrap();
        let result = export_codes(&standard,&original,&profile,&actual.blocks,&actual.diagnostics).unwrap();
        assert_eq!(result["beam_adjustments"],json!(changes));
        let physical = materialize(&original,&actual.blocks,&profile).unwrap();
        let expected_physical = materialize(&expected,&reference.blocks,&profile).unwrap();
        assert_eq!(serde_json::to_value(&physical.physical).unwrap(),serde_json::to_value(&expected_physical.physical).unwrap());
        assert_eq!(original.beams[0].geometry.height_mm,320.0);
        assert_eq!(original.beams[0].start_zmm,if direction<0.0 {320.0} else {0.0});
        assert_eq!(standard.model.beams[0].height_mm,320.0);
    }
}

#[test]
fn other_heights_and_inclined_beams_keep_their_source_geometry() {
    for height in [200.0,315.0,321.0] {
        let mut raw = request(height,1.0);
        assert!(normalize_beams(&mut raw).is_empty());
        assert_eq!(raw.beams[0].geometry.height_mm,height);
    }
    let mut inclined = request(320.0,1.0);
    inclined.beams[0].end_zmm=63.0;
    assert!(normalize_beams(&mut inclined).is_empty());
    inclined.beams[0].end_zmm=0.0;
    inclined.beams[0].geometry.height_direction_y=0.6;
    inclined.beams[0].geometry.height_direction_z=0.8;
    assert!(beam_adjustments(&inclined).is_empty());
    assert_eq!(inclined.beams[0].geometry.height_mm,320.0);
}

#[test]
fn centimm_height_matching_preserves_exact_source_and_bottom() {
    let original = request(319.999,-1.0);
    let mut effective = original.clone();
    let changes = normalize_beams(&mut effective);
    assert_eq!(changes.len(),1);
    assert_eq!(changes[0]["original_height_mm"],319.999);
    assert_eq!(changes[0]["bottom_start_z_mm"],0.0);
    assert_eq!(effective.beams[0].start_zmm,315.0);
    assert_eq!(original.beams[0].start_zmm,319.999);
    for height in [319.994,320.005,320.006] {
        let mut excluded = request(height,-1.0);
        assert!(normalize_beams(&mut excluded).is_empty(),"height={height}");
        assert_eq!(excluded.beams[0].start_zmm,height);
        assert_eq!(excluded.beams[0].geometry.height_mm,height);
    }
}

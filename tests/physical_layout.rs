use fb_layout::{
    api::LayoutRequest, layout::Profile, materialize::materialize, solid_geometry::volume,
};
use serde_json::json;

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}
fn request(length: f64, beams: serde_json::Value) -> LayoutRequest {
    serde_json::from_value(json!({"schema_version":1,"request_id":"physical-test","snapshot_hash":"a".repeat(64),"z0_mm":0,
        "wall_volumes":[{"guid":"wall","startXmm":0,"startYmm":0,"endXmm":length,"endYmm":0,"startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],"beams":beams})).unwrap()
}

#[test]
fn ordinary_cut_preserves_nominal_stock_and_finishes_at_wall_end() {
    let r = request(440.0, json!([]));
    let result = fb_layout::inspect_request(&r, &profile());
    assert!(result.diagnostics.is_empty());
    assert_eq!(result.blocks.len(), 1);
    let rendered = materialize(&r, &result.blocks).unwrap();
    let body = &rendered.physical.blocks[0];
    let actual: f64 = body.bodies.iter().map(volume).sum();
    assert!((actual - 440.0 * 193.0 * 63.0).abs() < 1e-4);
    assert_eq!(
        rendered.exchange_blocks[0]["product"]["nominal_size_mm"][0],
        640.0
    );
    assert_eq!(rendered.exchange_blocks[0]["cuts"][0]["kind"], "plane_cut");
    assert!(body
        .bodies
        .iter()
        .flat_map(|m| &m.vertices)
        .all(|p| p[0] >= -0.01 && p[0] <= 440.01));
}

#[test]
fn transverse_beam_keeps_twenty_mm_skin_without_its_own_lintel() {
    let r = request(
        640.0,
        json!([{"guid":"beam","startXmm":320,"startYmm":-500,"startZmm":0,"endXmm":320,"endYmm":76.5,"endZmm":0,
        "geometry":{"widthMm":160,"heightMm":320,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}]),
    );
    let result = fb_layout::inspect_request(&r, &profile());
    assert!(!result.blocks.is_empty());
    assert!(result.blocks.iter().all(|b| !b.is_bridge));
    let rendered = materialize(&r, &result.blocks).unwrap();
    let actual: f64 = rendered
        .physical
        .blocks
        .iter()
        .flat_map(|b| &b.bodies)
        .map(volume)
        .sum();
    let expected = 640.0 * 193.0 * 63.0 - 160.0 * 173.0 * 63.0;
    assert!((actual - expected).abs() < 1e-4, "{actual} != {expected}");
    assert!(rendered
        .physical
        .blocks
        .iter()
        .flat_map(|b| &b.bodies)
        .flat_map(|m| &m.vertices)
        .any(|p| (p[1] - 76.5).abs() < 1e-6));
    assert_eq!(
        rendered.exchange_blocks[0]["cuts"][0]["source_ids"][0],
        "beam"
    );
}

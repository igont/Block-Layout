use fb_layout::{api::LayoutRequest, code_result::export_codes, exchange, layout::{Block, Profile}, materialize::materialize, solid_geometry::volume};
use serde_json::json;

fn inputs(rotation: u16) -> (LayoutRequest, Profile, Block) {
    let profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let (x, y) = match rotation { 0 => (640, 0), 90 => (0, 640), 180 => (-640, 0), _ => (0, -640) };
    let request = serde_json::from_value(json!({"schema_version":1,"request_id":"end-states","snapshot_hash":"a".repeat(64),"z0_mm":0,
        "wall_volumes":[{"guid":"wall","startXmm":-x/640*100,"startYmm":-y/640*100,"endXmm":x/640*740,"endYmm":y/640*740,
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],"beams":[]})).unwrap();
    let block = serde_json::from_value(json!({"id":"part","wall_id":"wall","edge_id":"run","course_index":0,"z_centimm":0,
        "start":{"x":0,"y":0},"end":{"x":x*100,"y":y*100},"length_centimm":64000,"kind":"ordinary",
        "product_key":null,"catalog_status":"KnownPattern","rotation_deg":rotation,"local_origin":null,"local_rotation_deg":null,
        "is_bridge":false,"hide_spikes_left":true,"hide_spikes_right":false,"cuts":[],"source_ids":["wall:wall"],
        "catalog_nominal_centimm":72000,"arms":[]})).unwrap();
    (request, profile, block)
}

#[test]
fn only_natural_hidden_end_loses_five_mm_in_each_orientation() {
    for rotation in [0, 90, 180, 270] {
        let (request, profile, mut block) = inputs(rotation);
        for (natural, expected) in [(true, 635.0), (false, 640.0)] {
            block.natural_end_left = natural;
            let rendered = materialize(&request, &[block.clone()], &profile).unwrap();
            let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
            assert!((actual - expected * 193.0 * 63.0).abs() < 1e-4, "rotation={rotation}, natural={natural}");
            let standard = exchange::from_layout_request(&request, &profile).unwrap();
            let result = export_codes(&standard, &request, &profile, &[block.clone()], &[]).unwrap();
            assert_eq!(result["blocks"][0]["code1"], "П640");
            assert_eq!(result["blocks"][0]["code2"], "П640");
            assert_eq!(result["blocks"][0]["length_mm"], 640.0);
            assert_eq!(result["blocks"][0]["nominal_length_mm"], 640.0);
            assert_eq!(result["blocks"][0]["natural_end_left"], natural);
            assert_eq!(result["blocks"][0]["hide_spikes_left"], true);
            assert_eq!(result["beam_adjustments"], json!([]));
        }
    }
}

#[test]
fn artificial_end_with_spikes_is_rejected_by_both_exports() {
    let (request, profile, mut block) = inputs(0);
    block.natural_end_left = false;
    block.hide_spikes_left = false;
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    assert!(export_codes(&standard, &request, &profile, &[block.clone()], &[]).is_err());
    assert!(materialize(&request, &[block], &profile).is_err());
}

#[test]
fn legacy_end_origin_defaults_to_natural() {
    let (_, _, block) = inputs(0);
    assert!(block.natural_end_left && block.natural_end_right);
}

#[test]
fn ordinary_finished_part_keeps_its_production_profile() {
    let (request, profile, mut block) = inputs(0);
    block.hide_spikes_left = false;
    block.cuts = vec!["Type6:p32000:y1:run".into()];
    let rendered = materialize(&request, &[block.clone()], &profile).unwrap();
    let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
    assert!(actual < 640.0 * 193.0 * 63.0 - 1.0);
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let result = export_codes(&standard, &request, &profile, &[block], &[]).unwrap();
    assert_eq!(result["blocks"][0]["code1"], "П640 [Н-320-Тип 6]");
    assert_eq!(result["blocks"][0]["code2"], "П640 [В-320-Тип 6]");
}

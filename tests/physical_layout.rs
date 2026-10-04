use fb_layout::{
    api::LayoutRequest, layout::Profile, materialize::materialize as render_final_parts, solid_geometry::volume,
};
use serde_json::json;

// Fixtures describe raw stock; only the geometry owner finalizes it before rendering.
fn materialize(request: &LayoutRequest, blocks: &[fb_layout::layout::Block], profile: &Profile) -> Result<fb_layout::materialize::Materialized, fb_layout::api::ApiFailure> {
    let finished = fb_layout::layout::finalize_parts(request, profile, blocks)?;
    render_final_parts(request, &finished, profile)
}

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}
fn request(length: f64, beams: serde_json::Value) -> LayoutRequest {
    serde_json::from_value(json!({"schema_version":1,"request_id":"physical-test","snapshot_hash":"a".repeat(64),"z0_mm":0,
        "wall_volumes":[{"guid":"wall","startXmm":0,"startYmm":0,"endXmm":length,"endYmm":0,"startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],"beams":beams})).unwrap()
}

fn block(length_mm: i64) -> fb_layout::layout::Block {
    serde_json::from_value(json!({"id":"physical-part","wall_id":"wall","edge_id":"run","course_index":0,"z_centimm":0,
        "start":{"x":0,"y":0},"end":{"x":length_mm*100,"y":0},"length_centimm":length_mm*100,
        "kind":"ordinary","product_key":null,"catalog_status":"KnownPattern","rotation_deg":0,"local_origin":null,
        "local_rotation_deg":null,"is_bridge":false,"hide_spikes_left":false,"hide_spikes_right":false,
        "cuts":[],"source_ids":["wall:wall"],"catalog_nominal_centimm":64000,"arms":[]})).unwrap()
}

#[test]
fn ordinary_cut_exports_finished_stock_without_hidden_geometry() {
    let r = request(440.0, json!([]));
    let mut part = block(440);
    part.ends = fb_layout::end_state::EndStates::from_flags(part.hide_spikes_left(), part.hide_spikes_right(), part.natural_end_left(), false);
    part.ends = fb_layout::end_state::EndStates::from_flags(part.hide_spikes_left(), true, part.natural_end_left(), part.natural_end_right());
    let rendered = materialize(&r, &[part], &profile()).unwrap();
    let body = &rendered.physical.blocks[0];
    let actual: f64 = body.bodies.iter().map(volume).sum();
    assert!((actual - 435.0 * 193.0 * 63.0).abs() < 1e-4);
    assert_eq!(
        rendered.exchange_blocks[0]["product"]["nominal_size_mm"][0],
        440.0
    );
    assert_eq!(rendered.exchange_blocks[0]["cuts"][0]["kind"], "plane_cut");
    assert!(body
        .bodies
        .iter()
        .flat_map(|m| &m.vertices)
        .all(|p| p[0] >= -0.01 && p[0] <= 440.01));
}

#[test]
fn transverse_beam_removes_full_wall_thickness_without_its_own_lintel() {
    let r = request(
        640.0,
        json!([{"guid":"beam","startXmm":320,"startYmm":-500,"startZmm":0,"endXmm":320,"endYmm":76.5,"endZmm":0,
        "geometry":{"widthMm":160,"heightMm":320,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}]),
    );
    let rendered = materialize(&r, &[block(640)], &profile()).unwrap();
    let actual: f64 = rendered
        .physical
        .blocks
        .iter()
        .flat_map(|b| &b.bodies)
        .map(volume)
        .sum();
    let expected = (640.0 - 160.0 - 10.0) * 193.0 * 63.0;
    assert!((actual - expected).abs() < 1e-4, "{actual} != {expected}");
    assert!(!rendered
        .physical
        .blocks
        .iter()
        .flat_map(|b| &b.bodies)
        .flat_map(|m| &m.vertices)
        .any(|p| (p[1] - 76.5).abs() < 1e-6));
    assert_eq!(rendered.physical.blocks.len(), 2);
    assert_eq!(rendered.exchange_blocks[0]["natural_end_right"], false);
    assert_eq!(rendered.exchange_blocks[1]["natural_end_left"], false);
}

fn opening(kind: &str, from: f64, to: f64) -> fb_layout::api::Volume {
    serde_json::from_value(json!({"guid":"opening","openingType":kind,
        "startXmm":from,"startYmm":0,"endXmm":to,"endYmm":0,
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63}))
    .unwrap()
}

#[test]
fn all_opening_types_cut_ordinary_special_and_glued_parts() {
    let mut r = request(640.0, json!([]));
    let block = block(640);
    for kind in ["OPENING", "CONSOLE", "WINDOW", "DOOR"] {
        r.opening_volumes = vec![opening(kind, 160.0, 320.0)];
        for detail in ["ordinary", "special", "glued"] {
            let mut candidate = block.clone();
            if detail == "special" {
                candidate.kind = "special".into();
            } else if detail == "glued" {
                candidate.components = vec![block.clone()];
                candidate.is_bridge = true;
            }
            let rendered = materialize(&r, &[candidate], &profile()).unwrap();
            let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
            assert!(
                (actual - 470.0 * 193.0 * 63.0).abs() < 1e-4,
                "{kind}/{detail}: {actual}"
            );
            assert_eq!(rendered.physical.blocks.len(), 2);
            assert_eq!(rendered.exchange_blocks[0]["natural_end_right"], false);
            assert_eq!(rendered.exchange_blocks[1]["natural_end_left"], false);
        }
    }
}

#[test]
fn opening_cut_at_one_end_preserves_other_end_and_exact_length() {
    let mut r = request(640.0, json!([]));
    let block = block(640);
    for (from, to, _left, _right) in [(0.0, 320.0, true, false), (320.0, 640.0, false, true)] {
        r.opening_volumes = vec![opening("WINDOW", from, to)];
        let rendered = materialize(&r, &[block.clone()], &profile()).unwrap();
        let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
        assert!((actual - 315.0 * 193.0 * 63.0).abs() < 1e-4);
        assert_eq!(
            rendered.exchange_blocks[0]["spikes_removed"],
            json!({"left":true,"right":true})
        );
    }
}

#[test]
fn inclined_beam_axis_or_height_direction_does_not_cut_parts() {
    let block = block(640);
    for (end_z, height_x) in [(20.0, 0.0), (0.0, 0.1)] {
        let r = request(
            640.0,
            json!([{"guid":"beam","startXmm":320,"startYmm":-500,"startZmm":0,
            "endXmm":320,"endYmm":500,"endZmm":end_z,
            "geometry":{"widthMm":160,"heightMm":63,"heightDirectionX":height_x,"heightDirectionY":0,"heightDirectionZ":1}}]),
        );
        let rendered = materialize(&r, &[block.clone()], &profile()).unwrap();
        let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
        assert!((actual - 630.0 * 193.0 * 63.0).abs() < 1e-4);
        assert_eq!(rendered.physical.blocks.len(), 1);
    }
}

#[test]
fn downward_vertical_beam_height_matches_upward_physical_extents() {
    let mut r = request(640.0, json!([]));
    let block = block(640);
    for (axis_z, direction_z) in [(0.0, 1.0), (315.0, -1.0)] {
        for transverse in [false, true] {
            let (sx, sy, ex, ey) = if transverse {
                (320.0, -500.0, 320.0, 500.0)
            } else {
                (320.0, 0.0, 640.0, 0.0)
            };
            r.beams = request(640.0, json!([{"guid":"beam","startXmm":sx,"startYmm":sy,"startZmm":axis_z,
                "endXmm":ex,"endYmm":ey,"endZmm":axis_z,
                "geometry":{"widthMm":193,"heightMm":315,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":direction_z}}])).beams;
            let rendered = materialize(&r, &[block.clone()], &profile()).unwrap();
            let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
            let remaining_length = if transverse { 640.0 - 193.0 - 10.0 } else { 315.0 };
            assert!(
                (actual - remaining_length * 193.0 * 63.0).abs() < 1e-4,
                "axis_z={axis_z}, direction_z={direction_z}, transverse={transverse}: {actual}"
            );
            assert!(rendered.exchange_blocks.iter().all(|b| b["cuts"].as_array().unwrap().iter().all(|c| c["kind"] != "box_cut")));
        }
    }
}

#[test]
fn opening_cuts_node_profile_and_removes_fully_consumed_components() {
    let mut r = request(640.0, json!([]));
    let mut node = block(640);
    node.kind = "node_corner".into();
    node.product_key = Some("Type1".into());
    node.cuts = vec!["Type1:x1:y1:w".into()];
    let original = materialize(&r, &[node.clone()], &profile()).unwrap();
    let before: f64 = original.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
    r.opening_volumes = vec![opening("DOOR", 160.0, 320.0)];
    let rendered = materialize(&r, &[node.clone()], &profile()).unwrap();
    let after: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
    assert!(after < before - 1.0);
    assert!(rendered.physical.blocks.iter().flat_map(|b| &b.bodies).all(|m| {
        let min = m
            .vertices
            .iter()
            .map(|p| p[0])
            .fold(f64::INFINITY, f64::min);
        let max = m
            .vertices
            .iter()
            .map(|p| p[0])
            .fold(f64::NEG_INFINITY, f64::max);
        max <= 160.000001 || min >= 319.999999
    }));
    r.opening_volumes = vec![opening("WINDOW", -1000.0, 1000.0)];
    let mut glued = node.clone();
    glued.components = vec![node.clone()];
    let rendered = materialize(&r, &[node, glued], &profile()).unwrap();
    assert!(rendered.physical.blocks.is_empty());
    assert!(rendered.exchange_blocks.is_empty());
}

#[test]
fn longitudinal_beam_cuts_exact_body_without_extra_five_mm() {
    let mut r = request(640.0, json!([]));
    let block = block(640);
    r.beams = request(640.0, json!([{"guid":"beam","startXmm":320,"startYmm":0,"startZmm":0,
        "endXmm":640,"endYmm":0,"endZmm":0,
        "geometry":{"widthMm":193,"heightMm":63,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}])).beams;
    let mut glued = block.clone();
    glued.components = vec![block.clone()];
    let rendered = materialize(&r, &[block, glued], &profile()).unwrap();
    for (physical, exchange) in rendered
        .physical
        .blocks
        .iter()
        .zip(&rendered.exchange_blocks)
    {
        let actual: f64 = physical.bodies.iter().map(volume).sum();
        assert!((actual - 315.0 * 193.0 * 63.0).abs() < 1e-4);
        assert_eq!(
            exchange["spikes_removed"],
            json!({"left":true,"right":true})
        );
        assert!(physical.cut);
    }
}

#[test]
fn nominal_beam_contact_keeps_natural_end_origin_without_beam_extension() {
    let mut r = request(640.0, json!([]));
    let block = block(640);
    r.beams = request(640.0, json!([{"guid":"beam","startXmm":-640,"startYmm":0,"startZmm":0,
        "endXmm":0,"endYmm":0,"endZmm":0,"geometry":{"widthMm":193,"heightMm":63,"heightDirectionZ":1}}])).beams;
    let rendered = materialize(&r, &[block], &profile()).unwrap();
    let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
    assert!((actual - 630.0 * 193.0 * 63.0).abs() < 1e-4);
    assert_eq!(rendered.exchange_blocks[0]["natural_end_left"], true);
    assert_eq!(r.beams[0].end_xmm, 0.0);
}

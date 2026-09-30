use fb_layout::api::{ApiFailure, LayoutRequest};
use fb_layout::exchange::{
    failure_result, from_layout_request, parse_request, success_result,
    success_result_with_warnings,
};
use fb_layout::layout::Profile;
use serde_json::{json, Value};

const REQUEST: &str =
    include_str!("../Документация/Граничные контракты/examples/layout-request.v1.json");
const BEAM_REQUEST: &str =
    include_str!("../Документация/Граничные контракты/examples/model-with-beam.v1.json");

fn mutated(change: impl FnOnce(&mut Value)) -> String {
    let mut value: Value = serde_json::from_str(REQUEST).unwrap();
    change(&mut value);
    value.to_string()
}

#[test]
fn opening_kinds_and_every_source_volume_survive_neutral_conversion() {
    let input = mutated(|value| {
        let volume = value["model"]["wall_volumes"][0]["volume"].clone();
        value["model"]["openings"] =
            json!(
                ["opening", "window", "door", "console", "partition_opening"]
                    .iter()
                    .map(|purpose| json!({"id":purpose,"purpose":purpose,"volume":volume}))
                    .collect::<Vec<_>>()
            );
    });
    let neutral = parse_request(&input).unwrap();
    let internal = neutral.to_layout_request().unwrap();
    assert_eq!(internal.opening_volumes.len(), 5);
    for (opening, expected) in internal.opening_volumes.iter().zip([
        "OPENING",
        "WINDOW",
        "DOOR",
        "CONSOLE",
        "PARTITION_OPENING",
    ]) {
        assert_eq!(opening.opening_type, expected);
        assert_eq!(opening.guid, expected.to_lowercase());
        assert_eq!(opening.start_xmm, internal.wall_volumes[0].start_xmm);
        assert_eq!(opening.end_xmm, internal.wall_volumes[0].end_xmm);
        assert_eq!(
            opening.start_bottom_zmm,
            internal.wall_volumes[0].start_bottom_zmm
        );
        assert_eq!(opening.end_top_zmm, internal.wall_volumes[0].end_top_zmm);
        assert_eq!(opening.thickness_mm, 193.0);
    }
    let back = from_layout_request(
        &internal,
        &serde_json::from_str::<Profile>(include_str!("../profiles/banya-prototype.json")).unwrap(),
    )
    .unwrap();
    assert_eq!(
        serde_json::to_value(back.model.openings).unwrap(),
        serde_json::to_value(neutral.model.openings).unwrap()
    );
}

#[test]
fn neutral_input_preserves_source_geometry_without_application_metadata() {
    let request = parse_request(REQUEST).unwrap();
    let internal = request.to_layout_request().unwrap();
    assert_eq!(internal.request_id, "example-single-course");
    assert_eq!(internal.z0_mm, -252.0);
    assert_eq!(internal.wall_volumes[0].guid, "wall-example");
    assert_eq!(internal.wall_volumes[0].end_xmm, 440.0);
    assert_eq!(internal.wall_volumes[0].thickness_mm, 193.0);
    assert!(internal.project_id.is_none());
    assert_eq!(internal.metadata, Value::Null);
    let input =
        mutated(|v| v["model"]["wall_volumes"][0]["volume"]["end_xy_mm"][0] = json!(440.001));
    assert_eq!(
        parse_request(&input)
            .unwrap()
            .to_layout_request()
            .unwrap()
            .wall_volumes[0]
            .end_xmm,
        440.001
    );
}

#[test]
fn closed_input_rejects_unknown_geometry_and_result_documents() {
    for input in [
        mutated(|v| v["model"]["wall_volumes"][0]["volume"]["curve"] = json!(true)),
        mutated(|v| v["model"]["wall_volumes"][0]["unexpected"] = json!(1)),
        mutated(|v| v["model"]["hidden_blocks"] = json!([])),
        mutated(|v| v["ignored"] = json!(true)),
        include_str!("../Документация/Граничные контракты/examples/layout-result.v1.json").into(),
        include_str!("../Документация/Граничные контракты/examples/layout-failure.v1.json").into(),
    ] {
        assert_eq!(parse_request(&input).unwrap_err().code, "INVALID_EXCHANGE");
    }
}

#[test]
fn units_quantum_scope_hash_and_source_identity_are_checked() {
    let cases: Vec<(String, &str)> = vec![
        (
            mutated(|v| v["coordinate_system"]["units"] = json!("m")),
            "UNSUPPORTED_COORDINATES",
        ),
        (
            mutated(|v| v["coordinate_system"]["coordinate_quantum_mm"] = json!(1)),
            "UNSUPPORTED_COORDINATES",
        ),
        (
            mutated(|v| v["scope"] = json!({"mode":"selected","wall_ids":["wall-example"]})),
            "UNSUPPORTED_SCOPE",
        ),
        (
            mutated(|v| v["scope"]["wall_ids"] = json!(["wall-example"])),
            "INVALID_SCOPE",
        ),
        (
            mutated(|v| v["snapshot_hash"] = json!("snapshot")),
            "INVALID_EXCHANGE",
        ),
        (
            mutated(|v| {
                let wall = v["model"]["wall_volumes"][0].clone();
                v["model"]["wall_volumes"]
                    .as_array_mut()
                    .unwrap()
                    .push(wall);
            }),
            "DUPLICATE_SOURCE_ID",
        ),
        (
            mutated(|v| v["model"]["wall_volumes"][0]["volume"]["left_thickness_mm"] = json!(96)),
            "UNSUPPORTED_GEOMETRY",
        ),
        (
            mutated(|v| v["model"]["wall_volumes"][0]["volume"]["top_start_mm"] = json!(-252)),
            "INVALID_GEOMETRY",
        ),
        (
            mutated(|v| v["model"]["wall_volumes"][0]["purpose"] = json!("context")),
            "UNSUPPORTED_WALL_PURPOSE",
        ),
    ];
    for (input, code) in cases {
        assert_eq!(parse_request(&input).unwrap_err().code, code);
    }
}

#[test]
fn opening_binding_and_wall_trim_refuse_instead_of_silent_loss() {
    assert_eq!(
        parse_request(BEAM_REQUEST).unwrap_err().code,
        "UNSUPPORTED_OPENING_BINDING"
    );
    let mut value: Value = serde_json::from_str(BEAM_REQUEST).unwrap();
    value["model"]["openings"][0]
        .as_object_mut()
        .unwrap()
        .remove("wall_ids");
    assert!(parse_request(&value.to_string()).is_ok());
    value["model"]["openings"][0]["purpose"] = json!("wall_trim");
    assert_eq!(
        parse_request(&value.to_string()).unwrap_err().code,
        "UNSUPPORTED_OPENING_PURPOSE"
    );
}

#[test]
fn beam_basis_preserves_real_ends_and_rejects_nonorthogonal_height() {
    let mut value: Value = serde_json::from_str(BEAM_REQUEST).unwrap();
    value["model"]["openings"] = json!([]);
    let request = parse_request(&value.to_string())
        .unwrap()
        .to_layout_request()
        .unwrap();
    assert_eq!(request.beams[0].end_ymm, 76.5);
    assert_eq!(request.beams[0].geometry.height_direction_z, 1.0);
    value["model"]["beams"][0]["height_direction"] = json!([0, 1, 0]);
    assert_eq!(
        parse_request(&value.to_string()).unwrap_err().code,
        "INVALID_BEAM_FRAME"
    );
    value["model"]["beams"][0]["height_direction"] = json!([0, 0, 2]);
    assert_eq!(
        parse_request(&value.to_string()).unwrap_err().code,
        "INVALID_BEAM_FRAME"
    );
}

#[test]
fn output_envelope_preserves_identity_and_failure_has_no_partial_blocks() {
    let request = parse_request(REQUEST).unwrap();
    let result = success_result(&request, vec![], vec![]);
    assert_eq!(result["kind"], "layout_result");
    assert_eq!(result["snapshot_hash"], request.snapshot_hash);
    assert_eq!(result["coordinate_system"]["id"], "example-world");
    assert!(result.get("model").is_none());
    assert!(result.get("diagnostics").is_none());
    let mut diagnostic = ApiFailure::new(
        "UNSUPPORTED_GEOMETRY",
        "Проверка",
        Some("wall-example".into()),
    );
    diagnostic.source_ids = vec!["wall-example".into()];
    diagnostic.course_index = Some(4);
    let result = failure_result(&request, &[diagnostic]);
    assert!(result.get("blocks").is_none());
    assert!(result.get("beam_adjustments").is_none());
    assert_eq!(
        result["diagnostics"][0]["source_ids"],
        json!(["wall-example"])
    );
    assert_eq!(result["diagnostics"][0]["course_index"], 4);
    assert!(!failure_result(&request, &[])["diagnostics"]
        .as_array()
        .unwrap()
        .is_empty());
}

#[test]
fn actual_vertical_review_becomes_warning_without_discarding_complete_geometry() {
    use fb_layout::domain::Point;
    use fb_layout::vertical::{
        check_adjacent_nodes, JointEvidence, MaterialSegment, VerticalVerdict, WallAxis,
    };
    let lower = JointEvidence {
        segments: vec![MaterialSegment {
            axis: WallAxis {
                dx: 1,
                dy: 0,
                line: 0,
            },
            start: Point { x: 0, y: 0 },
            end: Point { x: 64000, y: 0 },
            source_ids: vec!["wall-example".into()],
            part_index: 0,
        }],
        joints: vec![],
    };
    let upper = JointEvidence {
        segments: vec![],
        joints: vec![],
    };
    let reason = match check_adjacent_nodes(&lower, &upper, 1) {
        VerticalVerdict::MissingEvidence { reason } => reason,
        verdict => panic!("Ожидалось предупреждение об отсутствии геометрии: {verdict:?}"),
    };
    let mut warning = ApiFailure::new(
        "NODE_VERTICAL_RULE_MISSING",
        reason,
        Some("wall-example".into()),
    );
    warning.course_index = Some(1);
    let request = parse_request(REQUEST).unwrap();
    let documented: Value = serde_json::from_str(include_str!(
        "../Документация/Граничные контракты/examples/layout-result.v1.json"
    ))
    .unwrap();
    let blocks = documented["blocks"].as_array().unwrap().clone();
    let result = success_result_with_warnings(&request, blocks.clone(), vec![], &[warning]);
    assert_eq!(result["status"], "success");
    assert_eq!(result["blocks"], json!(blocks));
    assert_eq!(result["warnings"][0]["message"], reason);
    assert_eq!(result["warnings"][0]["source_ids"], json!(["wall-example"]));
    assert_eq!(result["warnings"][0]["course_index"], 1);
    assert!(result.get("diagnostics").is_none());
    assert!(success_result_with_warnings(&request, vec![], vec![], &[])
        .get("warnings")
        .is_none());
    assert_eq!(
        parse_request(&mutated(|v| v["warnings"] = json!([])))
            .unwrap_err()
            .code,
        "INVALID_EXCHANGE"
    );
}

#[test]
fn fixture_adapter_roundtrip_keeps_internal_geometry_and_uses_profile_references() {
    let mut internal: LayoutRequest = parse_request(REQUEST).unwrap().to_layout_request().unwrap();
    internal.request_id.clear();
    let profile: Profile =
        serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let neutral = from_layout_request(&internal, &profile).unwrap();
    assert_eq!(neutral.catalog.id, profile.catalog_version);
    assert_eq!(neutral.profile.revision, profile.revision);
    assert!(neutral.validate_profile(&profile).is_ok());
    let mut mismatched = neutral.clone();
    mismatched.profile.revision.push('x');
    assert_eq!(
        mismatched.validate_profile(&profile).unwrap_err().code,
        "PROFILE_MISMATCH"
    );
    mismatched = neutral.clone();
    mismatched.catalog.revision.push('x');
    assert_eq!(
        mismatched.validate_profile(&profile).unwrap_err().code,
        "CATALOG_MISMATCH"
    );
    assert!(neutral.request_id.starts_with("snapshot-"));
    assert_eq!(
        neutral.to_layout_request().unwrap().wall_volumes[0].end_xmm,
        internal.wall_volumes[0].end_xmm
    );
    internal
        .opening_volumes
        .push(internal.wall_volumes[0].clone());
    assert_eq!(
        from_layout_request(&internal, &profile).unwrap_err().code,
        "UNSUPPORTED_OPENING_PURPOSE"
    );
}

use fb_layout::exchange::parse_request;
use serde_json::{json, Value};
use std::{fs, process::Command};

const REQUEST: &str =
    include_str!("../Документация/Граничные контракты/examples/layout-request.v1.json");

#[test]
fn all_source_dimensions_are_normalized_before_geometry_decisions() {
    let mut raw: Value = serde_json::from_str(REQUEST).unwrap();
    raw["coordinate_system"]["z0_mm"] = json!(-251.99999999999844);
    let prism = json!({"start_xy_mm":[1.015,-1.025],"end_xy_mm":[440.004,-1.025],
        "bottom_start_mm":-251.99999999999844,"bottom_end_mm":-251.99999999999844,
        "top_start_mm":2331.000000000002,"top_end_mm":2331.000000000002,
        "left_thickness_mm":96.500000000002,"right_thickness_mm":96.499999999998});
    raw["model"]["wall_volumes"][0]["volume"] = prism.clone();
    raw["model"]["openings"] = json!([{"id":"window","purpose":"window","volume":prism}]);
    let axis = std::f64::consts::FRAC_1_SQRT_2;
    raw["model"]["beams"] = json!([{"id":"beam","start_mm":[0.001,0.001,63.000000000002],
        "end_mm":[640.001,640.001,63.000000000002],"width_mm":160.000000000002,
        "height_mm":315.014,"height_direction":[axis,-axis,0.0]}]);
    let standard = parse_request(&raw.to_string()).unwrap();
    let request = standard.to_layout_request().unwrap();
    assert_eq!(serde_json::to_value(&standard).unwrap(), raw);
    assert_eq!(request.z0_mm, -252.0);
    for volume in request.wall_volumes.iter().chain(&request.opening_volumes) {
        assert_eq!((volume.start_xmm, volume.start_ymm), (1.02, -1.02));
        assert_eq!((volume.end_xmm, volume.end_ymm), (440.0, -1.02));
        assert_eq!(
            (volume.start_bottom_zmm, volume.end_bottom_zmm),
            (-252.0, -252.0)
        );
        assert_eq!((volume.start_top_zmm, volume.end_top_zmm), (2331.0, 2331.0));
        assert_eq!(volume.thickness_mm, 193.0);
    }
    let beam = &request.beams[0];
    assert_eq!(
        (beam.start_xmm, beam.start_ymm, beam.start_zmm),
        (0.0, 0.0, 63.0)
    );
    assert_eq!(
        (beam.end_xmm, beam.end_ymm, beam.end_zmm),
        (640.0, 640.0, 63.0)
    );
    assert_eq!(
        (beam.geometry.width_mm, beam.geometry.height_mm),
        (160.0, 315.01)
    );
    assert_eq!(beam.geometry.height_direction_x, axis);
    assert_eq!(beam.geometry.height_direction_y, -axis);

    // The legacy/programmatic boundary follows exactly the same precision rule.
    let mut legacy = request.clone();
    legacy.z0_mm += 2e-12;
    legacy.opening_volumes[0].start_top_zmm += 2e-12;
    legacy.beams[0].geometry.width_mm += 2e-12;
    let normalized = legacy.normalized();
    assert_eq!(normalized.z0_mm, -252.0);
    assert_eq!(normalized.opening_volumes[0].start_top_zmm, 2331.0);
    assert_eq!(normalized.beams[0].geometry.width_mm, 160.0);
    assert!(legacy.opening_volumes[0].start_top_zmm > 2331.0);
}

#[test]
fn dimensions_that_collapse_at_the_coordinate_quantum_are_rejected() {
    let mut raw: Value = serde_json::from_str(REQUEST).unwrap();
    raw["model"]["wall_volumes"][0]["volume"]["end_xy_mm"] = json!([0.004, 0.0]);
    assert_eq!(
        parse_request(&raw.to_string()).unwrap_err().code,
        "INVALID_GEOMETRY"
    );
}

fn round_dimensions(value: &mut Value) {
    fn numbers(value: &mut Value) {
        match value {
            Value::Number(n) => *value = json!((n.as_f64().unwrap() * 100.0).round() / 100.0),
            Value::Array(a) => a.iter_mut().for_each(numbers),
            _ => {}
        }
    }
    match value {
        Value::Object(fields) => {
            for (key, value) in fields {
                if key.ends_with("_mm") {
                    numbers(value);
                } else {
                    round_dimensions(value);
                }
            }
        }
        Value::Array(a) => a.iter_mut().for_each(round_dimensions),
        _ => {}
    }
}

fn assert_decimal_dimensions(value: &Value, dimension: bool) {
    match value {
        Value::Object(fields) => {
            for (key, value) in fields {
                assert_decimal_dimensions(value, key.ends_with("_mm") || key == "vertices");
            }
        }
        Value::Array(a) => {
            for value in a {
                assert_decimal_dimensions(value, dimension);
            }
        }
        Value::Number(n) if dimension => {
            let text = n.to_string();
            assert!(
                !text.contains(['e', 'E']),
                "unexpected dimensional exponent: {text}"
            );
            assert!(
                text.split('.').nth(1).map_or(true, |s| s.len() <= 2),
                "extra decimals: {text}"
            );
        }
        _ => {}
    }
}

#[test]
fn cli_project98_noisy_and_decimal_inputs_produce_identical_complete_results() {
    let folder = std::env::temp_dir().join(format!("fb-precision-{}", std::process::id()));
    fs::create_dir_all(&folder).unwrap();
    let profile =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("profiles/banya-prototype.json");
    let mut request: Value = serde_json::from_str(include_str!(
        "fixtures/regression/beresta44-opening-boundaries.json"
    ))
    .unwrap();
    let mut results = Vec::new();
    for name in ["noisy", "decimal"] {
        let input = folder.join(format!("{name}-request.json"));
        let output = folder.join(format!("{name}-result.json"));
        fs::write(&input, serde_json::to_vec(&request).unwrap()).unwrap();
        let status = Command::new(env!("CARGO_BIN_EXE_fb-layout"))
            .arg("--request")
            .arg(&input)
            .arg("--profile")
            .arg(&profile)
            .arg("--result")
            .arg(&output)
            .status()
            .unwrap();
        assert!(status.success());
        let result: Value = serde_json::from_slice(&fs::read(output).unwrap()).unwrap();
        assert_eq!(result["status"], "success");
        assert!(result["blocks"].as_array().unwrap().len() > 4000);
        assert_decimal_dimensions(&result, false);
        results.push(result);
        round_dimensions(&mut request);
    }
    assert_eq!(results[0], results[1]);
    fs::remove_dir_all(folder).unwrap();
}

use std::collections::BTreeSet;

use fb_layout::{api::LayoutRequest, code_result, exchange, inspect_request, layout::Profile};
use serde_json::json;

fn model() -> LayoutRequest {
    let wall = |id: &str, a: [f64; 2], b: [f64; 2]| {
        json!({
        "guid":id,"startXmm":a[0],"startYmm":a[1],"endXmm":b[0],"endYmm":b[1],
        "startBottomZmm":-252,"endBottomZmm":-252,"startTopZmm":2961,"endTopZmm":2961,
        "thicknessMm":193,"purposeType":1})
    };
    let opening = |id: &str, a: f64, b: f64| {
        json!({
        "guid":id,"startXmm":a,"startYmm":-320,"endXmm":b,"endYmm":-320,
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":2142,"endTopZmm":2142,
        "openingType":"OPENING","thicknessMm":193,"purposeType":2})
    };
    let beam = |id: &str, x: f64| {
        json!({
        "guid":id,"startXmm":x,"startYmm":-1520,"endXmm":x,"endYmm":-243.5,
        "startZmm":2457,"endZmm":2457,
        "geometry":{"widthMm":160,"heightMm":315,
            "heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}})
    };
    serde_json::from_value(json!({"schema_version":1,"request_id":"neighbour-lintels",
        "snapshot_hash":"a".repeat(64),"z0_mm":-252,
        "wall_volumes":[wall("axis",[0.,-320.],[7040.,-320.]),
            wall("stem",[3520.,-320.],[3520.,1600.])],
        "opening_volumes":[opening("left",3200.,2106.),opening("right",4934.,3840.)],
        "beams":[beam("left-beam",1920.),beam("right-beam",5120.)]}))
    .unwrap()
}

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}

#[test]
fn t_neighbours_keep_five_rows_each_and_only_valid_rows_contribute_to_the_optimum() {
    for split_at_t in [true, false] {
        let mut request = model();
        if !split_at_t {
            request.wall_volumes.retain(|wall| wall.guid != "stem");
        }
        let profile = profile();
        let result = inspect_request(&request, &profile);
        assert!(!result.blocks.is_empty(), "{:?}", result.diagnostics);
        let mut counts = Vec::new();
        for id in ["left", "right"] {
            let rows: BTreeSet<_> = result
                .blocks
                .iter()
                .filter(|b| {
                    b.is_bridge
                        && b.source_ids
                            .iter()
                            .any(|s| s == id || s == &format!("opening:{id}"))
                })
                .map(|b| b.course_index)
                .collect();
            assert_eq!(rows.len(), 5, "{id}: {rows:?}");
            let first = *rows.first().unwrap();
            assert_eq!(rows, (0..5).map(|layer| first + layer * 2).collect());
            let valid: BTreeSet<_> = result
                .blocks
                .iter()
                .filter(|b| {
                    b.lintel_is_valid()
                        && b.source_ids
                            .iter()
                            .any(|s| s == id || s == &format!("opening:{id}"))
                })
                .map(|b| b.course_index)
                .collect();
            assert!(valid.contains(&first), "the lowest row must be valid: {id}");
            counts.push(valid.len());
            assert!(result
                .diagnostics
                .iter()
                .any(|w| w.code == "LINTEL_REVIEW_REQUIRED"
                    && w.source_ids
                        .iter()
                        .any(|s| s == id || s == &format!("opening:{id}"))));
        }
        counts.sort();
        assert_eq!(counts, vec![2, 3]);
        // The diagnostic upper rows retain the real full-section gaps at both beams.
        for b in result
            .blocks
            .iter()
            .filter(|b| b.is_bridge && (43..48).contains(&b.course_index))
        {
            let low = b.start.x.min(b.end.x);
            let high = b.start.x.max(b.end.x);
            for (a, z) in [(184_000, 200_000), (504_000, 520_000)] {
                assert!(high <= a || low >= z, "{} fills a beam gap", b.id);
            }
        }
        let standard = exchange::from_layout_request(&request, &profile).unwrap();
        let exported = code_result::export_blocks(
            &standard,
            &request,
            &profile,
            &result.blocks,
            &result.diagnostics,
        )
        .unwrap();
        assert_eq!(exported["status"], "success");
        for id in ["left", "right"] {
            let rows: BTreeSet<_> = exported["blocks"]
                .as_array()
                .unwrap()
                .iter()
                .filter(|b| {
                    b["product_type"] == "Перемычка"
                        && b["source_ids"].as_array().unwrap().contains(&json!(id))
                })
                .map(|b| b["course_index"].as_i64().unwrap())
                .collect();
            assert_eq!(rows.len(), 5);
        }
        assert!(exported["warnings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|w| w["code"] == "LINTEL_REVIEW_REQUIRED"));
    }
}

#[test]
fn reversing_source_order_and_renaming_guids_preserves_selected_geometry() {
    let profile = profile();
    let request = model();
    let canonical = |request: &LayoutRequest| {
        let result = inspect_request(request, &profile);
        assert!(!result.blocks.is_empty(), "{:?}", result.diagnostics);
        let mut rows: Vec<_> = result
            .blocks
            .iter()
            .filter(|b| b.is_bridge)
            .map(|b| {
                (
                    b.course_index,
                    b.start.x,
                    b.start.y,
                    b.end.x,
                    b.end.y,
                    b.lintel_is_valid(),
                )
            })
            .collect();
        rows.sort();
        rows
    };
    let expected = canonical(&request);
    let mut reversed = model();
    reversed.wall_volumes.reverse();
    reversed.opening_volumes.reverse();
    reversed.beams.reverse();
    for opening in &mut reversed.opening_volumes {
        opening.guid = if opening.guid == "left" {
            "z-left"
        } else {
            "a-right"
        }
        .into();
    }
    assert_eq!(canonical(&reversed), expected);
}

#[test]
fn upper_rows_with_no_available_material_remain_diagnostic_placements_after_export() {
    let request: LayoutRequest = serde_json::from_value(json!({
        "schema_version":1,"request_id":"blocked-upper-rows","snapshot_hash":"b".repeat(64),"z0_mm":0,
        "wall_volumes":[{"guid":"wall","startXmm":0,"startYmm":0,"endXmm":3200,"endYmm":0,
            "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":630,"endTopZmm":630,
            "thicknessMm":193,"purposeType":1}],
        "opening_volumes":[{"guid":"opening","startXmm":1280,"startYmm":0,"endXmm":2080,"endYmm":0,
            "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":126,"endTopZmm":126,
            "openingType":"OPENING","thicknessMm":193,"purposeType":2}],
        "beams":[{"guid":"beam","startXmm":0,"startYmm":0,"endXmm":3200,"endYmm":0,
            "startZmm":252,"endZmm":252,"geometry":{"widthMm":160,"heightMm":315,
                "heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}}]})).unwrap();
    let profile = profile();
    let result = inspect_request(&request, &profile);
    assert!(!result.blocks.is_empty(), "{:?}", result.diagnostics);
    let bridges: Vec<_> = result.blocks.iter().filter(|b| b.is_bridge).collect();
    assert_eq!(bridges.len(), 3);
    assert_eq!(bridges.iter().filter(|b| b.lintel_is_valid()).count(), 1);
    assert!(bridges
        .iter()
        .filter(|b| !b.lintel_is_valid())
        .all(|b| b.course_index == 4 || b.course_index == 6));
    assert!(result
        .diagnostics
        .iter()
        .any(|w| w.code == "LINTEL_ROWS_INCOMPLETE" && w.message.contains("1 из 3")));
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let exported = code_result::export_blocks(
        &standard,
        &request,
        &profile,
        &result.blocks,
        &result.diagnostics,
    )
    .unwrap();
    assert_eq!(
        exported["blocks"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|b| b["product_type"] == "Перемычка")
            .count(),
        3
    );
    assert!(exported["warnings"]
        .as_array()
        .unwrap()
        .iter()
        .any(|w| w["code"] == "LINTEL_REVIEW_REQUIRED"));
}

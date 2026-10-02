use fb_layout::{api::LayoutRequest, exchange, inspect_request, layout::Profile};
use serde_json::json;

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}

fn request(console_length: f64) -> LayoutRequest {
    serde_json::from_value(json!({"schema_version":1,"request_id":"console-rules",
        "snapshot_hash":"a".repeat(64),"z0_mm":-252,
        "wall_volumes":[
            {"guid":"main","purposeType":1,"startXmm":0,"startYmm":0,"endXmm":3200,"endYmm":0,
             "startBottomZmm":-252,"endBottomZmm":-252,"startTopZmm":630,"endTopZmm":630,"thicknessMm":193},
            {"guid":"cross","purposeType":1,"startXmm":3200,"startYmm":0,"endXmm":3200,"endYmm":2560,
             "startBottomZmm":-252,"endBottomZmm":-252,"startTopZmm":630,"endTopZmm":630,"thicknessMm":193},
            {"guid":"console","purposeType":1,"openingType":"CONSOLE",
             "startXmm":3200,"startYmm":0,"endXmm":3200.0+console_length,"endYmm":0,
             "startBottomZmm":252.002,"endBottomZmm":252.002,"startTopZmm":630,"endTopZmm":630,"thicknessMm":193}
        ],"opening_volumes":[],"beams":[]})).unwrap()
}

#[test]
fn consoles_replace_all_type6_rows_without_vertical_shift_and_preserve_exchange() {
    let p = profile();
    let r = request(796.5);
    let standard = exchange::from_layout_request(&r, &p).unwrap();
    assert_eq!(standard.model.wall_volumes[2].purpose, "fb_console");
    let restored = standard.to_layout_request().unwrap();
    assert_eq!(restored.wall_volumes[2].opening_type, "CONSOLE");
    let mut baseline = r.clone();
    baseline.wall_volumes[2].opening_type.clear();
    let before = inspect_request(&baseline, &p);
    let eligible: Vec<_> = before
        .blocks
        .iter()
        .filter(|b| {
            b.product_key.as_deref() == Some("Type6")
                && b.source_ids.contains(&"wall:console".to_owned())
        })
        .map(|b| b.course_index)
        .collect();
    assert_eq!(eligible.len(), 3);
    let result = inspect_request(&restored, &p);
    let bridges: Vec<_> = result
        .blocks
        .iter()
        .filter(|b| b.is_bridge && b.source_ids.contains(&"console".to_owned()))
        .collect();
    assert_eq!(
        bridges.iter().map(|b| b.course_index).collect::<Vec<_>>(),
        eligible
    );
    for bridge in bridges {
        assert!(320_000 - bridge.start.x > 79_650);
        assert_eq!(bridge.end.x, 399_650);
        assert!(bridge.length_centimm <= 275_000);
        assert!(bridge.cuts.iter().any(|c| c.starts_with("Type6:")));
    }
    let mut reversed = r.clone();
    for wall in &mut reversed.wall_volumes {
        std::mem::swap(&mut wall.start_xmm, &mut wall.end_xmm);
        std::mem::swap(&mut wall.start_ymm, &mut wall.end_ymm);
    }
    let reversed = inspect_request(&reversed, &p);
    let bridges: Vec<_> = reversed
        .blocks
        .iter()
        .filter(|b| b.is_bridge && b.source_ids.contains(&"console".to_owned()))
        .collect();
    assert_eq!(
        bridges.iter().map(|b| b.course_index).collect::<Vec<_>>(),
        eligible
    );
    assert!(bridges
        .iter()
        .all(|b| b.start.x == 224_000 && b.end.x == 399_650));
}

#[test]
fn zero_opening_keeps_two_bottom_courses_and_does_not_mutate_source() {
    let mut r = request(796.5);
    r.wall_volumes.truncate(1);
    r.opening_volumes.push(
        serde_json::from_value(json!({"guid":"door","openingType":"DOOR",
        "startXmm":640,"startYmm":0,"endXmm":960,"endYmm":0,
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":378,"endTopZmm":378}))
        .unwrap(),
    );
    let result = inspect_request(&r, &profile());
    assert!(!result.blocks.is_empty());
    for course in 0..4 {
        let length: i64 = result
            .blocks
            .iter()
            .filter(|b| b.course_index == course)
            .map(|b| b.length_centimm)
            .sum();
        assert_eq!(length, if course < 2 { 320_000 } else { 288_000 });
    }
    assert_eq!(r.opening_volumes[0].start_bottom_zmm, 0.0);
}

#[test]
fn overlong_console_stays_in_its_row_and_reports_unplaced_lintels() {
    let r = request(1600.0);
    let result = inspect_request(&r, &profile());
    assert!(!result.blocks.is_empty());
    assert!(!result
        .blocks
        .iter()
        .any(|b| b.is_bridge && b.source_ids.contains(&"console".to_owned())));
    assert_eq!(
        result
            .diagnostics
            .iter()
            .filter(|d| d.code == "CONSOLE_LINTEL_UNPLACED")
            .count(),
        3
    );
}

#[test]
fn penetration_equal_to_console_length_is_insufficient() {
    let mut r = request(640.0);
    r.wall_volumes[0].start_xmm = 2560.0;
    let result = inspect_request(&r, &profile());
    assert!(!result.blocks.is_empty());
    assert!(!result
        .blocks
        .iter()
        .any(|b| b.is_bridge && b.source_ids.contains(&"console".to_owned())));
    assert_eq!(
        result
            .diagnostics
            .iter()
            .filter(|d| d.code == "CONSOLE_LINTEL_UNPLACED")
            .count(),
        3
    );
}

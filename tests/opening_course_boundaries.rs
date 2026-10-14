use fb_layout::code_result::export_codes;
use fb_layout::exchange::parse_request;
use fb_layout::layout::Profile;

#[test]
fn project98_window_keeps_the_complete_row_above_the_opening() {
    // Current source walls/openings of Beresta 44, project 98, with no beams.
    let standard = parse_request(include_str!("fixtures/regression/beresta44-opening-boundaries.json")).unwrap();
    let request = standard.to_layout_request().unwrap();
    let profile: Profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let opening = request.opening_volumes.iter()
        .find(|o| o.guid == "1aa2347b-fde4-4612-b0f7-02287b96d9d5").unwrap();
    let source = standard.model.openings.iter().find(|o| o.id == opening.guid).unwrap();
    assert!(source.volume.top_start_mm > 2331.0);
    assert!(source.volume.top_start_mm - 2331.0 < 1e-8);
    assert_eq!(opening.start_top_zmm, 2331.0);
    assert_eq!((opening.start_xmm, opening.end_xmm), (8320.0, 7360.0));

    let candidate = fb_layout::inspect_request(&request, &profile);
    let codes = export_codes(&standard, &request, &profile, &candidate.blocks, &candidate.diagnostics).unwrap();
    assert_eq!(codes["status"], "success");
    // Observe the SUP contract, including the final obstacle cutting pass.
    let spans = |course: i64| {
        let mut result: Vec<(f64, f64)> = codes["blocks"].as_array().unwrap().iter()
            .filter(|b| b["course_index"].as_i64() == Some(course))
            .filter(|b| b["placement"]["origin_mm"][1].as_f64().unwrap().abs() < 0.01)
            .filter(|b| b["placement"]["x_axis"][0].as_f64().unwrap().abs() > 0.99)
            .map(|b| {
                let start = b["placement"]["origin_mm"][0].as_f64().unwrap();
                let end = start + b["length_mm"].as_f64().unwrap()
                    * b["placement"]["x_axis"][0].as_f64().unwrap();
                (start.min(end), start.max(end))
            }).collect();
        result.sort_by(|a, b| a.0.total_cmp(&b.0));
        result
    };
    let mut covered = 7360.0;
    for (start, end) in spans(41) {
        if end <= covered { continue; }
        if start > covered + 0.01 { break; }
        covered = end;
    }
    assert!(covered >= 8320.0, "Missing material above the window from {covered} mm");
    assert!(spans(40).iter().all(|&(start, end)| end <= 7365.01 || start >= 8314.99),
        "The opening itself must remain clear");
}

#[test]
fn sloped_head_exports_full_height_staircase_without_solid_cuts_or_lintels() {
    let standard = parse_request(include_str!("fixtures/regression/sloped-window.json")).unwrap();
    let request = standard.to_layout_request().unwrap();
    let profile: Profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let candidate = fb_layout::inspect_request(&request, &profile);
    let codes = export_codes(&standard, &request, &profile, &candidate.blocks, &candidate.diagnostics).unwrap();
    assert_eq!(codes["status"], "success");
    let blocks = codes["blocks"].as_array().unwrap();
    for block in blocks {
        assert_ne!(block["product_type"], "Перемычка");
        assert_eq!(block["height_mm"], 63.0);
        assert!(block["trims"].as_array().unwrap().iter().all(|trim|
            trim["kind"] != "opening_slope" && trim["kind"] != "convex_cut"));
    }
    // Source head rises from 126 to 441 mm over x=640..1280 mm.
    // Each intersected course excludes the entire x interval occupied at its bottom.
    for course in 2..=6 {
        let boundary = 640.0 + (course as f64 * 63.0 - 126.0) * 640.0 / 315.0;
        let row: Vec<_> = blocks.iter().filter(|b| b["course_index"].as_i64() == Some(course)).collect();
        assert!(!row.is_empty());
        let mut left_end: f64 = 0.0;
        for block in row {
            let start = block["placement"]["origin_mm"][0].as_f64().unwrap();
            let end = start + block["length_mm"].as_f64().unwrap()
                * block["placement"]["x_axis"][0].as_f64().unwrap();
            let (low, high) = (start.min(end), start.max(end));
            assert!(high <= boundary + 0.01 || low >= 1280.0 - 0.01,
                "course={course}, block=[{low},{high}], opening=[{boundary},1280]");
            if high <= boundary + 0.01 { left_end = left_end.max(high); }
        }
        assert!((left_end - boundary).abs() <= 0.01,
            "Missing staircase material: course={course}, end={left_end}, expected={boundary}");
    }
}

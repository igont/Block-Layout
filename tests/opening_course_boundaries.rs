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

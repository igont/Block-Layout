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
            block.ends = fb_layout::end_state::EndStates::from_flags(true, false, natural, true);
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
fn artificial_end_with_spikes_is_rejected_on_input() {
    let (_, _, block) = inputs(0);
    let mut value = serde_json::to_value(block).unwrap();
    value.as_object_mut().unwrap().remove("left_end");
    value.as_object_mut().unwrap().remove("right_end");
    value["natural_end_left"] = json!(false);
    value["hide_spikes_left"] = json!(false);
    assert!(serde_json::from_value::<Block>(value).is_err());
}

#[test]
fn legacy_end_origin_defaults_to_natural() {
    let (_, _, block) = inputs(0);
    assert!(block.natural_end_left() && block.natural_end_right());
}

#[test]
fn ordinary_finished_part_keeps_its_production_profile() {
    let (request, profile, mut block) = inputs(0);
    block.ends = fb_layout::end_state::EndStates::factory();
    block.cuts = vec!["Type6:p32000:y1:run".into()];
    let rendered = materialize(&request, &[block.clone()], &profile).unwrap();
    let actual: f64 = rendered.physical.blocks.iter().flat_map(|b| &b.bodies).map(volume).sum();
    assert!(actual < 640.0 * 193.0 * 63.0 - 1.0);
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let result = export_codes(&standard, &request, &profile, &[block], &[]).unwrap();
    assert_eq!(result["blocks"][0]["code1"], "П640 [Н-320-Тип 6]");
    assert_eq!(result["blocks"][0]["code2"], "П640 [В-320-Тип 6]");
}

#[test]
fn opening_cut_is_preserved_through_codes_and_physical_export_in_two_rough_courses() {
    use fb_layout::end_state::{CutSource, EndState};
    let profile: Profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let request: LayoutRequest = serde_json::from_value(json!({
        "schema_version":1,"request_id":"rough-opening","snapshot_hash":"b".repeat(64),"z0_mm":-252,
        "wall_volumes":[{"guid":"wall","startXmm":0,"startYmm":0,"endXmm":3200,"endYmm":0,
            "startBottomZmm":-252,"endBottomZmm":-252,"startTopZmm":378,"endTopZmm":378,
            "thicknessMm":193,"purposeType":1}],
        "opening_volumes":[{"guid":"opening","openingType":"OPENING","isOutside":false,
            "startXmm":640,"startYmm":0,"endXmm":1600,"endYmm":0,
            "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":378,"endTopZmm":378,"thicknessMm":193}],"beams":[]
    })).unwrap();
    let source = serde_json::to_value(exchange::from_layout_request(&request, &profile).unwrap()).unwrap();
    let candidate = fb_layout::inspect_request(&request, &profile);
    assert!(!candidate.blocks.is_empty(), "{:?}", candidate.diagnostics);
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let exported = export_codes(&standard, &request, &profile, &candidate.blocks, &candidate.diagnostics).unwrap();
    let rendered = materialize(&request, &candidate.blocks, &profile).unwrap();
    for course in [2, 3] {
        let sides: Vec<_> = candidate.blocks.iter().filter(|b| b.course_index == course)
            .flat_map(|b| [(b.start, b.ends.left()), (b.end, b.ends.right())])
            .filter(|(p, _)| p.x == 64_000 || p.x == 160_000).collect();
        assert_eq!(sides.len(), 2);
        for (_, state) in sides {
            assert!(matches!(state, EndState::FactoryCut(_) | EndState::SawCut(_)));
            assert!(state.face().unwrap().sources.contains(&CutSource::Opening("opening".into())));
        }
        let values: Vec<_> = exported["blocks"].as_array().unwrap().iter()
            .filter(|b| b["course_index"] == course).collect();
        for value in values {
            let original = candidate.blocks.iter().find(|b| b.id == value["id"].as_str().unwrap()).unwrap();
            assert_eq!(value["left_end"], serde_json::to_value(original.ends.left()).unwrap());
            assert_eq!(value["right_end"], serde_json::to_value(original.ends.right()).unwrap());
        }
    }
    // The lower two crowns retain material underneath the opening.
    for course in [0, 1] {
        assert!(candidate.blocks.iter().any(|b| b.course_index == course
            && b.start.x < 160_000 && b.end.x > 64_000));
    }
    assert_eq!(rendered.physical.blocks.len(), candidate.blocks.len());
    assert_eq!(serde_json::to_value(exchange::from_layout_request(&request, &profile).unwrap()).unwrap(), source);
}

#[test]
fn split_reversal_and_join_preserve_cut_faces_and_source() {
    use fb_layout::end_state::{CutSource, EndState, EndStates};
    let mut ends = EndStates::factory();
    ends.record_cut(true, 0, CutSource::Opening("door".into())).unwrap();
    ends.record_cut(false, 64_000, CutSource::Beam("beam".into())).unwrap();
    let left = ends.slice(0, 32_000, 64_000, CutSource::Stock, CutSource::Stock);
    let right = ends.slice(32_000, 64_000, 64_000, CutSource::Stock, CutSource::Stock);
    assert_eq!(left.left(), ends.left());
    assert!(matches!(left.right(), EndState::SawCut(_)));
    assert!(matches!(right.left(), EndState::SawCut(_)));
    assert_eq!(right.right().face().unwrap().sources, ends.right().face().unwrap().sources);
    assert_eq!(right.right().face().unwrap().plane_centimm, 32_000);
    assert_eq!(ends.reversed(64_000).reversed(64_000), ends);
    assert_eq!(EndStates::outer(&left, &right, 32_000), ends);
    let saved = ends.clone();
    ends.record_cut(true, 0, CutSource::Opening("door".into())).unwrap();
    assert_eq!(ends, saved);
}

#[test]
fn a_new_cause_cannot_move_an_existing_cut_face() {
    use fb_layout::end_state::{CutSource, EndStates};
    let mut ends = EndStates::factory();
    ends.record_cut(false, 64_000, CutSource::Opening("door".into())).unwrap();
    let saved = ends.clone();
    let error = ends.record_cut(false, 63_500, CutSource::Beam("beam".into())).unwrap_err();
    assert_eq!((error.recorded_centimm, error.requested_centimm), (64_000, 63_500));
    assert_eq!(ends, saved);
    ends.record_cut(false, 64_000, CutSource::Beam("beam".into())).unwrap();
    assert_eq!(ends.right().face().unwrap().sources.len(), 2);
}

#[test]
fn legacy_unknown_plane_is_bound_once_to_a_real_cut() {
    use fb_layout::end_state::{CutSource, EndStates};
    let mut ends = EndStates::from_flags(false, true, true, true);
    ends.record_cut(false, 32_000, CutSource::Opening("door".into())).unwrap();
    let cut = ends.right().face().unwrap();
    assert_eq!(cut.plane_centimm, 32_000);
    assert_eq!(cut.sources, vec![CutSource::Opening("door".into())]);
    assert!(ends.record_cut(false, 31_500, CutSource::Opening("door".into())).is_err());
}

#[test]
fn shortened_t_stem_does_not_cut_the_perpendicular_catalog_part() {
    use fb_layout::end_state::CutSource;
    let profile: Profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let wall = |guid: &str, sx, sy, ex, ey, start_top, end_top| json!({
        "guid":guid,"startXmm":sx,"startYmm":sy,"endXmm":ex,"endYmm":ey,
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":start_top,"endTopZmm":end_top,
        "thicknessMm":193,"purposeType":1
    });
    let request: LayoutRequest = serde_json::from_value(json!({
        "schema_version":1,"request_id":"short-t-stem","snapshot_hash":"c".repeat(64),"z0_mm":0,
        "wall_volumes":[wall("left",-1280,0,0,0,189,189),wall("right",0,0,1280,0,189,189),
            wall("stem",0,0,0,-800,252,1)],"beams":[]
    })).unwrap();
    let candidate = fb_layout::inspect_request(&request, &profile);
    assert!(!candidate.blocks.is_empty(), "{:?}", candidate.diagnostics);
    let nodes: Vec<_> = candidate.blocks.iter().filter(|b| b.kind == "node_T" && b.course_index == 2).collect();
    assert!(!nodes.is_empty(), "{:?}", candidate.diagnostics);
    assert!(nodes.iter().any(|block| block.rotation_deg % 180 == 90
        && [block.ends.left(), block.ends.right()].into_iter().any(|end|
            end.face().is_some_and(|face| face.sources.contains(&CutSource::Wall("stem".into()))))));
    let axial: Vec<_> = nodes.iter().filter(|b| b.rotation_deg % 180 == 0).collect();
    assert!(!axial.is_empty());
    for block in axial {
        assert!(!block.hide_spikes_left() && !block.hide_spikes_right(), "{}: {:?}", block.id, block.ends);
        // Adjacent walls can explain catalog joint machining without cutting
        // either axial end. CutFace, not the shared provenance set, owns that fact.
        assert!(matches!(block.ends.left(), fb_layout::end_state::EndState::FactoryUncut));
        assert!(matches!(block.ends.right(), fb_layout::end_state::EndState::FactoryUncut));
        assert!(block.ends.left().face().is_none() && block.ends.right().face().is_none());
    }
    let standard = exchange::from_layout_request(&request, &profile).unwrap();
    let result = export_codes(&standard, &request, &profile, &candidate.blocks, &candidate.diagnostics).unwrap();
    let mut wall_cuts = 0;
    for value in result["blocks"].as_array().unwrap() {
        let block = candidate.blocks.iter().find(|b| b.id == value["id"].as_str().unwrap()).unwrap();
        if block.kind == "node_T" && block.course_index == 2 && block.rotation_deg % 180 == 0 {
            assert_eq!(value["left_end"], json!({"state":"factory_uncut"}));
            assert_eq!(value["right_end"], json!({"state":"factory_uncut"}));
            assert_eq!(value["hide_spikes_left"], false);
            assert_eq!(value["hide_spikes_right"], false);
        }
        for (side, end) in [("left_end", block.ends.left()), ("right_end", block.ends.right())] {
            assert_eq!(value[side], serde_json::to_value(end).unwrap());
            for cause in end.face().into_iter().flat_map(|face| &face.sources) {
                if let CutSource::Wall(id) = cause {
                    wall_cuts += 1;
                    assert!(request.wall_volumes.iter().any(|w| &w.guid == id));
                    assert!(value["source_ids"].as_array().unwrap().iter().any(|source| source.as_str() == Some(id)));
                }
            }
        }
    }
    assert!(wall_cuts > 0);
}

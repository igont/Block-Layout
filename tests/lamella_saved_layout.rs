use fb_layout::{
    api::LayoutRequest,
    exchange,
    lamella_saved::{self, SavedBlock, SavedPlacement},
    layout::Profile,
    solid_geometry::{self, Mesh},
};
use serde_json::{json, Value};

fn profile() -> Profile {
    serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap()
}
fn request() -> exchange::ExchangeRequest {
    let r:LayoutRequest=serde_json::from_value(json!({"schema_version":1,"request_id":"saved-lamella","snapshot_hash":"a".repeat(64),"z0_mm":0,
        "wall_volumes":[{"guid":"wall","startXmm":0,"startYmm":0,"endXmm":1920,"endYmm":0,
        "startBottomZmm":0,"endBottomZmm":0,"startTopZmm":63,"endTopZmm":63,"thicknessMm":193,"purposeType":1}],
        "beams":[{"guid":"beam","startXmm":500,"startYmm":0,"endXmm":900,"endYmm":0,"startZmm":0,"endZmm":0,
        "geometry":{"widthMm":160,"heightMm":63,"heightDirectionZ":1}}]})).unwrap();
    let mut standard = exchange::from_layout_request(&r, &profile()).unwrap();
    standard.kind = "lamella_layout_request".into();
    standard.model.saved_blocks = Some(vec![]);
    standard
}
fn body(start: f64, end: f64, y: f64, width: f64) -> Mesh {
    solid_geometry::box_mesh(
        [start, y - width / 2.0, 0.0],
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [end - start, width, 63.0],
    )
}
fn saved(start: f64) -> SavedBlock {
    SavedBlock {
        id: "manual-guid".into(),
        placement: SavedPlacement {
            origin_mm: [start, 0.0, 0.0],
            x_axis: [1.0, 0.0, 0.0],
            y_axis: [0.0, 1.0, 0.0],
            z_axis: [0.0, 0.0, 1.0],
        },
        length_mm: 640.0,
        width_mm: 193.0,
        height_mm: 63.0,
        course_index: 0,
        wall_ids: vec!["wall".into()],
        solids: vec![body(start, start + 640.0, 0.0, 160.0)],
        grid_start_mm: Some([start, 0.0, 0.0]),
        grid_end_mm: Some([start + 640.0, 0.0, 0.0]),
    }
}
fn xbounds(l: &Value) -> (f64, f64) {
    let c = l["placement"]["center_mm"][0].as_f64().unwrap();
    let n = l["length_mm"].as_f64().unwrap();
    (c - n / 2.0, c + n / 2.0)
}

#[test]
fn saved_geometry_roundoff_does_not_change_lamella_output() {
    let mut r = request();
    r.model.saved_blocks = Some(vec![saved(640.0)]);
    let expected = lamella_saved::export(&r, &profile()).unwrap();
    let b = &mut r.model.saved_blocks.as_mut().unwrap()[0];
    b.length_mm += 2e-12;
    b.placement.origin_mm[0] += 2e-12;
    b.grid_start_mm.as_mut().unwrap()[0] += 2e-12;
    for mesh in &mut b.solids {
        for p in &mut mesh.vertices {
            for v in p { *v += 2e-12; }
        }
    }
    let source = serde_json::to_value(&r).unwrap();
    assert_eq!(lamella_saved::export(&r, &profile()).unwrap(), expected);
    assert_eq!(serde_json::to_value(&r).unwrap(), source);
}

#[test]
fn beresta44_export_keeps_sloped_course_closed_after_rounding_collapsed_edges() {
    let request: exchange::ExchangeRequest = serde_json::from_str(include_str!(
        "fixtures/regression/beresta44-lamella-quantized-edge.json"
    )).unwrap();
    let before = serde_json::to_value(&request).unwrap();
    let plan = lamella_saved::calculate(&request, &profile()).unwrap();
    assert!(plan.lamellas.iter().filter_map(|l| l.solid.as_ref()).any(|mesh| {
        let rounded: Vec<_> = mesh.vertices.iter().map(|p| p.map(fb_layout::precision::mm)).collect();
        rounded.iter().enumerate().any(|(i, p)| rounded[..i].contains(p))
    }), "Реальный срез должен схлопывать короткое ребро при округлении");
    let result = lamella_saved::export(&request, &profile()).unwrap();
    assert_eq!(serde_json::to_value(&request).unwrap(), before);
    assert_eq!(result["blocks"], json!([]));
    assert_eq!(result["lamellas"].as_array().unwrap().len(), 16);
    let mut exact = 0;
    for lamella in result["lamellas"].as_array().unwrap() {
        let Some(solid) = lamella.get("solid") else { continue; };
        let mesh: Mesh = serde_json::from_value(solid.clone()).unwrap();
        assert_eq!(lamella["course_index"], 68);
        assert_eq!(lamella["length_mm"], 615.72);
        assert_eq!(lamella["height_mm"], 63.0);
        assert_eq!(mesh.vertices.len(), 8);
        assert_eq!(mesh.faces.len(), 6);
        let mut edges = std::collections::BTreeMap::new();
        for (i, vertex) in mesh.vertices.iter().enumerate() {
            assert!(!mesh.vertices[..i].contains(vertex));
        }
        for face in &mesh.faces {
            assert!(face.len() >= 3);
            for i in 0..face.len() {
                let a = face[i];
                let b = face[(i + 1) % face.len()];
                assert_ne!(a, b);
                assert!(edges.insert((a, b), ()).is_none());
            }
        }
        for &(a, b) in edges.keys() { assert!(edges.contains_key(&(b, a))); }
        assert!(solid_geometry::volume(&mesh) > 0.0);
        exact += 1;
    }
    assert_eq!(exact, 2);
}

#[test]
fn sloping_saved_face_remains_usable_after_coordinate_quantization() {
    let mut r = request();
    r.model.beams.clear();
    let mut block = saved(640.0);
    for p in &mut block.solids[0].vertices {
        if p[2] > 0.0 { p[2] += 0.0173 * p[0] + 0.02353 * p[1]; }
    }
    r.model.saved_blocks = Some(vec![block]);
    assert_eq!(lamella_saved::export(&r, &profile()).unwrap()["status"], "success");
    let mesh = &mut r.model.saved_blocks.as_mut().unwrap()[0].solids[0];
    mesh.vertices.iter_mut().find(|p| p[2] > 0.0).unwrap()[2] += 1.0;
    assert_eq!(lamella_saved::export(&r, &profile()).unwrap_err().code, "INVALID_SAVED_BLOCK");
}

#[test]
fn short_full_width_remnant_keeps_base_cell_and_authorizes_only_its_recess() {
    let mut r = request();
    let mut short = saved(640.0);
    short.length_mm = 320.0;
    short.grid_end_mm = Some([960.0, 0.0, 0.0]);
    short.solids = vec![body(640.0, 960.0, 0.0, 193.0)];
    r.model.saved_blocks = Some(vec![short]);
    let before = serde_json::to_value(&r).unwrap();
    let result = lamella_saved::export(&r, &profile()).unwrap();
    assert_eq!(serde_json::to_value(&r).unwrap(), before);
    let ls = result["lamellas"].as_array().unwrap();
    assert!(!ls.is_empty());
    for side in ["left", "right"] {
        let cell = ls
            .iter()
            .find(|l| l["side"] == side && l.get("recess_block_ids").is_some())
            .unwrap();
        assert_eq!(cell["recess_block_ids"], json!(["manual-guid"]));
        assert_eq!(cell["length_mm"], json!(640.0));
    }
    assert_eq!(result["blocks"], json!([]));
}

#[test]
fn whole_630_and_640_facade_blocks_suppress_the_entire_cell_without_spike_nibbles() {
    for nominal in [630.0, 640.0] {
        let mut r = request();
        r.model.beams[0].start_mm[0] = 0.0;
        r.model.beams[0].end_mm[0] = 5.0;
        let mut b = saved(0.0);
        b.length_mm = nominal;
        b.grid_end_mm = Some([nominal, 0.0, 0.0]);
        b.solids = vec![body(5.0, 635.0, 0.0, 193.0)];
        r.model.saved_blocks = Some(vec![b]);
        let result = lamella_saved::export(&r, &profile()).unwrap();
        assert_eq!(result["lamellas"], json!([]), "nominal {nominal}");
    }
}

#[test]
fn third_pass_does_not_cut_neighbor_sides_for_five_mm_end_spikes() {
    let mut r = request();
    r.model.beams[0].start_mm[0] = 650.0;
    r.model.beams[0].end_mm[0] = 900.0;
    let mut short = saved(960.0);
    short.length_mm = 320.0;
    short.grid_end_mm = Some([1280.0, 0.0, 0.0]);
    short.solids = vec![body(965.0, 1285.0, 0.0, 193.0)];
    let mut whole = saved(1280.0);
    whole.id = "whole-with-spike".into();
    whole.solids = vec![body(1275.0, 1925.0, 0.0, 193.0)];
    let mut previous = saved(0.0);
    previous.id = "previous-with-spike".into();
    previous.solids = vec![body(-5.0, 645.0, 0.0, 193.0)];
    r.model.saved_blocks = Some(vec![short, whole, previous]);
    let result = lamella_saved::export(&r, &profile()).unwrap();
    for l in result["lamellas"].as_array().unwrap() {
        assert_eq!(xbounds(l), (640.0, 1280.0));
        assert_eq!(
            l["recess_block_ids"],
            json!(["manual-guid"])
        );
    }
    assert_eq!(result["lamellas"].as_array().unwrap().len(), 2);
}

#[test]
fn third_pass_uses_grid_and_course_without_reading_finished_geometry() {
    let mut r = request();
    let mut block = saved(640.0);
    block.length_mm = 320.0;
    block.grid_end_mm = Some([960.0, 0.0, 0.0]);
    block.solids = vec![body(640.0, 960.0, 0.0, 193.0)];
    r.model.saved_blocks = Some(vec![block]);
    let mut plan = lamella_saved::calculate(&r, &profile()).unwrap();
    let expected: Vec<_> = plan.lamellas.iter().map(|l| l.recess_block_ids.clone()).collect();
    assert!(expected.iter().any(|ids| ids.contains("manual-guid")));
    let blocks = r.model.saved_blocks.as_mut().unwrap();
    blocks[0].solids.clear();
    let mut other_course = blocks[0].clone();
    other_course.id = "other-course".into();
    other_course.course_index += 1;
    blocks.push(other_course);
    for l in &mut plan.lamellas {
        l.placement.center_mm = [9000.0; 3];
        l.length_mm = 1.0;
        l.solid = Some(Mesh { vertices: vec![[0.0; 3]], faces: vec![] });
        l.recess_block_ids.insert("stale-reference".into());
    }
    lamella_saved::assign_grid_recesses(&mut plan.lamellas, blocks);
    let actual: Vec<_> = plan.lamellas.iter().map(|l| l.recess_block_ids.clone()).collect();
    assert_eq!(actual, expected, "Физическое тело не определяет срез стороны по сетке");
}

#[test]
fn short_recess_relations_and_neighbor_clipping_are_order_and_face_invariant() {
    let mut r = request();
    let mut short = saved(640.0);
    short.length_mm = 320.0;
    short.grid_end_mm = Some([960.0, 0.0, 0.0]);
    short.solids = vec![body(640.0, 960.0, 0.0, 193.0)];
    let mut neighbor = saved(1000.0);
    neighbor.id = "neighbor".into();
    neighbor.wall_ids = vec!["other-wall".into()];
    neighbor.solids = vec![body(1000.0, 1100.0, 0.0, 193.0)];
    r.model.saved_blocks = Some(vec![short, neighbor]);
    let first = lamella_saved::export(&r, &profile()).unwrap();
    let blocks = r.model.saved_blocks.as_mut().unwrap();
    blocks.reverse();
    for b in blocks {
        for m in &mut b.solids {
            m.faces.rotate_left(2);
        }
    }
    let second = lamella_saved::export(&r, &profile()).unwrap();
    assert_eq!(first["lamellas"], second["lamellas"]);
    assert!(first["lamellas"].as_array().unwrap().iter().all(|l| l
        .get("recess_block_ids")
        .map_or(true, |ids| ids == &json!(["manual-guid"]))));
}

#[test]
fn beam_cut_ends_do_not_break_bonded_modules_over_half_blocks() {
    let mut r = request();
    r.model.wall_volumes[0].volume.top_start_mm = 126.0;
    r.model.wall_volumes[0].volume.top_end_mm = 126.0;
    r.model.wall_volumes[0].volume.end_xy_mm[0] = 2560.0;
    r.model.beams[0].height_mm = 126.0;
    let mut blocks = Vec::new();
    for course in 0..2 {
        let z = course as f64 * 63.0;
        let mut whole = saved(1280.0 + course as f64 * 320.0);
        whole.id = format!("whole-{course}");
        let mut half = saved(320.0);
        half.id = format!("half-at-beam-{course}");
        half.length_mm = 180.0;
        half.grid_end_mm = Some([500.0, 0.0, 0.0]);
        half.solids = vec![body(320.0, 500.0, 0.0, 96.5)];
        for mut b in [whole, half] {
            b.course_index = course;
            b.placement.origin_mm[2] = z;
            for mesh in &mut b.solids {
                for vertex in &mut mesh.vertices {
                    vertex[2] += z;
                }
            }
            blocks.push(b);
        }
    }
    r.model.saved_blocks = Some(blocks);
    let before = serde_json::to_value(&r.model.saved_blocks).unwrap();
    let result = lamella_saved::export(&r, &profile()).unwrap();
    for course in 0..2 {
        let mut spans: Vec<_> = result["lamellas"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|l| l["course_index"] == course && l["side"] == "left")
            .map(xbounds)
            .collect();
        spans.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert_eq!(
            spans,
            if course == 0 {
                vec![(0.0, 640.0), (640.0, 1280.0)]
            } else {
                vec![(320.0, 960.0)]
            }
        );
    }
    assert_eq!(serde_json::to_value(&r.model.saved_blocks).unwrap(), before);
}

#[test]
fn course_without_whole_blocks_uses_bond_from_saved_course_above() {
    let mut r = request();
    let mut above = saved(320.0);
    above.course_index = 1;
    above.placement.origin_mm[2] = 63.0;
    for mesh in &mut above.solids {
        for vertex in &mut mesh.vertices {
            vertex[2] += 63.0;
        }
    }
    r.model.saved_blocks = Some(vec![above]);
    let result = lamella_saved::export(&r, &profile()).unwrap();
    let mut spans: Vec<_> = result["lamellas"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|l| l["side"] == "left")
        .map(xbounds)
        .collect();
    spans.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(spans, vec![(0.0, 640.0), (640.0, 1280.0)]);
}

#[test]
fn manual_saved_grid_is_authoritative_and_source_blocks_are_unchanged() {
    for start in [320.0, 420.0] {
        let mut r = request();
        r.model.saved_blocks = Some(vec![saved(start)]);
        let before = serde_json::to_value(&r).unwrap();
        let result = lamella_saved::export(&r, &profile()).unwrap();
        assert_eq!(serde_json::to_value(&r).unwrap(), before);
        assert_eq!(result["blocks"], json!([]));
        let ls = result["lamellas"].as_array().unwrap();
        assert_eq!(ls.len(), 2);
        for l in ls {
            assert_eq!(xbounds(l), (start, start + 640.0));
            assert!(l.get("trims").is_none());
        }
        assert!(result.get("warnings").is_none());
    }
}

#[test]
fn saved_material_is_subtracted_without_emitting_any_block_edits() {
    let mut r = request();
    let mut block = saved(420.0);
    // Real preexisting material occupies the facade only in this 200mm window;
    // the rest of the saved block keeps its already recessed facade geometry.
    block.solids.push(body(600.0, 800.0, 0.0, 193.0));
    r.model.saved_blocks = Some(vec![block]);
    let before = serde_json::to_value(&r.model.saved_blocks).unwrap();
    let result = lamella_saved::export(&r, &profile()).unwrap();
    assert_eq!(result["blocks"], json!([]));
    assert_eq!(serde_json::to_value(&r.model.saved_blocks).unwrap(), before);
    let parts = result["lamellas"].as_array().unwrap();
    assert_eq!(parts.len(), 4);
    for side in ["left", "right"] {
        let mut xs: Vec<_> = parts
            .iter()
            .filter(|l| l["side"] == side)
            .map(xbounds)
            .collect();
        xs.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert_eq!(xs, vec![(420.0, 600.0), (800.0, 1060.0)]);
    }
    assert!(result["warnings"]
        .as_array()
        .unwrap()
        .iter()
        .any(|w| w["code"] == "LAMELLA_SAVED_MATERIAL_CLIPPED"));
}

#[test]
fn partial_thickness_obstacle_does_not_remove_free_ends_when_faces_start_across_skin() {
    let mut r = request();
    let mut b = saved(420.0);
    let mut obstacle = body(600.0, 800.0, 0.0, 183.0);
    obstacle.faces.rotate_left(2);
    b.solids.push(obstacle);
    r.model.saved_blocks = Some(vec![b]);
    let result = lamella_saved::export(&r, &profile()).unwrap();
    for side in ["left", "right"] {
        let mut spans: Vec<_> = result["lamellas"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|l| l["side"] == side)
            .map(xbounds)
            .collect();
        spans.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert_eq!(spans, vec![(420.0, 600.0), (800.0, 1060.0)]);
    }
}

#[test]
fn partial_skin_thickness_is_not_mislabelled_as_a_15mm_lamella() {
    let mut r = request();
    let mut b = saved(420.0);
    b.solids.push(body(420.0, 1060.0, 84.0, 5.0));
    r.model.saved_blocks = Some(vec![b]);
    let result = lamella_saved::export(&r, &profile()).unwrap();
    let parts = result["lamellas"].as_array().unwrap();
    assert_eq!(parts.len(), 1);
    assert_eq!(parts[0]["side"], "right");
    assert!(result["warnings"]
        .as_array()
        .unwrap()
        .iter()
        .any(|w| w["code"] == "LAMELLA_SAVED_MATERIAL_THIN_FRAGMENT"));
}

#[test]
fn saved_joint_endpoints_override_l_stock_extent() {
    let mut r = request();
    let mut b = saved(420.0);
    b.placement.origin_mm[0] = 323.5;
    b.length_mm = 736.5;
    r.model.saved_blocks = Some(vec![b]);
    let result = lamella_saved::export(&r, &profile()).unwrap();
    assert!(result["lamellas"]
        .as_array()
        .unwrap()
        .iter()
        .all(|l| xbounds(l) == (420.0, 1060.0)));
}

#[test]
fn no_saved_grid_warns_and_missing_array_or_nonconvex_material_fails_explicitly() {
    let mut r = request();
    let result = lamella_saved::export(&r, &profile()).unwrap();
    assert_eq!(result["blocks"], json!([]));
    assert_eq!(result["lamellas"], json!([]));
    assert_eq!(
        result["warnings"][0]["code"],
        "LAMELLA_SAVED_GRID_UNAVAILABLE"
    );
    r.model.saved_blocks = None;
    assert_eq!(r.to_layout_request().unwrap_err().code, "INVALID_EXCHANGE");
    let mut b = saved(420.0);
    b.solids[0].faces.pop();
    r.model.saved_blocks = Some(vec![b]);
    assert_eq!(
        lamella_saved::export(&r, &profile()).unwrap_err().code,
        "INVALID_SAVED_BLOCK"
    );
}

#[test]
fn cli_lamella_route_never_runs_solver_or_publishes_candidate_blocks() {
    let mut r = request();
    r.model.saved_blocks = Some(vec![saved(420.0)]);
    let folder = std::env::temp_dir().join(format!("fb-saved-lamella-{}", std::process::id()));
    std::fs::create_dir_all(&folder).unwrap();
    let input = folder.join("request.json");
    let result = folder.join("result.json");
    let candidate = folder.join("candidate.json");
    let bytes = serde_json::to_vec(&r).unwrap();
    std::fs::write(&input, &bytes).unwrap();
    let status = std::process::Command::new(env!("CARGO_BIN_EXE_fb-layout"))
        .args(["--request"])
        .arg(&input)
        .arg("--profile")
        .arg(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("profiles/banya-prototype.json"))
        .arg("--result")
        .arg(&result)
        .arg("--candidate")
        .arg(&candidate)
        .status()
        .unwrap();
    assert!(status.success());
    assert!(!candidate.exists());
    assert_eq!(std::fs::read(&input).unwrap(), bytes);
    let response: Value = serde_json::from_slice(&std::fs::read(&result).unwrap()).unwrap();
    assert_eq!(response["blocks"], json!([]));
    assert_eq!(response["lamellas"].as_array().unwrap().len(), 2);
    std::fs::remove_file(&input).unwrap();
    std::fs::remove_file(&result).unwrap();
    std::fs::remove_dir(&folder).unwrap();
}

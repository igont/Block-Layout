use fb_layout::{exchange::parse_request,layout::Profile,inspect_request,code_result::export_blocks};
use serde_json::{json,Value};

#[test]
fn roof_removes_whole_candidates_before_production_and_preserves_partial_blocks() {
    let profile:Profile=serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
    let mut input:Value=serde_json::from_str(include_str!("../Документация/Граничные контракты/examples/layout-request.v1.json")).unwrap();
    input["model"]["wall_volumes"][0]["volume"]=json!({"start_xy_mm":[0,0],"end_xy_mm":[1280,0],
        "bottom_start_mm":0,"bottom_end_mm":0,"top_start_mm":252,"top_end_mm":252,
        "left_thickness_mm":96.5,"right_thickness_mm":96.5});
    input["model"]["openings"]=json!([]);input["model"]["beams"]=json!([]);
    input["coordinate_system"]["z0_mm"]=json!(0);
    let original=parse_request(&input.to_string()).unwrap().to_layout_request().unwrap();
    let baseline=inspect_request(&original,&profile);
    assert!(!baseline.blocks.is_empty());
    input["model"]["roof_volumes"]=json!([{"id":"r","top_vertices_mm":[[-100,-200,200],[1400,-200,200],[1400,200,200],[-100,200,200]],"top_triangles":[[0,1,2],[0,2,3]]}]);
    let standard=parse_request(&input.to_string()).unwrap();let request=standard.to_layout_request().unwrap();
    let result=inspect_request(&request,&profile);
    let expected:Vec<_>=baseline.blocks.into_iter().filter(|b|b.z_centimm<10_000).collect();
    assert_eq!(expected,result.blocks);
    assert!(result.blocks.iter().any(|b|b.z_centimm==6300));
    let output=export_blocks(&standard,&request,&profile,&result.blocks,&result.diagnostics).unwrap();
    assert_eq!(output["blocks"].as_array().unwrap().len(),result.blocks.len());
    assert!(result.diagnostics.iter().any(|d|d.code=="ROOF_BLOCKS_EXCLUDED"));

    for vertex in input["model"]["roof_volumes"][0]["top_vertices_mm"].as_array_mut().unwrap() {
        vertex[2]=json!(-100);
    }
    let standard=parse_request(&input.to_string()).unwrap();
    let request=standard.to_layout_request().unwrap();
    let result=inspect_request(&request,&profile);
    assert!(result.blocks.is_empty());
    let output=export_blocks(&standard,&request,&profile,&result.blocks,&result.diagnostics).unwrap();
    assert_eq!(output["status"],"success");
    assert!(output["blocks"].as_array().unwrap().is_empty());
    assert!(output["warnings"].as_array().unwrap().iter().any(|w|w["code"]=="ROOF_BLOCKS_EXCLUDED"));
}

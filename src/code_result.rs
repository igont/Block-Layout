//! Коды, положение и факты обработки для SUP. Геометрию изделия строит SUP.
use crate::api::{ApiFailure, LayoutRequest};
use crate::exchange::{self, ExchangeRequest};
use crate::layout::{Block, Profile};
use serde_json::{json, Value};
use std::collections::BTreeSet;

fn issue(block: &Block, message: &str) -> ApiFailure {
    ApiFailure::new("INVALID_PRODUCT_CODE", message, Some(block.id.clone()))
}
fn mm(value: i64) -> f64 {
    value as f64 / 100.0
}
fn number(value: i64) -> String {
    if value % 100 == 0 {
        return (value / 100).to_string();
    }
    let text = format!("{}.{:02}", value / 100, value % 100);
    text.trim_end_matches('0').to_owned()
}
fn type_number(key: &str) -> Option<String> {
    let value = key.strip_prefix("Type")?.replace('_', ".");
    let value = match value.as_str() {
        "5" => "5.1".into(),
        "10" => "10.1".into(),
        _ => value,
    };
    [
        "1", "2", "3", "4", "5.1", "6", "7.1", "8", "8.1", "10.1", "11", "12", "13", "14",
    ]
    .contains(&value.as_str())
    .then_some(value)
}
fn token(face: u8, position: i64, length: i64, product: &str) -> String {
    let face = match face {
        1 => "Н",
        2 => "С",
        3 => "В",
        _ => unreachable!(),
    };
    let position = if position == 0 {
        "НЧ".into()
    } else if position == length {
        "КН".into()
    } else {
        number(position)
    };
    format!(" [{face}-{position}-Тип {product}]")
}

fn export_block(
    request: &LayoutRequest,
    profile: &Profile,
    block: &Block,
) -> Result<Value, ApiFailure> {
    if block.id.is_empty() || block.length_centimm <= 0 || block.length_centimm > 100_000_000 {
        return Err(issue(block, "Некорректная идентичность или длина детали"));
    }
    let mut tokens = Vec::new();
    let mut node_cuts = Vec::new();
    let mut trims = Vec::new();
    for cut in &block.cuts {
        if cut.starts_with("Type") {
            let mut fields = cut.splitn(4, ':');
            let key = fields.next().unwrap_or("");
            let product = type_number(key).ok_or_else(|| issue(block, "Неизвестный тип врезки"))?;
            let x: u8 = fields
                .next()
                .and_then(|s| s.strip_prefix('x'))
                .and_then(|s| s.parse().ok())
                .filter(|x| (1..=3).contains(x))
                .ok_or_else(|| issue(block, "Некорректная позиция x врезки"))?;
            let y: u8 = fields
                .next()
                .and_then(|s| s.strip_prefix('y'))
                .and_then(|s| s.parse().ok())
                .filter(|y| (1..=3).contains(y))
                .ok_or_else(|| issue(block, "Некорректная грань y врезки"))?;
            let sources = fields
                .next()
                .ok_or_else(|| issue(block, "Не указано происхождение врезки"))?;
            let position = ((i64::from(x) - 1) * 32_000).min(block.length_centimm);
            tokens.push((position, y, product));
            node_cuts.push(json!({"product_type":format!("Тип {}",key.trim_start_matches("Type").replace('_',".")),
                "x":x,"y":y,"position_mm":mm(position),"face":match y {1=>"Н",2=>"С",_=>"В"},
                "source_run_ids":sources.split(',').collect::<Vec<_>>()}));
        } else {
            let mut trim = json!({"description":cut});
            if cut == "left" || cut == "right" {
                trim["side"] = json!(cut);
            }
            if let Some(run) = cut
                .strip_prefix("arm:")
                .and_then(|c| c.strip_suffix(":distal"))
            {
                trim["kind"] = json!("arm_end_trim");
                trim["source_run_id"] = json!(run);
                if let Some(arm) = block.arms.iter().find(|a| a.edge_id == run) {
                    trim["start_mm"] =
                        json!([mm(arm.start.x), mm(arm.start.y), mm(block.z_centimm)]);
                    trim["end_mm"] = json!([mm(arm.end.x), mm(arm.end.y), mm(block.z_centimm)]);
                    trim["length_mm"] = json!(mm(arm.length_centimm));
                }
            }
            trims.push(trim);
        }
    }
    // Одинаковое положение и грань упорядочиваются как FbProductCodeParser.
    tokens.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| {
                let order = |f| match f {
                    2 => 0,
                    1 => 1,
                    _ => 2,
                };
                order(a.1).cmp(&order(b.1))
            })
            .then_with(|| a.2.cmp(&b.2))
    });
    let mut code1 = format!("П{}", number(block.length_centimm));
    for (position, face, product) in &tokens {
        code1.push_str(&token(*face, *position, block.length_centimm, product));
    }
    let mut mirror: Vec<_> = tokens
        .iter()
        .map(|(p, f, t)| (block.length_centimm - p, 4 - f, t))
        .collect();
    mirror.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.1.cmp(&b.1))
            .then_with(|| a.2.cmp(b.2))
    });
    let mut code2 = format!("П{}", number(block.length_centimm));
    for (position, face, product) in mirror {
        code2.push_str(&token(face, position, block.length_centimm, product));
    }
    let walls: Vec<_> = request
        .wall_volumes
        .iter()
        .filter(|w| block.source_ids.contains(&format!("wall:{}", w.guid)))
        .collect();
    let wall_ids: BTreeSet<_> = walls.iter().map(|w| w.guid.clone()).collect();
    for wall in &walls {
        let dx = wall.end_xmm - wall.start_xmm;
        let dy = wall.end_ymm - wall.start_ymm;
        let axis_squared = dx * dx + dy * dy;
        if axis_squared <= 0.0 {
            continue;
        }
        let points = std::iter::once(block.start)
            .chain(std::iter::once(block.end))
            .chain(block.arms.iter().flat_map(|arm| [arm.start, arm.end]));
        let requires_trim = points.into_iter().any(|point| {
            let t = (((mm(point.x) - wall.start_xmm) * dx + (mm(point.y) - wall.start_ymm) * dy)
                / axis_squared)
                .clamp(0.0, 1.0);
            let bottom = wall.start_bottom_zmm + (wall.end_bottom_zmm - wall.start_bottom_zmm) * t;
            let top = wall.start_top_zmm + (wall.end_top_zmm - wall.start_top_zmm) * t;
            mm(block.z_centimm) < bottom - 1e-8
                || mm(block.z_centimm + profile.index_centimm) > top + 1e-8
        });
        if requires_trim {
            trims.push(json!({"kind":"wall_boundary","source_wall_id":wall.guid}));
        }
    }
    let width = walls
        .first()
        .map(|w| w.thickness_mm)
        .ok_or_else(|| issue(block, "Не найдена исходная стена детали"))?;
    if !width.is_finite() || width <= 0.0 || profile.index_centimm <= 0 {
        return Err(issue(block, "Некорректное сечение детали"));
    }
    let source_ids: BTreeSet<_> = block
        .source_ids
        .iter()
        .filter(|id| !id.starts_with("edge:"))
        .map(|id| {
            id.strip_prefix("wall:")
                .or_else(|| id.strip_prefix("opening:"))
                .or_else(|| id.strip_prefix("beam:"))
                .unwrap_or(id)
                .to_owned()
        })
        .collect();
    let angle = f64::from(block.rotation_deg).to_radians();
    let (sin, cos) = angle.sin_cos();
    let clean = |v: f64| if v.abs() < 1e-12 { 0.0 } else { v };
    let product_type = match block.product_key.as_deref() {
        Some(key) if key.starts_with("Type") => {
            format!("Тип {}", key.trim_start_matches("Type").replace('_', "."))
        }
        _ if block.is_bridge => "Перемычка".into(),
        _ if block.kind == "special_ordinary" => "Специальный".into(),
        _ => "Рядовой".into(),
    };
    Ok(
        json!({"id":block.id,"code1":code1,"code2":code2,"product_type":product_type,
        "placement":{"origin_mm":[mm(block.start.x),mm(block.start.y),mm(block.z_centimm)],
            "x_axis":[clean(cos),clean(sin),0.0],"y_axis":[clean(-sin),clean(cos),0.0],"z_axis":[0.0,0.0,1.0]},
        "length_mm":mm(block.length_centimm),"nominal_length_mm":mm(block.catalog_nominal_centimm.unwrap_or(block.length_centimm)),
        "width_mm":width,"height_mm":mm(profile.index_centimm),"course_index":block.course_index,
        "source_ids":source_ids,"wall_ids":wall_ids,"hide_spikes_left":block.hide_spikes_left,"hide_spikes_right":block.hide_spikes_right,
        "node_cuts":node_cuts,"trims":trims}),
    )
}

pub fn export_codes(
    standard: &ExchangeRequest,
    request: &LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
    warnings: &[ApiFailure],
) -> Result<Value, ApiFailure> {
    let mut ordered: Vec<_> = blocks.iter().collect();
    ordered.sort_by(|a, b| a.id.cmp(&b.id));
    let mut seen = BTreeSet::new();
    let mut values = Vec::new();
    for block in ordered {
        if !seen.insert(&block.id) {
            return Err(issue(block, "Повторный идентификатор детали"));
        }
        values.push(export_block(request, profile, block)?);
    }
    let mut result = exchange::success_result_with_warnings(standard, values, Vec::new(), warnings);
    result["format"] = json!("fb-layout-codes/1");
    result.as_object_mut().unwrap().remove("beam_adjustments");
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exchange::parse_request;

    fn block(id: &str, kind: &str, key: Option<&str>, cuts: Vec<&str>) -> Block {
        serde_json::from_value(json!({"id":id,"wall_id":"wall-example","edge_id":"run-example",
            "course_index":0,"z_centimm":-25200,"start":{"x":0,"y":0},"end":{"x":64000,"y":0},
            "length_centimm":64000,"kind":kind,"product_key":key,"catalog_status":"KnownPattern",
            "rotation_deg":0,"local_origin":null,"local_rotation_deg":null,"is_bridge":false,
            "hide_spikes_left":false,"hide_spikes_right":false,"cuts":cuts,
            "source_ids":["wall:wall-example","edge:wall-example:0"],"catalog_nominal_centimm":64000,"arms":[]})).unwrap()
    }

    #[test]
    fn actual_node_cut_tokens_survive_in_sup_codes_for_l_t_x_and_ordinary() {
        let standard = parse_request(include_str!(
            "../Документация/Граничные контракты/examples/layout-request.v1.json"
        ))
        .unwrap();
        let request = standard.to_layout_request().unwrap();
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let blocks = vec![
            block(
                "L",
                "node_L",
                Some("Type1"),
                vec!["Type1:x1:y1:run-example"],
            ),
            block(
                "T",
                "node_T",
                Some("Type6"),
                vec!["Type6:x2:y1:run-example"],
            ),
            block(
                "X",
                "node_X",
                Some("Type7_1"),
                vec!["Type6:x2:y1:run-example", "Type6:x2:y3:run-example"],
            ),
            block("ordinary", "ordinary", None, vec![]),
        ];
        let result = export_codes(&standard, &request, &profile, &blocks, &[]).unwrap();
        assert_eq!(result["format"], "fb-layout-codes/1");
        assert_eq!(result["kind"], "layout_result");
        assert_eq!(result["snapshot_hash"], standard.snapshot_hash);
        assert_eq!(
            result["coordinate_system"]["id"],
            standard.coordinate_system.id
        );
        assert_eq!(result["blocks"][0]["code1"], "П640 [Н-НЧ-Тип 1]");
        assert_eq!(result["blocks"][0]["code2"], "П640 [В-КН-Тип 1]");
        assert_eq!(result["blocks"][1]["code1"], "П640 [Н-320-Тип 6]");
        assert_eq!(
            result["blocks"][2]["code1"],
            "П640 [Н-320-Тип 6] [В-320-Тип 6]"
        );
        assert_eq!(
            result["blocks"][2]["node_cuts"].as_array().unwrap().len(),
            2
        );
        assert_eq!(result["blocks"][3]["code1"], "П640");
        let mut reversed = blocks.clone();
        reversed.reverse();
        assert_eq!(
            export_codes(&standard, &request, &profile, &reversed, &[]).unwrap(),
            result
        );
        for value in result["blocks"].as_array().unwrap() {
            for forbidden in [
                "stock_shape",
                "cuts",
                "geometry",
                "bodies",
                "planes_local",
                "vertices_mm",
            ] {
                assert!(value.get(forbidden).is_none());
            }
            assert_eq!(value["source_ids"], json!(["wall-example"]));
            assert_eq!(value["wall_ids"], json!(["wall-example"]));
        }
    }

    #[test]
    fn actual_trim_keeps_nominal_stock_length_and_declares_sup_wall_boundary_work() {
        let standard = parse_request(include_str!(
            "../Документация/Граничные контракты/examples/layout-request.v1.json"
        ))
        .unwrap();
        let mut request = standard.to_layout_request().unwrap();
        request.wall_volumes[0].start_top_zmm = -200.0;
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let mut ordinary = block("ordinary", "ordinary", None, vec!["right"]);
        ordinary.length_centimm = 57632;
        ordinary.hide_spikes_right = true;
        let result = export_codes(&standard, &request, &profile, &[ordinary], &[]).unwrap();
        let value = &result["blocks"][0];
        assert_eq!(value["code1"], "П576.32");
        assert_eq!(value["length_mm"], 576.32);
        assert_eq!(value["nominal_length_mm"], 640.0);
        assert_eq!(value["hide_spikes_right"], true);
        assert!(value["trims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|t| t["kind"] == "wall_boundary"));
    }
}

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

type Cut = (i64, u8, String);

fn end_extension(cuts: &[Cut], end: i64) -> i64 {
    if cuts.iter().any(|(pos, _, product)| *pos == end && matches!(product.as_str(), "1" | "2" | "3" | "4")) {
        crate::node_shapes::CORNER_EXTENSION_CENTIMM
    } else { 0 }
}

fn product_codes(length: i64, tokens: &mut [Cut]) -> (String, String) {
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
    let mut code1 = format!("П{}", number(length));
    for (position, face, product) in tokens.iter() {
        code1.push_str(&token(*face, *position, length, product));
    }
    let mut mirror: Vec<_> = tokens
        .iter()
        .map(|(p, f, t)| (length - p, 4 - f, t))
        .collect();
    mirror.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.1.cmp(&b.1))
            .then_with(|| a.2.cmp(b.2))
    });
    let mut code2 = format!("П{}", number(length));
    for (position, face, product) in mirror {
        code2.push_str(&token(face, position, length, product));
    }
    (code1, code2)
}

fn valid_length(length: i64) -> bool { (1..=100_000_000).contains(&length) }

pub(crate) fn validate_end_states(block: &Block) -> Result<(), ApiFailure> {
    if (!block.natural_end_left() && !block.hide_spikes_left())
        || (!block.natural_end_right() && !block.hide_spikes_right())
    {
        return Err(issue(block, "Искусственный торец не может иметь шипы"));
    }
    let insets = i64::from(block.natural_end_left() && block.hide_spikes_left())
        + i64::from(block.natural_end_right() && block.hide_spikes_right());
    if block.length_centimm <= insets * 500 {
        return Err(issue(block, "Снятие шипов поглощает всю длину детали"));
    }
    Ok(())
}

pub(crate) fn export_block(
    request: &LayoutRequest,
    profile: &Profile,
    block: &Block,
) -> Result<Value, ApiFailure> {
    if block.id.is_empty() || !valid_length(block.length_centimm) {
        return Err(issue(block, "Некорректная идентичность или длина детали"));
    }
    validate_end_states(block)?;
    let mut tokens = Vec::new();
    let mut node_cuts = Vec::new();
    let mut trims = Vec::new();
    for cut in &block.cuts {
        if cut.starts_with("Type") {
            let mut fields = cut.splitn(4, ':');
            let key = fields.next().unwrap_or("");
            let product = type_number(key).ok_or_else(|| issue(block, "Неизвестный тип врезки"))?;
            let location = fields.next().unwrap_or("");
            let (x, position) = if let Some(slot) = location.strip_prefix('x') {
                let x: i64 = slot
                    .parse()
                    .ok()
                    .filter(|x| *x > 0 && *x <= 3126)
                    .ok_or_else(|| issue(block, "Некорректная позиция x врезки"))?;
                (Some(x), (x - 1) * 32_000)
            } else if let Some(offset) = location.strip_prefix('p') {
                let position: i64 = offset
                    .parse()
                    .ok()
                    .filter(|p| *p >= 0 && *p <= block.length_centimm)
                    .ok_or_else(|| issue(block, "Некорректная координата врезки"))?;
                (None, position)
            } else {
                return Err(issue(block, "Некорректная позиция врезки"));
            };
            let y: u8 = fields
                .next()
                .and_then(|s| s.strip_prefix('y'))
                .and_then(|s| s.parse().ok())
                .filter(|y| (1..=3).contains(y))
                .ok_or_else(|| issue(block, "Некорректная грань y врезки"))?;
            let sources = fields
                .next()
                .ok_or_else(|| issue(block, "Не указано происхождение врезки"))?;
            // Legacy-сборка 8/8.1 хранит y до перестановки сторон GDL.
            // Канонический FbEdgeMatrix ждёт Тип 8 на Н-НЧ, Тип 8.1 на В-НЧ.
            let canonical_y = if matches!(key, "Type8" | "Type8_1") {
                4 - y
            } else {
                y
            };
            if position > block.length_centimm {
                return Err(issue(block, "Врезка за пределами детали"));
            }
            tokens.push((position, canonical_y, product));
            let mut node_cut = json!({"product_type":format!("Тип {}",key.trim_start_matches("Type").replace('_',".")),
                "y":canonical_y,"position_mm":mm(position),"face":match canonical_y {1=>"Н",2=>"С",_=>"В"},
                "source_run_ids":sources.split(',').collect::<Vec<_>>()});
            if let Some(x) = x {
                node_cut["x"] = json!(x);
            }
            node_cuts.push(node_cut);
        } else if cut.starts_with("opening_slope:") || cut.starts_with("opening_volume:") || cut.starts_with("beam_volume:") {
            return Err(issue(block, "Объёмный разрез не завершён до экспорта детали"));
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
    let (code1, code2) = product_codes(block.length_centimm, &mut tokens);
    let walls: Vec<_> = request
        .wall_volumes
        .iter()
        .filter(|w| block.source_ids.contains(&format!("wall:{}", w.guid)))
        .collect();
    let wall_ids: BTreeSet<_> = walls.iter().map(|w| w.guid.clone()).collect();
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
    // SUP принимает габарит с L-выступами и начало этого габарита.
    // Код и раскладочная сетка сохраняют длину тела между узлами.
    let [leading, trailing] = crate::node_shapes::corner_extensions(block);
    let begin_extension = mm(leading);
    let physical_length = mm(block.length_centimm + leading + trailing);
    let product_type = match block.product_key.as_deref() {
        _ if block.is_bridge => "Перемычка".into(),
        _ if block.kind == "node_compound" => "Комбинированный".into(),
        Some(key) if key.starts_with("Type") => {
            format!("Тип {}", key.trim_start_matches("Type").replace('_', "."))
        }
        _ if block.kind == "special_ordinary" => "Специальный".into(),
        _ => "Рядовой".into(),
    };
    let mut value = json!({"id":block.id,"code1":code1,"code2":code2,"product_type":product_type,
        "placement":{"origin_mm":[mm(block.start.x)-begin_extension*cos,mm(block.start.y)-begin_extension*sin,mm(block.z_centimm)],
            "x_axis":[clean(cos),clean(sin),0.0],"y_axis":[clean(-sin),clean(cos),0.0],"z_axis":[0.0,0.0,1.0]},
        "length_mm":physical_length,"nominal_length_mm":mm(block.length_centimm),
        "width_mm":width,"height_mm":mm(profile.index_centimm),"course_index":block.course_index,
        "source_ids":source_ids,"wall_ids":wall_ids,"hide_spikes_left":block.hide_spikes_left(),"hide_spikes_right":block.hide_spikes_right(),
        "natural_end_left":block.natural_end_left(),"natural_end_right":block.natural_end_right(),
        "left_end":block.ends.left(),"right_end":block.ends.right(),
        "node_cuts":node_cuts,"trims":trims});
    if !block.rectangular_cuts.is_empty() {
        value["rectangular_cuts"] = json!(block.rectangular_cuts);
    }
    Ok(value)
}

pub fn export_codes(
    standard: &ExchangeRequest,
    request: &LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
    warnings: &[ApiFailure],
) -> Result<Value, ApiFailure> {
    export_internal(standard,request,profile,blocks,warnings,true)
}

pub fn export_blocks(standard:&ExchangeRequest,request:&LayoutRequest,profile:&Profile,blocks:&[Block],warnings:&[ApiFailure])->Result<Value,ApiFailure> {
    export_internal(standard,request,profile,blocks,warnings,false)
}

fn export_internal(standard:&ExchangeRequest,request:&LayoutRequest,profile:&Profile,blocks:&[Block],warnings:&[ApiFailure],include_lamellas:bool)->Result<Value,ApiFailure> {
    let normalized = request.normalized();
    let request = &normalized;
    let prepared = crate::offcut_plan::prepare(request, profile, blocks)?;
    let mut ordered: Vec<_> = prepared.blocks.iter().collect();
    ordered.sort_by(|a, b| a.id.cmp(&b.id));
    let mut seen = BTreeSet::new();
    let mut values = Vec::new();
    for block in ordered {
        if !seen.insert(&block.id) {
            return Err(issue(block, "Повторный идентификатор детали"));
        }
        let mut value = export_block(request, profile, block)?;
        prepared.annotate(&mut value);
        values.push(value);
    }
    let lamella_plan = if include_lamellas {crate::lamella::calculate(request, profile, &prepared.blocks)?}
        else{crate::lamella::LamellaPlan {lamellas:Vec::new(),warnings:Vec::new()}};
    if include_lamellas {crate::lamella::add_recesses(&mut values, &prepared.blocks, &lamella_plan.lamellas, request);}
    let mut all_warnings = warnings.to_vec();
    all_warnings.extend(lamella_plan.warnings);
    let mut result = exchange::success_result_with_warnings(standard, values, crate::effective_geometry::beam_adjustments(request), &all_warnings);
    result["format"] = json!("fb-layout-codes/1");
    result["lamellas"] = serde_json::to_value(lamella_plan.lamellas).map_err(|e|
        ApiFailure::new("LAMELLA_EXPORT_FAILED", e.to_string(), None))?;
    crate::precision::normalize_result(&mut result);
    Ok(result)
}

/// Rebuilds product codes from machining facts, using the same formatter and end
/// geometry as the layout export. Each part has an independent outcome.
pub fn selected_codes(request: &Value) -> Result<Value, String> {
    let parts = request["blocks"].as_array().ok_or("Нет деталей для пересчёта")?;
    let results: Vec<_> = parts.iter().map(|part| {
        let id = &part["id"];
        match recode(part) {
            Ok(mut result) => { result["id"] = id.clone(); result }
            Err(message) => json!({"id":id,"error":message}),
        }
    }).collect();
    Ok(json!({"format":"fb-product-codes/1","request_id":request["request_id"],"blocks":results}))
}

fn recode(part: &Value) -> Result<Value, String> {
    fn units(value: &Value) -> Result<i64, String> {
        let n = crate::precision::centimm(value.as_f64().ok_or("Отсутствует размер в мм")?);
        if !n.is_finite() || n.abs() >= (i64::MAX / 4) as f64 { return Err("Размер вне диапазона модели".into()); }
        Ok(n as i64)
    }
    let nominal = units(&part["source_nominal_mm"])?;
    let actual = units(&part["length_mm"])?;
    let mut cuts = Vec::new();
    for cut in part["cuts"].as_array().ok_or("Нет списка врезок")? {
        let face = match cut["face"].as_str() {
            Some("Н") => 1, Some("С") => 2, Some("В") => 3, _ => return Err("Неизвестная грань врезки".into())
        };
        let key = cut["type"].as_str().ok_or("Нет типа врезки")?;
        let product = type_number(&format!("Type{key}")).ok_or("Неизвестный тип врезки")?;
        let position = units(&cut["position_mm"])?;
        if !(0..=nominal).contains(&position) { return Err("Исходная врезка вне оси детали".into()); }
        cuts.push((position, face, product));
    }
    let leading = end_extension(&cuts, 0);
    let trailing = end_extension(&cuts, nominal);
    let source_length = if part["source_length_mm"].is_null() {
        nominal + leading + trailing
    } else { units(&part["source_length_mm"])? };
    let length = nominal + actual - source_length;
    if nominal <= 0 || source_length <= 0 || actual <= 0 || !valid_length(length) {
        return Err("Некорректная длина детали".into());
    }
    if part["current_facts"] == true && actual != source_length {
        return Err("Номинальная длина и врезки не соответствуют габариту модели".into());
    }
    let shift = if !part["start_shift_mm"].is_null() { units(&part["start_shift_mm"])? }
        else if cuts.is_empty() || length == nominal { 0 }
        else { return Err("Для изменённой детали нужна исходная ось или текущие координаты врезок".into()); };
    let mut relocated = Vec::with_capacity(cuts.len());
    for (position, face, product) in cuts {
        let kind = format!("Type{}", product.replace('.', "_"));
        if let Some(p) = crate::node_shapes::relocated_position(&kind, position, shift, shift + length)? {
            relocated.push((p, face, product));
        }
    }
    let mut cuts = relocated;
    if leading != end_extension(&cuts, 0) || trailing != end_extension(&cuts, length) {
        return Err("Изменился угловой торец. Укажите номинальную длину и текущие врезки".into());
    }
    let (code1, code2) = product_codes(length, &mut cuts);
    Ok(json!({"code1":code1,"code2":code2,"nominal_length_mm":mm(length)}))
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
    fn selected_parts_match_full_export_and_relocate_the_console_joint() {
        let standard = parse_request(include_str!(
            "../Документация/Граничные контракты/examples/layout-request.v1.json"
        )).unwrap();
        let request = standard.to_layout_request().unwrap();
        let profile: Profile = serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let mut lintel = block("console", "lintel", None, vec!["Type6:p160000:y3:wall"]);
        lintel.length_centimm = 269650;
        lintel.is_bridge = true;
        let exported = export_block(&request, &profile, &lintel).unwrap();
        let mut facts = json!({"id":"console","source_nominal_mm":2696.5,"length_mm":2696.5,
            "cuts":[{"type":"6","face":"В","position_mm":1600}]});
        let before = recode(&facts).unwrap();
        assert_eq!(before["code1"], exported["code1"]);
        assert_eq!(before["code2"], exported["code2"]);
        facts["length_mm"] = json!(2675);
        assert!(recode(&facts).is_err());
        facts["start_shift_mm"] = json!(21.5);
        let after = recode(&facts).unwrap();
        assert_eq!(after["code1"], "П2675 [В-1578.5-Тип 6]");
        assert_eq!(after["code2"], "П2675 [Н-1096.5-Тип 6]");
        lintel.length_centimm = 267500;
        lintel.cuts = vec!["Type6:p157850:y3:wall".to_string()];
        let full = export_block(&request, &profile, &lintel).unwrap();
        assert_eq!(after["code1"], full["code1"]);
        assert_eq!(after["code2"], full["code2"]);
        let neighbor = json!({"id":"neighbor","source_nominal_mm":640,"length_mm":661.5,"cuts":[]});
        let invalid = json!({"id":"invalid","source_nominal_mm":640,"length_mm":600,"start_shift_mm":40,
            "cuts":[{"position_mm":10,"type":"6","face":"Н"}]});
        let response = selected_codes(&json!({"blocks":[facts,invalid,neighbor]})).unwrap();
        assert!(response["blocks"][1]["error"].is_string());
        assert_eq!(response["blocks"][2]["code1"], "П661.5");
    }

    #[test]
    fn relocated_node_cut_keeps_coordinate_and_refuses_lost_partial_profile() {
        let mut item = block("part", "ordinary", None, vec!["Type6:x2:y1:run-example"]);
        crate::node_shapes::relocate_node_cuts(&mut item, 16000, 64000).unwrap();
        assert_eq!(item.cuts, vec!["Type6:p16000:y1:run-example"]);
        let mut crossing = block("part", "ordinary", None, vec!["Type6:x2:y1:run-example"]);
        let error = crate::node_shapes::relocate_node_cuts(&mut crossing, 35000, 64000).unwrap_err();
        assert_eq!(error.code, "NODE_CUT_CROSSES_SPLIT");
        let mut end_profile = block("part", "node_L", Some("Type1"), vec!["Type1:x1:y1:run-example"]);
        assert_eq!(crate::node_shapes::relocate_node_cuts(&mut end_profile, 5000, 64000).unwrap_err().code,
            "NODE_CUT_CROSSES_SPLIT");
    }

    #[test]
    fn bridge_preserves_long_cut_positions_without_late_volume_trims() {
        let standard = parse_request(include_str!(
            "../Документация/Граничные контракты/examples/layout-request.v1.json"
        ))
        .unwrap();
        let mut request = standard.to_layout_request().unwrap();
        let mut opening = request.wall_volumes[0].clone();
        opening.guid = "opening-example".into();
        request.opening_volumes.push(opening);
        request.beams.push(serde_json::from_value(json!({"guid":"beam-example",
            "startXmm":0,"startYmm":0,"startZmm":-252,"endXmm":1920,"endYmm":0,"endZmm":-252,
            "geometry":{"widthMm":160,"heightMm":315,"heightDirectionX":0,"heightDirectionY":0,"heightDirectionZ":1}})).unwrap());
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let mut bridge = block(
            "bridge",
            "lintel",
            Some("Type7_1"),
            vec!["Type6:x5:y1:run-example", "Type6:x5:y3:run-example"],
        );
        bridge.length_centimm = 192000;
        bridge.is_bridge = true;
        bridge.catalog_nominal_centimm = Some(192000);
        bridge.source_ids.push("opening:opening-example".into());
        let result = export_block(&request, &profile, &bridge).unwrap();
        assert_eq!(result["product_type"], "Перемычка");
        assert_eq!(result["code1"], "П1920 [Н-1280-Тип 6] [В-1280-Тип 6]");
        assert_eq!(result["code2"], "П1920 [Н-640-Тип 6] [В-640-Тип 6]");
        assert_eq!(result["node_cuts"][0]["position_mm"], 1280.0);
        assert_eq!(result["nominal_length_mm"], 1920.0);
        assert_eq!(
            result["source_ids"],
            json!(["opening-example", "wall-example"])
        );
        assert!(!result["trims"].as_array().unwrap().iter().any(|trim|
            matches!(trim["kind"].as_str(), Some("beam_volume" | "opening_volume"))));
        bridge.cuts = vec![
            "Type6:p128012:y1:run-example".into(),
            "Type6:p128012:y3:run-example".into(),
        ];
        let arbitrary = export_block(&request, &profile, &bridge).unwrap();
        assert_eq!(
            arbitrary["code1"],
            "П1920 [Н-1280.12-Тип 6] [В-1280.12-Тип 6]"
        );
        assert_eq!(
            arbitrary["code2"],
            "П1920 [Н-639.88-Тип 6] [В-639.88-Тип 6]"
        );
        assert_eq!(arbitrary["node_cuts"][0]["position_mm"], 1280.12);
        assert!(arbitrary["node_cuts"][0].get("x").is_none());
        for cut in ["beam_volume:beam-example", "opening_volume:opening-example"] {
            bridge.cuts = vec![cut.into()];
            assert!(export_block(&request, &profile, &bridge).is_err());
        }
    }

    #[test]
    fn actual_node_cut_tokens_survive_in_sup_codes_for_l_t_x_and_ordinary() {
        let standard = parse_request(include_str!(
            "../Документация/Граничные контракты/examples/layout-request.v1.json"
        ))
        .unwrap();
        let mut request = standard.to_layout_request().unwrap();
        request.wall_volumes[0].end_xmm = 640.0;
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
    fn l_corner_exports_sup_extent_without_moving_the_nominal_grid() {
        let standard = parse_request(include_str!(
            "../Документация/Граничные контракты/examples/layout-request.v1.json"
        ))
        .unwrap();
        let request = standard.to_layout_request().unwrap();
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        for rotation in [0, 90, 180, 270] {
            for (product, face) in [("Type1", 1), ("Type2", 3), ("Type3", 3), ("Type4", 1)] {
                let mut corner = block("L", "node_L", Some(product), vec![]);
                corner.rotation_deg = rotation;
                corner.start = crate::domain::Point {
                    x: 100000,
                    y: 200000,
                };
                corner.cuts = vec![format!("{product}:x1:y{face}:run-example")];
                let value = export_block(&request, &profile, &corner).unwrap();
                assert_eq!(value["length_mm"], 736.5);
                assert_eq!(value["nominal_length_mm"], 640.0);
                assert!(value["code1"].as_str().unwrap().starts_with("П640 "));
                // SUP сдвигает начало тела от origin на 96.5 мм.
                // После преобразования его начало обязано совпасть с узлом.
                for axis in 0..2 {
                    let origin = value["placement"]["origin_mm"][axis].as_f64().unwrap();
                    let direction = value["placement"]["x_axis"][axis].as_f64().unwrap();
                    assert!((origin + 96.5 * direction - [1000.0, 2000.0][axis]).abs() < 1e-8);
                }
            }
        }
        let mut both = block(
            "L",
            "node_L",
            Some("Type1"),
            vec!["Type1:x1:y1:run-example", "Type1:x3:y3:run-example"],
        );
        let value = export_block(&request, &profile, &both).unwrap();
        assert_eq!(value["length_mm"], 833.0);
        both.cuts = vec!["Type1:x3:y3:run-example".into()];
        let value = export_block(&request, &profile, &both).unwrap();
        assert_eq!(value["length_mm"], 736.5);
        assert_eq!(value["placement"]["origin_mm"], json!([0.0, 0.0, -252.0]));
    }

    #[test]
    fn actual_trim_exports_finished_length_without_sup_wall_geometry() {
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
        ordinary.ends = crate::end_state::EndStates::from_flags(ordinary.hide_spikes_left(), true, ordinary.natural_end_left(), ordinary.natural_end_right());
        let result = export_codes(&standard, &request, &profile, &[ordinary], &[]).unwrap();
        let value = &result["blocks"][0];
        assert_eq!(value["code1"], "П576.32");
        assert_eq!(value["length_mm"], 576.32);
        assert_eq!(value["nominal_length_mm"], 576.32);
        assert_eq!(value["hide_spikes_right"], true);
        assert!(value["trims"]
            .as_array()
            .unwrap()
            .iter()
            .all(|t| t["kind"] != "wall_boundary"));
    }

    #[test]
    fn legacy_type8_faces_are_converted_before_both_canonical_views() {
        let standard = parse_request(include_str!(
            "../Документация/Граничные контракты/examples/layout-request.v1.json"
        ))
        .unwrap();
        let mut request = standard.to_layout_request().unwrap();
        request.wall_volumes[0].end_xmm = 640.0;
        let profile: Profile =
            serde_json::from_str(include_str!("../profiles/banya-prototype.json")).unwrap();
        let blocks = [
            block(
                "8",
                "node_T",
                Some("Type8"),
                vec!["Type8:x1:y3:run-example"],
            ),
            block(
                "8.1",
                "node_T",
                Some("Type8_1"),
                vec!["Type8_1:x1:y1:run-example"],
            ),
        ];
        let result = export_codes(&standard, &request, &profile, &blocks, &[]).unwrap();
        assert_eq!(result["blocks"][0]["code1"], "П640 [Н-НЧ-Тип 8]");
        assert_eq!(result["blocks"][0]["code2"], "П640 [В-КН-Тип 8]");
        assert_eq!(result["blocks"][0]["node_cuts"][0]["y"], 1);
        assert_eq!(result["blocks"][1]["code1"], "П640 [В-НЧ-Тип 8.1]");
        assert_eq!(result["blocks"][1]["code2"], "П640 [Н-КН-Тип 8.1]");
        assert_eq!(result["blocks"][1]["node_cuts"][0]["face"], "В");
    }
}

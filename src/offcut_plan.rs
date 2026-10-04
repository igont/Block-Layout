//! Подтверждённые пары рядовых деталей из одной заготовки. Геометрия установки
//! сохраняется; пропил и оставшийся материал проверяются в координатах заготовки.
use crate::api::{ApiFailure, LayoutRequest};
use crate::layout::{Block, Profile};
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

const STOCK: i64 = 64_000;
const KERF: i64 = 500;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum End {
    Left,
    Right,
    Flat,
}

#[derive(Clone)]
struct Candidate {
    index: usize,
    length: i64,
    end: End,
    zone_ids: Vec<String>,
    side: Option<bool>,
    receives_offcut: bool,
    short: bool,
    typed_stock: Option<String>,
    stock_first: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct StockPart {
    pub block_id: String,
    pub interval_mm: [f64; 2],
    pub stock_end: &'static str,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub reversed: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct StockCutPlan {
    pub stock_code: String,
    pub stock_length_mm: f64,
    pub kerf_mm: f64,
    pub parts: Vec<StockPart>,
    pub saw_cuts_mm: Vec<[f64; 2]>,
    pub waste_intervals_mm: Vec<[f64; 2]>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DoborSide { Negative, Positive }

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct DoborGroup {
    pub source_id: String,
    pub side: DoborSide,
}

pub struct PreparedParts {
    pub blocks: Vec<Block>,
    pub parents: BTreeMap<String, String>,
    pub dobor_groups: BTreeMap<String, DoborGroup>,
    pub stock_plans: BTreeMap<String, StockCutPlan>,
    pub zones: BTreeMap<String, Vec<String>>,
}

impl PreparedParts {
    pub fn annotate(&self, value: &mut Value) {
        let Some(id) = value.get("id").and_then(Value::as_str).map(str::to_owned) else {
            return;
        };
        value["is_dobor"] = json!(self.dobor_groups.contains_key(&id));
        value["cut_from_block_id"] = json!(self.parents.get(&id));
        if let Some(group) = self.dobor_groups.get(&id) {
            value["dobor_group"] = json!(group);
        }
        if let Some(zones) = self.zones.get(&id) {
            value["cut_zone_ids"] = json!(zones);
        }
        if let Some(plan) = self.stock_plans.get(&id) {
            value["stock_cut_plan"] = json!(plan);
        }
    }
}

fn obstacle_zones(block: &Block, request: &LayoutRequest) -> Vec<String> {
    block
        .obstacle_ends()
        .iter()
        .filter(|end| {
            if end.left {
                block.hide_spikes_left()
            } else {
                block.hide_spikes_right()
            }
        })
        .filter_map(|end| {
            let source = &end.source_id;
            if let Some(id) = source.strip_prefix("opening:") {
                request
                    .opening_volumes
                    .iter()
                    .any(|o| {
                        o.guid == id
                            && matches!(o.opening_type.as_str(), "OPENING" | "WINDOW" | "DOOR")
                    })
                    .then(|| id.to_owned())
            } else if let Some(id) = source.strip_prefix("beam:") {
                request
                    .beams
                    .iter()
                    .any(|b| b.guid == id)
                    .then(|| id.to_owned())
            } else {
                None
            }
        })
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

fn section(block: &Block, request: &LayoutRequest, profile: &Profile) -> Option<(i64, i64)> {
    let mut width = None;
    for wall in request
        .wall_volumes
        .iter()
        .filter(|w| block.source_ids.contains(&format!("wall:{}", w.guid)))
    {
        if !wall.thickness_mm.is_finite() || wall.thickness_mm <= 0.0 {
            return None;
        }
        let scaled = (wall.thickness_mm * 100.0).round() as i64;
        if width.is_some_and(|other| other != scaled) {
            return None;
        }
        width = Some(scaled);
        let (dx, dy) = (wall.end_xmm - wall.start_xmm, wall.end_ymm - wall.start_ymm);
        let squared = dx * dx + dy * dy;
        if squared <= 1e-8 {
            return None;
        }
        // Скосы и неполные по высоте фрагменты нельзя подменять прямым остатком.
        for point in [block.start, block.end] {
            let t = (((point.x as f64 / 100.0 - wall.start_xmm) * dx
                + (point.y as f64 / 100.0 - wall.start_ymm) * dy)
                / squared)
                .clamp(0.0, 1.0);
            let bottom = wall.start_bottom_zmm + t * (wall.end_bottom_zmm - wall.start_bottom_zmm);
            let top = wall.start_top_zmm + t * (wall.end_top_zmm - wall.start_top_zmm);
            if block.z_centimm as f64 / 100.0 < bottom - 1e-8
                || (block.z_centimm + profile.index_centimm) as f64 / 100.0 > top + 1e-8
            {
                return None;
            }
        }
    }
    width.map(|w| (w, profile.index_centimm))
}

fn candidate(
    index: usize,
    block: &Block,
    request: &LayoutRequest,
    profile: &Profile,
) -> Option<((i64, i64), Candidate)> {
    let typed = block.kind.starts_with("node_") && matches!(block.product_key.as_deref(),
        Some("Type5_1" | "Type6" | "Type7_1" | "Type8" | "Type8_1"));
    if block.is_bridge || !block.components.is_empty()
        || !typed && (block.kind != "ordinary" || block.product_key.is_some()
            || !block.arms.is_empty() || !block.cuts.is_empty())
        || typed && block.cuts.iter().any(|cut| !cut.starts_with("Type")) {
        return None;
    }
    let zone_ids = obstacle_zones(block, request);
    if zone_ids.is_empty() {
        return None;
    }
    // Сторона определяется геометрией стены, а не GUID или длиной детали.
    // Выбираем направление оси с положительным X (для вертикальной - Y).
    let dx = block.end.x - block.start.x;
    let dy = block.end.y - block.start.y;
    let forward = dx > 0 || dx == 0 && dy > 0;
    let sides: BTreeSet<bool> = block
        .obstacle_ends()
        .iter()
        .filter(|e| {
            zone_ids.iter().any(|id| {
                e.source_id == format!("opening:{id}") || e.source_id == format!("beam:{id}")
            })
        })
        .map(|e| e.left == forward)
        .collect();
    // Между двумя проёмами сторона неоднозначна: такие детали могут быть
    // основными, но не получают добор, нарушающий выбранную сторону соседей.
    let side = (sides.len() == 1).then(|| *sides.first().unwrap());
    let left = block.natural_end_left() && !block.hide_spikes_left();
    let right = block.natural_end_right() && !block.hide_spikes_right();
    let end = match (left, right) {
        (true, false) => End::Left,
        (false, true) => End::Right,
        (false, false) => End::Flat,
        (true, true) => return None,
    };
    let inset = i64::from(block.natural_end_left() && block.hide_spikes_left())
        + i64::from(block.natural_end_right() && block.hide_spikes_right());
    let length = block.length_centimm - inset * KERF;
    // Длинная часть тоже может оставить пригодный обрезок.
    if length <= 0 || length >= STOCK {
        return None;
    }
    let stock_first = block.natural_end_left() && !block.hide_spikes_left();
    let typed_stock = if typed {
        // Одна наружная заводская грань фиксирует место детали в заготовке.
        if !stock_first && !(block.natural_end_right() && !block.hide_spikes_right()) { return None; }
        let offset = if stock_first { 0 } else { STOCK - block.length_centimm };
        let kept = (offset, offset + length);
        let mut stock = block.clone();
        stock.length_centimm = STOCK;
        stock.arms.clear();
        stock.cuts = block.cuts.iter().map(|cut| {
            let fields: Vec<_> = cut.splitn(4, ':').collect();
            let position = fields.get(1).and_then(|p| p.strip_prefix('p').and_then(|v| v.parse::<i64>().ok())
                .or_else(|| p.strip_prefix('x').and_then(|v| v.parse::<i64>().ok()).map(|v| (v-1)*32000)))?;
            Some(format!("{}:p{}:{}:{}", fields[0], position + offset, fields.get(2)?, fields.get(3)?))
        }).collect::<Option<Vec<_>>>()?;
        if !crate::node_shapes::plain_offcut(&stock, kept) { return None; }
        let code = crate::code_result::export_block(request, profile, &stock).ok()?;
        Some(code.get("code1")?.as_str()?.to_owned())
    } else { None };
    Some((
        section(block, request, profile)?,
        Candidate {
            index,
            length,
            end,
            zone_ids,
            side,
            receives_offcut: false,
            short: !typed && block.length_centimm <= STOCK - profile.minimum_cut_centimm,
            typed_stock,
            stock_first,
        },
    ))
}

fn mm(interval: (i64, i64)) -> [f64; 2] {
    [interval.0 as f64 / 100.0, interval.1 as f64 / 100.0]
}

fn assign(prepared: &mut PreparedParts, a: &Candidate, b: &Candidate) {
    let typed = a.typed_stock.is_some();
    let (first, second) = if typed {
        if a.stock_first { (a, b) } else { (b, a) }
    } else if a.end == End::Right || b.end == End::Left {
        (b, a)
    } else {
        (a, b)
    };
    let first_start = if typed && (first.index == a.index || first.end != End::Flat)
        || !typed && first.end == End::Left { 0 } else { KERF };
    let second_end = if typed && (second.index == a.index || second.end != End::Flat)
        || !typed && second.end == End::Right { STOCK } else { STOCK - KERF };
    let first_interval = (first_start, first_start + first.length);
    let second_interval = (second_end - second.length, second_end);
    let mut cuts = Vec::new();
    if first_start > 0 {
        cuts.push((0, KERF));
    }
    cuts.push((first_interval.1, first_interval.1 + KERF));
    cuts.push((second_interval.0 - KERF, second_interval.0));
    if second_end < STOCK {
        cuts.push((STOCK - KERF, STOCK));
    }
    cuts.sort();
    cuts.dedup();
    // Пропилы могут частично перекрываться при подгонке остатка. Учитываем
    // объединение удалённого материала, а не дважды списанный участок.
    let mut occupied = vec![first_interval, second_interval];
    occupied.extend(cuts.iter().copied());
    occupied.sort();
    let mut cursor = 0;
    let mut waste = Vec::new();
    for (start, end) in occupied {
        if start > cursor {
            waste.push((cursor, start));
        }
        cursor = cursor.max(end);
    }
    if cursor < STOCK {
        waste.push((cursor, STOCK));
    }
    let parent = a.index;
    let child = b.index;
    let parent_id = prepared.blocks[parent].id.clone();
    prepared
        .parents
        .insert(prepared.blocks[child].id.clone(), parent_id.clone());
    for candidate in [a, b] {
        prepared.zones.insert(
            prepared.blocks[candidate.index].id.clone(),
            candidate.zone_ids.clone(),
        );
    }
    let parts = [(first, first_interval), (second, second_interval)]
        .into_iter()
        .map(|(c, interval)| StockPart {
            block_id: prepared.blocks[c.index].id.clone(),
            interval_mm: mm(interval),
            reversed: typed && c.index == b.index && c.end != End::Flat
                && ((c.end == End::Left) != (interval.0 == 0)),
            stock_end: if typed {
                if interval.0 == 0 { "left" } else if interval.1 == STOCK { "right" } else { "none" }
            } else { match c.end {
                End::Left => "left",
                End::Right => "right",
                End::Flat => "none",
            } },
        })
        .collect();
    prepared.stock_plans.insert(
        parent_id,
        StockCutPlan {
            stock_code: a.typed_stock.clone().unwrap_or_else(|| "П640".into()),
            stock_length_mm: 640.0,
            kerf_mm: 5.0,
            parts,
            saw_cuts_mm: cuts.into_iter().map(mm).collect(),
            waste_intervals_mm: waste.into_iter().map(mm).collect(),
        },
    );
}

fn fits(source: &Candidate, child: &Candidate) -> bool {
    let flat_ends = i64::from(source.typed_stock.is_none() && source.end == End::Flat) + i64::from(child.end == End::Flat);
    source.index != child.index
        && (source.typed_stock.is_some() || source.end != child.end || source.end == End::Flat)
        && source.length + child.length <= STOCK - KERF * (1 + flat_ends)
}

fn select_sides(groups: &mut BTreeMap<(i64, i64), Vec<Candidate>>, request: &LayoutRequest) {
    // Membership uses compatible classes on the opposite side, not the number
    // of stock pieces left after matching individual physical pairs.
    let mut eligible = BTreeSet::new();
    let mut scores: BTreeMap<String, [bool; 2]> = BTreeMap::new();
    for candidates in groups.values() {
        for child in candidates {
            if !child.short || child.zone_ids.len() != 1 { continue; }
            let Some(side) = child.side else { continue; };
            if candidates.iter().any(|source| opposite_source(source, child)) {
                eligible.insert(child.index);
                scores.entry(child.zone_ids[0].clone()).or_default()[usize::from(side)] = true;
            }
        }
    }
    let mut selected = BTreeMap::new();
    for (zone, score) in &scores {
        let high_side = request.opening_volumes.iter().find(|o| &o.guid == zone)
            .filter(|o| (o.start_top_zmm - o.end_top_zmm).abs() > 1e-8)
            .map(|o| {
                let forward = o.end_xmm > o.start_xmm
                    || o.end_xmm == o.start_xmm && o.end_ymm > o.start_ymm;
                (o.end_top_zmm > o.start_top_zmm) == forward
            });
        let side = high_side.filter(|&side| score[usize::from(side)])
            .unwrap_or(score[1]);
        selected.insert(zone.clone(), side);
    }
    for child in groups.values_mut().flatten() {
        child.receives_offcut = eligible.contains(&child.index) && child.side.is_some_and(|side| {
            child.zone_ids.iter().all(|zone| selected.get(zone) == Some(&side))
        });
    }
}

fn opposite_source(source: &Candidate, child: &Candidate) -> bool {
    source.side.is_some() && source.side != child.side
        && source.zone_ids.iter().any(|zone| child.zone_ids.contains(zone))
        && fits(source, child)
}

fn match_group(prepared: &mut PreparedParts, candidates: &[Candidate]) {
    let mut available: BTreeSet<usize> = (0..candidates.len()).collect();
    let mut children: Vec<_> = candidates.iter().enumerate()
        .filter(|(_, c)| c.receives_offcut).collect();
    // Крупные получатели первыми: маленький не забирает единственный
    // остаток, из которого можно выпилить более длинную деталь.
    children.sort_by_key(|(_, c)| (
        &c.zone_ids, std::cmp::Reverse(c.length), &prepared.blocks[c.index].id,
    ));
    for (child_index, child) in children {
        if !available.contains(&child_index) { continue; }
        let source_index = available.iter().copied()
            .filter(|&i| !candidates[i].receives_offcut && opposite_source(&candidates[i], child))
            .min_by_key(|&i| (
                STOCK - candidates[i].length - child.length,
                &prepared.blocks[candidates[i].index].id,
            ));
        if let Some(i) = source_index {
            available.remove(&i);
            available.remove(&child_index);
            assign(prepared, &candidates[i], child);
        }
    }
}

pub fn prepare(
    request: &LayoutRequest,
    profile: &Profile,
    blocks: &[Block],
) -> Result<PreparedParts, ApiFailure> {
    // Geometry is finalized by the layout owner before classification.
    let blocks = blocks.to_vec();
    let mut ids = BTreeSet::new();
    let mut ambiguous_ids = BTreeSet::new();
    for block in &blocks {
        crate::code_result::validate_end_states(block)?;
        if !ids.insert(&block.id) {
            ambiguous_ids.insert(block.id.clone());
        }
    }
    let mut prepared = PreparedParts {
        blocks,
        parents: BTreeMap::new(),
        dobor_groups: BTreeMap::new(),
        stock_plans: BTreeMap::new(),
        zones: BTreeMap::new(),
    };
    // Каталог подтверждает единственную рядовую заводскую заготовку П640.
    // Сеточная половина 320 мм не становится самостоятельной заготовкой.
    if profile.ordinary_length_centimm != STOCK {
        return Ok(prepared);
    }
    let mut groups: BTreeMap<(i64, i64), Vec<Candidate>> = BTreeMap::new();
    for (index, block) in prepared.blocks.iter().enumerate() {
        // Диагностический материализатор допускает повтор временных сборок;
        // кодовый экспорт отдельно отвергает повтор ID. Ссылки здесь однозначны.
        if ambiguous_ids.contains(&block.id) {
            continue;
        }
        if let Some((section, candidate)) = candidate(index, block, request, profile) {
            groups.entry(section).or_default().push(candidate);
        }
    }
    select_sides(&mut groups, request);
    for candidate in groups.values().flatten() {
        let id = &prepared.blocks[candidate.index].id;
        prepared.zones.insert(id.clone(), candidate.zone_ids.clone());
        if candidate.receives_offcut {
            prepared.dobor_groups.insert(id.clone(), DoborGroup {
                source_id: candidate.zone_ids[0].clone(),
                side: if candidate.side == Some(true) { DoborSide::Positive } else { DoborSide::Negative },
            });
        }
    }
    for candidates in groups.values() {
        match_group(&mut prepared, candidates);
    }
    Ok(prepared)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_request() -> LayoutRequest {
        serde_json::from_value(json!({"schema_version":1,"z0_mm":0,"wall_volumes":[]})).unwrap()
    }

    fn part(index: usize, length: i64, side: bool) -> Candidate {
        Candidate {
            index,
            length,
            end: if side { End::Right } else { End::Left },
            zone_ids: vec!["window".into()],
            side: Some(side),
            receives_offcut: false,
            short: true,
            typed_stock: None,
            stock_first: false,
        }
    }

    #[test]
    fn incompatible_classes_do_not_establish_a_dobor_side() {
        let mut groups = BTreeMap::from([(
            (19_300, 6300),
            vec![
                part(0, 40_000, false),
                part(1, 40_000, false),
                part(2, 24_000, true),
            ],
        )]);
        select_sides(&mut groups, &empty_request());
        let candidates = &groups[&(19_300, 6300)];
        assert!(!candidates[0].receives_offcut);
        assert!(!candidates[1].receives_offcut);
        assert!(!candidates[2].receives_offcut);
        assert!(!candidates.iter().any(|source| fits(source, &candidates[0])));
    }

    #[test]
    fn one_opening_keeps_the_same_side_across_different_sections() {
        let mut groups = BTreeMap::from([
            ((19_300, 6300), vec![part(0, 30_600, false), part(3, 30_600, true)]),
            (
                (16_000, 6300),
                vec![part(1, 30_600, true), part(2, 30_600, true), part(4, 30_600, false)],
            ),
        ]);
        select_sides(&mut groups, &empty_request());
        assert!(!groups[&(19_300, 6300)][0].receives_offcut);
        assert!(groups[&(16_000, 6300)][..2].iter().all(|c| c.receives_offcut));
        assert!(!groups[&(16_000, 6300)][2].receives_offcut);
    }
}

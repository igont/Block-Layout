//! Подтверждённые пары рядовых деталей из одной заготовки. Геометрия установки
//! сохраняется; пропил и оставшийся материал проверяются в координатах заготовки.
use crate::api::{ApiFailure, LayoutRequest};
use crate::layout::{Block, Profile};
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::ops::Bound;

const STOCK: i64 = 64_000;
const KERF: i64 = 500;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum End { Left, Right, Flat }

#[derive(Clone)]
struct Candidate { index: usize, length: i64, end: End, zone_ids: Vec<String> }

#[derive(Clone, Debug, Serialize)]
pub struct StockPart {
    pub block_id: String,
    pub interval_mm: [f64; 2],
    pub stock_end: &'static str,
}

#[derive(Clone, Debug, Serialize)]
pub struct StockCutPlan {
    pub stock_code: &'static str,
    pub stock_length_mm: f64,
    pub kerf_mm: f64,
    pub parts: Vec<StockPart>,
    pub saw_cuts_mm: Vec<[f64; 2]>,
    pub waste_intervals_mm: Vec<[f64; 2]>,
}

pub struct PreparedParts {
    pub blocks: Vec<Block>,
    pub parents: BTreeMap<String, String>,
    pub stock_plans: BTreeMap<String, StockCutPlan>,
    pub zones: BTreeMap<String, Vec<String>>,
}

impl PreparedParts {
    pub fn annotate(&self, value: &mut Value) {
        let Some(id) = value.get("id").and_then(Value::as_str).map(str::to_owned) else { return; };
        value["is_dobor"] = json!(self.parents.contains_key(&id));
        value["cut_from_block_id"] = json!(self.parents.get(&id));
        if let Some(zones) = self.zones.get(&id) { value["cut_zone_ids"] = json!(zones); }
        if let Some(plan) = self.stock_plans.get(&id) { value["stock_cut_plan"] = json!(plan); }
    }
}

fn obstacle_zones(block: &Block, request: &LayoutRequest) -> Vec<String> {
    block.obstacle_ends.iter().filter(|end| if end.left { block.hide_spikes_left }
        else { block.hide_spikes_right }).filter_map(|end| {
        let source = &end.source_id;
        if let Some(id) = source.strip_prefix("opening:") {
            request.opening_volumes.iter().any(|o| o.guid == id
                && matches!(o.opening_type.as_str(), "OPENING" | "WINDOW" | "DOOR")).then(|| id.to_owned())
        } else if let Some(id) = source.strip_prefix("beam:") {
            request.beams.iter().any(|b| b.guid == id).then(|| id.to_owned())
        } else { None }
    }).collect::<BTreeSet<_>>().into_iter().collect()
}

fn section(block: &Block, request: &LayoutRequest, profile: &Profile) -> Option<(i64, i64)> {
    let mut width = None;
    for wall in request.wall_volumes.iter().filter(|w| block.source_ids.contains(&format!("wall:{}", w.guid))) {
        if !wall.thickness_mm.is_finite() || wall.thickness_mm <= 0.0 { return None; }
        let scaled = (wall.thickness_mm * 100.0).round() as i64;
        if width.is_some_and(|other| other != scaled) { return None; }
        width = Some(scaled);
        let (dx, dy) = (wall.end_xmm - wall.start_xmm, wall.end_ymm - wall.start_ymm);
        let squared = dx * dx + dy * dy;
        if squared <= 1e-8 { return None; }
        // Скосы и неполные по высоте фрагменты нельзя подменять прямым остатком.
        for point in [block.start, block.end] {
            let t = (((point.x as f64 / 100.0 - wall.start_xmm) * dx
                + (point.y as f64 / 100.0 - wall.start_ymm) * dy) / squared).clamp(0.0, 1.0);
            let bottom = wall.start_bottom_zmm + t * (wall.end_bottom_zmm - wall.start_bottom_zmm);
            let top = wall.start_top_zmm + t * (wall.end_top_zmm - wall.start_top_zmm);
            if block.z_centimm as f64 / 100.0 < bottom - 1e-8
                || (block.z_centimm + profile.index_centimm) as f64 / 100.0 > top + 1e-8 { return None; }
        }
    }
    width.map(|w| (w, profile.index_centimm))
}

fn candidate(index: usize, block: &Block, request: &LayoutRequest, profile: &Profile)
    -> Option<((i64, i64), Candidate)>
{
    if block.kind != "ordinary" || block.is_bridge || block.product_key.is_some()
        || !block.components.is_empty() || !block.arms.is_empty()
        || !block.cuts.is_empty() { return None; }
    let zone_ids = obstacle_zones(block, request);
    if zone_ids.is_empty() { return None; }
    let left = block.natural_end_left && !block.hide_spikes_left;
    let right = block.natural_end_right && !block.hide_spikes_right;
    let end = match (left, right) {
        (true, false) => End::Left,
        (false, true) => End::Right,
        (false, false) => End::Flat,
        (true, true) => return None,
    };
    let inset = i64::from(block.natural_end_left && block.hide_spikes_left)
        + i64::from(block.natural_end_right && block.hide_spikes_right);
    let length = block.length_centimm - inset * KERF;
    if length <= 0 || length >= STOCK { return None; }
    Some((section(block, request, profile)?, Candidate { index, length, end, zone_ids }))
}

fn normalize(block: &mut Block) {
    let left = if block.natural_end_left && block.hide_spikes_left { KERF } else { 0 };
    let right = if block.natural_end_right && block.hide_spikes_right { KERF } else { 0 };
    let angle = f64::from(block.rotation_deg).to_radians();
    let shift = |point: &mut crate::domain::Point, distance: i64| {
        point.x += (distance as f64 * angle.cos()).round() as i64;
        point.y += (distance as f64 * angle.sin()).round() as i64;
    };
    shift(&mut block.start, left);
    shift(&mut block.end, -right);
    block.length_centimm -= left + right;
    block.catalog_nominal_centimm = Some(block.length_centimm);
    if block.hide_spikes_left { block.natural_end_left = false; }
    if block.hide_spikes_right { block.natural_end_right = false; }
}

fn mm(interval: (i64, i64)) -> [f64; 2] { [interval.0 as f64 / 100.0, interval.1 as f64 / 100.0] }

fn assign(prepared: &mut PreparedParts, a: &Candidate, b: &Candidate) {
    let (first, second) = if a.end == End::Right || b.end == End::Left { (b, a) } else { (a, b) };
    let first_start = if first.end == End::Left { 0 } else { KERF };
    let second_end = if second.end == End::Right { STOCK } else { STOCK - KERF };
    let first_interval = (first_start, first_start + first.length);
    let second_interval = (second_end - second.length, second_end);
    let mut cuts = Vec::new();
    if first_start > 0 { cuts.push((0, KERF)); }
    cuts.push((first_interval.1, first_interval.1 + KERF));
    cuts.push((second_interval.0 - KERF, second_interval.0));
    if second_end < STOCK { cuts.push((STOCK - KERF, STOCK)); }
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
        if start > cursor { waste.push((cursor, start)); }
        cursor = cursor.max(end);
    }
    if cursor < STOCK { waste.push((cursor, STOCK)); }
    let parent = if a.length > b.length || (a.length == b.length
        && prepared.blocks[a.index].id < prepared.blocks[b.index].id) { a.index } else { b.index };
    let child = if parent == a.index { b.index } else { a.index };
    let parent_id = prepared.blocks[parent].id.clone();
    prepared.parents.insert(prepared.blocks[child].id.clone(), parent_id.clone());
    for candidate in [a, b] {
        prepared.zones.insert(prepared.blocks[candidate.index].id.clone(), candidate.zone_ids.clone());
    }
    let parts = [(first, first_interval), (second, second_interval)].into_iter().map(|(c, interval)| StockPart {
        block_id: prepared.blocks[c.index].id.clone(), interval_mm: mm(interval),
        stock_end: match c.end { End::Left => "left", End::Right => "right", End::Flat => "none" },
    }).collect();
    prepared.stock_plans.insert(parent_id, StockCutPlan { stock_code: "П640", stock_length_mm: 640.0,
        kerf_mm: 5.0, parts, saw_cuts_mm: cuts.into_iter().map(mm).collect(),
        waste_intervals_mm: waste.into_iter().map(mm).collect() });
    normalize(&mut prepared.blocks[a.index]);
    normalize(&mut prepared.blocks[b.index]);
}

type Pool = BTreeSet<(i64, String, usize)>;

fn match_pools(prepared: &mut PreparedParts, candidates: &[Candidate], first: &mut Pool,
    second: &mut Pool, flat_ends: i64)
{
    let capacity = STOCK - KERF * (1 + flat_ends);
    let ordered: Vec<_> = first.iter().rev().cloned().collect();
    for key in ordered {
        let maximum = capacity - key.0;
        let partner = second.range((Bound::Unbounded,
            Bound::Excluded((maximum + 1, String::new(), 0)))).next_back().cloned();
        if let Some(other) = partner {
            first.remove(&key);
            second.remove(&other);
            assign(prepared, &candidates[key.2], &candidates[other.2]);
        }
    }
}

pub fn prepare(request: &LayoutRequest, profile: &Profile, blocks: &[Block]) -> Result<PreparedParts, ApiFailure> {
    let blocks = crate::layout::finalize_parts(request, profile, blocks)?;
    let mut ids = BTreeSet::new();
    for block in &blocks {
        crate::code_result::validate_end_states(block)?;
        if !ids.insert(&block.id) { return Err(ApiFailure::new("INVALID_OFFCUT_PLAN",
            "Повтор идентификатора детали", Some(block.id.clone()))); }
    }
    let mut prepared = PreparedParts { blocks, parents: BTreeMap::new(), stock_plans: BTreeMap::new(), zones: BTreeMap::new() };
    // Каталог подтверждает единственную рядовую заводскую заготовку П640.
    // Сеточная половина 320 мм не становится самостоятельной заготовкой.
    if profile.ordinary_length_centimm != STOCK { return Ok(prepared); }
    let mut groups: BTreeMap<(i64, i64), Vec<Candidate>> = BTreeMap::new();
    for (index, block) in prepared.blocks.iter().enumerate() {
        if let Some((section, candidate)) = candidate(index, block, request, profile) {
            groups.entry(section).or_default().push(candidate);
        }
    }
    for candidates in groups.values() {
        let (mut left, mut right, mut flat) = (Pool::new(), Pool::new(), Pool::new());
        for (index, candidate) in candidates.iter().enumerate() {
            let key = (candidate.length, prepared.blocks[candidate.index].id.clone(), index);
            match candidate.end { End::Left => left.insert(key), End::Right => right.insert(key), End::Flat => flat.insert(key) };
        }
        match_pools(&mut prepared, candidates, &mut left, &mut right, 0);
        match_pools(&mut prepared, candidates, &mut left, &mut flat, 1);
        match_pools(&mut prepared, candidates, &mut right, &mut flat, 1);
        while let Some(key) = flat.pop_last() {
            let maximum = STOCK - 3 * KERF - key.0;
            let partner = flat.range((Bound::Unbounded,
                Bound::Excluded((maximum + 1, String::new(), 0)))).next_back().cloned();
            if let Some(other) = partner {
                flat.remove(&other);
                assign(&mut prepared, &candidates[key.2], &candidates[other.2]);
            }
        }
    }
    Ok(prepared)
}

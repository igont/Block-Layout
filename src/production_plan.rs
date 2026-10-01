//! Pure, all-or-nothing boundary between a geometric layout and production data.
//! A successful plan is a checked candidate for a future SUP adapter, not a SUP export.

use crate::domain::Point;
use crate::layout::{Block, Layout, Profile};
use crate::product_catalog::{ProductCatalog, ProductEntry, ProductFamily};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProfileApproval {
    pub profile_id: String,
    pub revision: String,
    pub catalog_version: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Section {
    pub width_centimm: i64,
    pub height_centimm: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProductBinding {
    pub product_type: String,
    pub selected_code: String,
}

/// External production decisions are explicit and keyed by stable layout ID.
/// They must be supplied by a separately approved upstream process.
#[derive(Clone, Debug, Default)]
pub struct ProductionOptions {
    pub approved_profile: Option<ProfileApproval>,
    pub section: Option<Section>,
    pub insu_by_block: BTreeMap<String, bool>,
    pub product_bindings: BTreeMap<String, ProductBinding>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProductionGeometry {
    pub origin: Point,
    pub z_centimm: i64,
    pub rotation_deg: u16,
    pub length_centimm: i64,
    pub width_centimm: i64,
    pub height_centimm: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProductionCut {
    pub product_key: String,
    pub x: u8,
    pub y: u8,
    pub source_run_ids: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProductionPart {
    pub stable_id: String,
    pub product_type: String,
    pub code1: String,
    pub code2: String,
    pub insu: bool,
    pub geometry: ProductionGeometry,
    pub is_bridge: bool,
    pub hide_spikes_left: bool,
    pub hide_spikes_right: bool,
    pub source_ids: Vec<String>,
    pub cuts: Vec<ProductionCut>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidatedProductionPlan {
    pub profile_id: String,
    pub profile_revision: String,
    pub catalog_version: String,
    pub parts: Vec<ProductionPart>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProductionErrorKind {
    UnapprovedProfile,
    InvalidSection,
    MissingSection,
    EmptyLayout,
    MissingStableId,
    DuplicateStableId,
    MissingProvenance,
    MissingInsuDecision,
    MissingProductBinding,
    UnresolvedProduct,
    UnsupportedCatalogStatus,
    UnsupportedBridgeGeometry,
    UnsupportedEndProcessing,
    UnsupportedCompoundCut,
    InvalidGeometry,
    CatalogLengthMismatch,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProductionError {
    pub block_id: Option<String>,
    pub kind: ProductionErrorKind,
    pub detail: String,
}

fn error(
    block: Option<&Block>,
    kind: ProductionErrorKind,
    detail: impl Into<String>,
) -> ProductionError {
    ProductionError {
        block_id: block.map(|item| item.id.clone()),
        kind,
        detail: detail.into(),
    }
}

fn node_name(key: &str) -> Option<&'static str> {
    Some(match key {
        "Type1" => "Тип 1",
        "Type2" => "Тип 2",
        "Type3" => "Тип 3",
        "Type4" => "Тип 4",
        "Type5" => "Тип 5",
        "Type5_1" => "Тип 5.1",
        "Type6" => "Тип 6",
        "Type7_1" => "Тип 7.1",
        "Type8" => "Тип 8",
        "Type8_1" => "Тип 8.1",
        "Type10_1" => "Тип 10.1",
        _ => return None,
    })
}

fn expected_node_cuts(kind: &str, key: &str) -> Option<&'static [(&'static str, u8, u8)]> {
    Some(match (kind, key) {
        ("node_L", "Type1") => &[("Type1", 1, 1)],
        ("node_L", "Type2") => &[("Type2", 1, 3)],
        ("node_L", "Type3") => &[("Type3", 1, 3)],
        ("node_L", "Type4") => &[("Type4", 1, 1)],
        ("node_T", "Type5") => &[("Type5_1", 2, 2)],
        ("node_T", "Type5_1") | ("node_X", "Type5_1") => &[("Type5_1", 1, 2)],
        ("node_T", "Type6") => &[("Type6", 2, 1)],
        ("node_T", "Type8") => &[("Type8", 1, 3)],
        ("node_T", "Type8_1") => &[("Type8_1", 1, 1)],
        ("node_T", "Type10_1") => &[("Type10_1", 1, 2)],
        ("node_X", "Type7_1") => &[("Type6", 2, 1), ("Type6", 2, 3)],
        _ => return None,
    })
}

fn catalog_entry<'a>(
    catalog: &'a ProductCatalog,
    name: &str,
    bridge: bool,
) -> Option<&'a ProductEntry> {
    if bridge {
        catalog.lintels().find(|entry| entry.name == name)
    } else {
        catalog.blocks().find(|entry| entry.name == name)
    }
}

fn checked_geometry(
    block: &Block,
    section: Section,
) -> Result<ProductionGeometry, ProductionError> {
    if block.length_centimm <= 0 || block.rotation_deg >= 360 {
        return Err(error(
            Some(block),
            ProductionErrorKind::InvalidGeometry,
            "Длина или поворот вне допустимого диапазона",
        ));
    }
    let is_node = block.kind.starts_with("node_");
    if is_node {
        if block.start != block.end
            || block.arms.is_empty()
            || block.local_origin.is_none()
            || block.local_rotation_deg.is_none()
        {
            return Err(error(
                Some(block),
                ProductionErrorKind::InvalidGeometry,
                "Узел не имеет физического тела и покрытых плеч",
            ));
        }
        let mut length_sum = 0_i64;
        let mut origin_on_arm = false;
        for arm in &block.arms {
            if arm.edge_id.is_empty() || arm.length_centimm <= 0 || arm.start == arm.end {
                return Err(error(
                    Some(block),
                    ProductionErrorKind::InvalidGeometry,
                    "Некорректное покрытое плечо",
                ));
            }
            origin_on_arm |= arm.start == block.start || arm.end == block.start;
            let dx = arm.end.x as f64 - arm.start.x as f64;
            let dy = arm.end.y as f64 - arm.start.y as f64;
            if !dx.hypot(dy).is_finite() || (dx.hypot(dy) - arm.length_centimm as f64).abs() > 1.0 {
                return Err(error(
                    Some(block),
                    ProductionErrorKind::InvalidGeometry,
                    "Ось плеча не соответствует его длине",
                ));
            }
            length_sum = length_sum.checked_add(arm.length_centimm).ok_or_else(|| {
                error(
                    Some(block),
                    ProductionErrorKind::InvalidGeometry,
                    "Переполнение длины плеч",
                )
            })?;
        }
        if length_sum != block.length_centimm || !origin_on_arm {
            return Err(error(
                Some(block),
                ProductionErrorKind::InvalidGeometry,
                "Плечи не составляют длину физической детали",
            ));
        }
    } else {
        if !block.arms.is_empty() || block.start == block.end {
            return Err(error(
                Some(block),
                ProductionErrorKind::InvalidGeometry,
                "Рядовой блок должен иметь одну ось и не иметь плеч узла",
            ));
        }
        let dx = block.end.x as f64 - block.start.x as f64;
        let dy = block.end.y as f64 - block.start.y as f64;
        let measured = dx.hypot(dy);
        if !measured.is_finite() || (measured - block.length_centimm as f64).abs() > 1.0 {
            return Err(error(
                Some(block),
                ProductionErrorKind::InvalidGeometry,
                "Геометрическая ось не соответствует длине",
            ));
        }
        let angle = f64::from(block.rotation_deg).to_radians();
        if (dx - measured * angle.cos()).abs() > 1.0 || (dy - measured * angle.sin()).abs() > 1.0 {
            return Err(error(
                Some(block),
                ProductionErrorKind::InvalidGeometry,
                "Поворот не соответствует оси",
            ));
        }
    }
    Ok(ProductionGeometry {
        origin: block.start,
        z_centimm: block.z_centimm,
        rotation_deg: block.rotation_deg,
        length_centimm: block.length_centimm,
        width_centimm: section.width_centimm,
        height_centimm: section.height_centimm,
    })
}

fn parse_node_cuts(block: &Block) -> Result<Vec<ProductionCut>, ProductionError> {
    let runs: BTreeSet<&str> = block.arms.iter().map(|arm| arm.edge_id.as_str()).collect();
    let mut result = Vec::new();
    for raw in &block.cuts {
        let fields: Vec<_> = raw.splitn(4, ':').collect();
        if fields.len() != 4 || node_name(fields[0]).is_none() {
            return Err(error(
                Some(block),
                ProductionErrorKind::UnsupportedCompoundCut,
                "Непроверенный формат запила узла",
            ));
        }
        let x = fields[1]
            .strip_prefix('x')
            .and_then(|value| value.parse::<u8>().ok());
        let y = fields[2]
            .strip_prefix('y')
            .and_then(|value| value.parse::<u8>().ok());
        let source_run_ids: Vec<_> = fields[3].split(',').map(str::to_owned).collect();
        if !matches!(x, Some(1..=3))
            || !matches!(y, Some(1..=3))
            || source_run_ids.is_empty()
            || source_run_ids.iter().any(|id| !runs.contains(id.as_str()))
        {
            return Err(error(
                Some(block),
                ProductionErrorKind::UnsupportedCompoundCut,
                "Запил не связан с покрытым плечом",
            ));
        }
        if fields[0] != block.product_key.as_deref().unwrap_or("")
            && !(block.product_key.as_deref() == Some("Type7_1") && fields[0] == "Type6")
            && !(block.kind == "node_T"
                && block.product_key.as_deref() == Some("Type5")
                && fields[0] == "Type5_1")
        {
            return Err(error(
                Some(block),
                ProductionErrorKind::UnsupportedCompoundCut,
                "Тип запила не соответствует изделию",
            ));
        }
        result.push(ProductionCut {
            product_key: fields[0].to_owned(),
            x: x.unwrap(),
            y: y.unwrap(),
            source_run_ids,
        });
    }
    if result.is_empty() {
        return Err(error(
            Some(block),
            ProductionErrorKind::UnsupportedCompoundCut,
            "Нет фиксированных запилов узла",
        ));
    }
    let expected = expected_node_cuts(&block.kind, block.product_key.as_deref().unwrap_or(""))
        .ok_or_else(|| {
            error(
                Some(block),
                ProductionErrorKind::UnsupportedCompoundCut,
                "Тип не относится к указанному узлу",
            )
        })?;
    let mut actual: Vec<_> = result
        .iter()
        .map(|cut| (cut.product_key.as_str(), cut.x, cut.y))
        .collect();
    actual.sort_unstable();
    let mut expected = expected.to_vec();
    expected.sort_unstable();
    if actual != expected {
        return Err(error(
            Some(block),
            ProductionErrorKind::UnsupportedCompoundCut,
            "Фиксированные запилы не совпадают с физической деталью каталога",
        ));
    }
    Ok(result)
}

fn validate_part(
    block: &Block,
    section: Section,
    options: &ProductionOptions,
    catalog: &ProductCatalog,
) -> Result<ProductionPart, ProductionError> {
    if block.id.is_empty() {
        return Err(error(
            Some(block),
            ProductionErrorKind::MissingStableId,
            "Пустой идентификатор",
        ));
    }
    if block.source_ids.is_empty() || block.source_ids.iter().any(|id| id.is_empty()) {
        return Err(error(
            Some(block),
            ProductionErrorKind::MissingProvenance,
            "Не указан исходный объект",
        ));
    }
    let insu = *options.insu_by_block.get(&block.id).ok_or_else(|| {
        error(
            Some(block),
            ProductionErrorKind::MissingInsuDecision,
            "Insu не задан явно",
        )
    })?;
    let geometry = checked_geometry(block, section)?;
    if block.hide_spikes_left || block.hide_spikes_right {
        return Err(error(
            Some(block),
            ProductionErrorKind::UnsupportedEndProcessing,
            "Нет проверенного правила спила шипов для кодовой пары",
        ));
    }
    if block.is_bridge || block.kind == "bridge" {
        let binding = options.product_bindings.get(&block.id).ok_or_else(|| {
            error(
                Some(block),
                ProductionErrorKind::MissingProductBinding,
                "Не выбран номер перемычки",
            )
        })?;
        if catalog
            .resolve(&binding.product_type, &binding.selected_code, true)
            .is_err()
        {
            return Err(error(
                Some(block),
                ProductionErrorKind::UnresolvedProduct,
                "Номер и код перемычки не найдены в каталоге",
            ));
        }
        return Err(error(
            Some(block),
            ProductionErrorKind::UnsupportedBridgeGeometry,
            "Не проверены профиль и обработка торцов перемычки",
        ));
    }
    let (product_type, cuts) = if block.kind.starts_with("node_") {
        if block.catalog_status != "KnownPattern" {
            return Err(error(
                Some(block),
                ProductionErrorKind::UnsupportedCatalogStatus,
                "Каталожная сборка узла не подтверждена",
            ));
        }
        let key = block
            .product_key
            .as_deref()
            .and_then(node_name)
            .ok_or_else(|| {
                error(
                    Some(block),
                    ProductionErrorKind::UnresolvedProduct,
                    "Неизвестный тип физической детали узла",
                )
            })?;
        (key.to_owned(), parse_node_cuts(block)?)
    } else if block.kind == "ordinary" {
        if block.catalog_status != "length_only" || !block.cuts.is_empty() {
            return Err(error(
                Some(block),
                ProductionErrorKind::UnsupportedEndProcessing,
                "Рядовая подрезка не имеет проверенной кодовой пары",
            ));
        }
        let binding = options.product_bindings.get(&block.id).ok_or_else(|| {
            error(
                Some(block),
                ProductionErrorKind::MissingProductBinding,
                "Не выбрано рядовое изделие",
            )
        })?;
        (binding.product_type.clone(), Vec::new())
    } else {
        return Err(error(
            Some(block),
            ProductionErrorKind::UnsupportedCatalogStatus,
            "Неизвестное семейство блока",
        ));
    };
    let entry = catalog_entry(catalog, &product_type, false).ok_or_else(|| {
        error(
            Some(block),
            ProductionErrorKind::UnresolvedProduct,
            "Изделие отсутствует в каталоге",
        )
    })?;
    if entry.family != ProductFamily::StandardBlock
        || entry.code1.is_empty()
        || entry.code2.is_empty()
    {
        return Err(error(
            Some(block),
            ProductionErrorKind::UnresolvedProduct,
            "Нет проверенной пары стандартных кодов",
        ));
    }
    if let Some(binding) = options.product_bindings.get(&block.id) {
        if binding.product_type != product_type
            || catalog
                .resolve(&product_type, &binding.selected_code, false)
                .is_err()
        {
            return Err(error(
                Some(block),
                ProductionErrorKind::UnresolvedProduct,
                "Привязка изделия противоречит каталогу",
            ));
        }
    }
    if entry.code_length_mm.map(|mm| i64::from(mm) * 100) != Some(block.length_centimm)
        || block
            .catalog_nominal_centimm
            .is_some_and(|mm| mm != block.length_centimm)
    {
        return Err(error(
            Some(block),
            ProductionErrorKind::CatalogLengthMismatch,
            "Длина детали и номинал изделия различаются",
        ));
    }
    Ok(ProductionPart {
        stable_id: block.id.clone(),
        product_type,
        code1: entry.code1.clone(),
        code2: entry.code2.clone(),
        insu,
        geometry,
        is_bridge: false,
        hide_spikes_left: false,
        hide_spikes_right: false,
        source_ids: block.source_ids.clone(),
        cuts,
    })
}

impl ValidatedProductionPlan {
    /// Without external approvals, an existing layout must not be called production-ready.
    pub fn try_from_layout(
        layout: &Layout,
        profile: &Profile,
        catalog: &ProductCatalog,
    ) -> Result<Self, Vec<ProductionError>> {
        Self::try_from_layout_with_options(layout, profile, catalog, &ProductionOptions::default())
    }

    pub fn try_from_layout_with_options(
        layout: &Layout,
        profile: &Profile,
        catalog: &ProductCatalog,
        options: &ProductionOptions,
    ) -> Result<Self, Vec<ProductionError>> {
        let mut errors = Vec::new();
        if layout.blocks.is_empty() {
            errors.push(error(
                None,
                ProductionErrorKind::EmptyLayout,
                "Раскладка не содержит деталей",
            ));
        }
        let approval = options.approved_profile.as_ref();
        if approval.is_none_or(|value| {
            value.profile_id != profile.profile_id
                || value.revision != profile.revision
                || value.catalog_version != profile.catalog_version
        }) {
            errors.push(error(
                None,
                ProductionErrorKind::UnapprovedProfile,
                "Точная редакция профиля и каталога не утверждена",
            ));
        }
        let section = match options.section {
            None => {
                errors.push(error(
                    None,
                    ProductionErrorKind::MissingSection,
                    "Сечение изделия не задано",
                ));
                None
            }
            Some(value) if value.width_centimm <= 0 || value.height_centimm <= 0 => {
                errors.push(error(
                    None,
                    ProductionErrorKind::InvalidSection,
                    "Некорректное сечение изделия",
                ));
                None
            }
            Some(value) => Some(value),
        };
        let mut seen = BTreeSet::new();
        let mut blocks: Vec<_> = layout.blocks.iter().collect();
        blocks.sort_by(|a, b| a.id.cmp(&b.id));
        let mut parts = Vec::with_capacity(blocks.len());
        for block in blocks {
            if !seen.insert(block.id.as_str()) {
                errors.push(error(
                    Some(block),
                    ProductionErrorKind::DuplicateStableId,
                    "Идентификатор детали повторён",
                ));
                continue;
            }
            if let Some(section) = section {
                match validate_part(block, section, options, catalog) {
                    Ok(part) => parts.push(part),
                    Err(issue) => errors.push(issue),
                }
            }
        }
        if !errors.is_empty() {
            return Err(errors);
        }
        Ok(Self {
            profile_id: profile.profile_id.clone(),
            profile_revision: profile.revision.clone(),
            catalog_version: profile.catalog_version.clone(),
            parts,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> Profile {
        Profile {
            profile_id: "approved-test".into(),
            revision: "1".into(),
            catalog_version: "snapshot-1".into(),
            node_assembly_family: None,
            world_joint_policy: None,
            lintel_support_mm: 160.0,
            longitudinal_full_wall_thickness: false,
            full_course_clearance: false,
            assembly_clearance_mm: 0.0,
            ordinary_length_centimm: 64_000,
            maximum_ordinary_blank_centimm: None,
            maximum_special_blank_centimm: None,
            ordinary_nominal_lengths_centimm: vec![32_000, 64_000],
            max_ordinary_cuts_per_free_span: 1,
            minimum_cut_centimm: 10_000,
            index_centimm: 6_300,
            coordinate_tolerance_centimm: 100,
            phase_count: 2,
            nodes: Vec::new(),
            bridge_nominal_lengths_centimm: vec![160_000],
        }
    }

    fn options(id: &str) -> ProductionOptions {
        let mut result = ProductionOptions {
            approved_profile: Some(ProfileApproval {
                profile_id: "approved-test".into(),
                revision: "1".into(),
                catalog_version: "snapshot-1".into(),
            }),
            section: Some(Section {
                width_centimm: 19_300,
                height_centimm: 6_300,
            }),
            ..Default::default()
        };
        result.insu_by_block.insert(id.into(), true);
        result
    }

    fn node() -> Block {
        let origin = Point { x: 0, y: 0 };
        Block {
            id: "node-1".into(),
            wall_id: "wall-1".into(),
            edge_id: "run-1".into(),
            course_index: 0,
            z_centimm: 0,
            start: origin,
            end: origin,
            length_centimm: 64_000,
            kind: "node_L".into(),
            product_key: Some("Type1".into()),
            catalog_status: "KnownPattern".into(),
            rotation_deg: 90,
            local_origin: Some(origin),
            local_rotation_deg: Some(90),
            is_bridge: false,
            hide_spikes_left: false,
            hide_spikes_right: false,
            natural_end_left: true,
            natural_end_right: true,
            cuts: vec!["Type1:x1:y1:run-1".into()],
            source_ids: vec!["wall:wall-1".into()],
            catalog_nominal_centimm: Some(64_000),
            components: Vec::new(),
            obstacle_ends: Vec::new(),
            arms: vec![crate::layout::BlockArm {
                wall_id: "wall-1".into(),
                edge_id: "run-1".into(),
                start: origin,
                end: Point { x: 0, y: 64_000 },
                length_centimm: 64_000,
            }],
        }
    }

    fn ordinary() -> Block {
        let mut block = node();
        block.id = "ordinary-1".into();
        block.kind = "ordinary".into();
        block.product_key = None;
        block.catalog_status = "length_only".into();
        block.start = Point { x: 0, y: 0 };
        block.end = Point { x: 64_000, y: 0 };
        block.rotation_deg = 0;
        block.local_origin = None;
        block.local_rotation_deg = None;
        block.catalog_nominal_centimm = None;
        block.arms.clear();
        block.cuts.clear();
        block
    }

    fn kinds(errors: &[ProductionError]) -> Vec<ProductionErrorKind> {
        errors.iter().map(|e| e.kind.clone()).collect()
    }

    #[test]
    fn exact_node_part_and_catalog_pair_can_form_a_plan() {
        let catalog = ProductCatalog::bundled().unwrap();
        let part = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![node()],
            },
            &profile(),
            &catalog,
            &options("node-1"),
        )
        .unwrap()
        .parts
        .remove(0);
        assert_eq!(part.product_type, "Тип 1");
        assert_eq!(part.code1, "П640 [Н-НЧ-Тип 1]");
        assert_eq!(part.code2, "П640 [В-КН-Тип 1]");
        assert_eq!(part.geometry.width_centimm, 19_300);
        assert_eq!(part.cuts[0].source_run_ids, vec!["run-1"]);
    }

    #[test]
    fn t0_short_type5_requires_its_exact_type5_1_node_cut() {
        let catalog = ProductCatalog::bundled().unwrap();
        let mut short = node();
        short.kind = "node_T".into();
        short.product_key = Some("Type5".into());
        short.length_centimm = 32_000;
        short.catalog_nominal_centimm = Some(32_000);
        short.start = Point { x: 0, y: 32_000 };
        short.end = short.start;
        short.rotation_deg = 270;
        short.local_origin = Some(Point { x: 0, y: 32_000 });
        short.local_rotation_deg = Some(270);
        short.arms[0].end = short.start;
        short.arms[0].length_centimm = 32_000;
        short.cuts = vec!["Type5_1:x2:y2:run-1".into()];

        let part = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![short.clone()],
            },
            &profile(),
            &catalog,
            &options("node-1"),
        )
        .unwrap()
        .parts
        .remove(0);
        assert_eq!(part.product_type, "Тип 5");
        assert_eq!(part.code1, "П320 [С-НЧ-Тип 5.1]");
        assert_eq!(part.code2, "П320 [С-КН-Тип 5.1]");
        assert_eq!(part.geometry.origin, short.start);
        assert_eq!(part.geometry.rotation_deg, 270);
        assert_eq!(part.cuts[0].product_key, "Type5_1");
        assert_eq!((part.cuts[0].x, part.cuts[0].y), (2, 2));

        short.cuts = vec!["Type5_1:x1:y2:run-1".into()];
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![short.clone()],
            },
            &profile(),
            &catalog,
            &options("node-1"),
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::UnsupportedCompoundCut));

        short.kind = "node_X".into();
        short.cuts = vec!["Type5_1:x2:y2:run-1".into()];
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![short],
            },
            &profile(),
            &catalog,
            &options("node-1"),
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::UnsupportedCompoundCut));
    }

    #[test]
    fn ordinary_requires_exact_explicit_binding_and_catalog_length() {
        let catalog = ProductCatalog::bundled().unwrap();
        let mut opts = options("ordinary-1");
        opts.product_bindings.insert(
            "ordinary-1".into(),
            ProductBinding {
                product_type: "Рядовой".into(),
                selected_code: "П640".into(),
            },
        );
        let part = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![ordinary()],
            },
            &profile(),
            &catalog,
            &opts,
        )
        .unwrap()
        .parts
        .remove(0);
        assert_eq!((part.code1.as_str(), part.code2.as_str()), ("П640", "П640"));
        let mut short = ordinary();
        short.length_centimm = 32_000;
        short.end.x = 32_000;
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![short],
            },
            &profile(),
            &catalog,
            &opts,
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::CatalogLengthMismatch));
    }

    #[test]
    fn three_argument_boundary_never_approves_study_layout_implicitly() {
        let catalog = ProductCatalog::bundled().unwrap();
        let errors = ValidatedProductionPlan::try_from_layout(
            &Layout {
                blocks: vec![node()],
            },
            &profile(),
            &catalog,
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::UnapprovedProfile));
        assert!(kinds(&errors).contains(&ProductionErrorKind::MissingSection));
    }

    #[test]
    fn y_and_unverified_node_are_rejected_without_partial_plan() {
        let catalog = ProductCatalog::bundled().unwrap();
        let mut bad = node();
        bad.id = "node-y".into();
        bad.kind = "node_Y".into();
        bad.product_key = Some("Type11".into());
        bad.catalog_status = "UnsupportedExportCatalog".into();
        let mut opts = options("node-1");
        opts.insu_by_block.insert("node-y".into(), true);
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![node(), bad],
            },
            &profile(),
            &catalog,
            &opts,
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::UnsupportedCatalogStatus));
    }

    #[test]
    fn missing_insu_and_unchecked_spike_processing_are_distinct_failures() {
        let catalog = ProductCatalog::bundled().unwrap();
        let mut opts = options("wrong-id");
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![node()],
            },
            &profile(),
            &catalog,
            &opts,
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::MissingInsuDecision));
        opts.insu_by_block.insert("node-1".into(), false);
        let mut altered = node();
        altered.hide_spikes_right = true;
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![altered],
            },
            &profile(),
            &catalog,
            &opts,
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::UnsupportedEndProcessing));
    }

    #[test]
    fn bridge_number_is_not_inferred_from_nominal_length() {
        let catalog = ProductCatalog::bundled().unwrap();
        let mut bridge = node();
        bridge.id = "bridge-1".into();
        bridge.kind = "bridge".into();
        bridge.is_bridge = true;
        bridge.start = Point { x: 0, y: 0 };
        bridge.end = Point { x: 64_000, y: 0 };
        bridge.arms.clear();
        bridge.local_origin = None;
        bridge.local_rotation_deg = None;
        bridge.rotation_deg = 0;
        let opts = options("bridge-1");
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![bridge],
            },
            &profile(),
            &catalog,
            &opts,
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::MissingProductBinding));
    }

    #[test]
    fn source_of_compound_cut_must_be_a_covered_arm() {
        let catalog = ProductCatalog::bundled().unwrap();
        let mut altered = node();
        altered.cuts[0] = "Type1:x1:y1:other-run".into();
        let errors = ValidatedProductionPlan::try_from_layout_with_options(
            &Layout {
                blocks: vec![altered],
            },
            &profile(),
            &catalog,
            &options("node-1"),
        )
        .unwrap_err();
        assert!(kinds(&errors).contains(&ProductionErrorKind::UnsupportedCompoundCut));
    }
}

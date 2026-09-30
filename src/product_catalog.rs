//! Снимок каталога изделий SUP. Коды и номиналы здесь являются данными,
//! а не описанием формы изделия или правил раскладки.

use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProductFamily {
    StandardBlock,
    CombinedBlock,
    SingleBlock,
    Lintel,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProductEntry {
    pub name: String,
    pub display_name: String,
    pub code1: String,
    pub code2: String,
    pub family: ProductFamily,
    pub code_length_mm: Option<u32>,
    pub production_blank_mm: Option<u32>,
    pub display_length_mm: Option<u32>,
    /// Исходная десятичная запись сохраняется без округления.
    pub volume_m3: Option<String>,
    pub volume_source: Option<String>,
    pub note: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CatalogError {
    pub table: &'static str,
    pub line: usize,
    pub message: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum UnresolvedProduct {
    MissingType,
    MissingCode {
        product_type: String,
    },
    UnknownType {
        product_type: String,
        is_bridge: bool,
    },
    UnknownCode {
        product_type: String,
        code: String,
    },
    BridgeMismatch {
        product_type: String,
        is_bridge: bool,
    },
}

#[derive(Clone, Debug, Default)]
pub struct ProductCatalog {
    blocks: BTreeMap<String, ProductEntry>,
    lintels: BTreeMap<String, ProductEntry>,
}

impl ProductCatalog {
    pub fn parse(block_tsv: &str, lintel_tsv: &str) -> Result<Self, CatalogError> {
        Ok(Self {
            blocks: parse_blocks(block_tsv)?,
            lintels: parse_lintels(lintel_tsv)?,
        })
    }

    pub fn bundled() -> Result<Self, CatalogError> {
        Self::parse(
            include_str!("../profiles/sup-block-catalog.tsv"),
            include_str!("../profiles/sup-lintel-catalog.tsv"),
        )
    }

    pub fn blocks(&self) -> impl Iterator<Item = &ProductEntry> {
        self.blocks.values()
    }

    pub fn lintels(&self) -> impl Iterator<Item = &ProductEntry> {
        self.lintels.values()
    }

    /// Ищет точную пару Type + Code в соответствующем каталоге.
    /// Булево значение is_bridge выбирает каталог перемычек.
    pub fn resolve(
        &self,
        product_type: &str,
        code: &str,
        is_bridge: bool,
    ) -> Result<&ProductEntry, UnresolvedProduct> {
        if product_type.trim().is_empty() {
            return Err(UnresolvedProduct::MissingType);
        }
        if code.trim().is_empty() {
            return Err(UnresolvedProduct::MissingCode {
                product_type: product_type.to_owned(),
            });
        }
        let (selected, other) = if is_bridge {
            (&self.lintels, &self.blocks)
        } else {
            (&self.blocks, &self.lintels)
        };
        let Some(entry) = selected.get(product_type) else {
            return Err(if other.contains_key(product_type) {
                UnresolvedProduct::BridgeMismatch {
                    product_type: product_type.to_owned(),
                    is_bridge,
                }
            } else {
                UnresolvedProduct::UnknownType {
                    product_type: product_type.to_owned(),
                    is_bridge,
                }
            });
        };
        if code == entry.code1 || code == entry.code2 {
            Ok(entry)
        } else {
            Err(UnresolvedProduct::UnknownCode {
                product_type: product_type.to_owned(),
                code: code.to_owned(),
            })
        }
    }
}

const BLOCK_HEADER: [&str; 11] = [
    "Name",
    "DisplayName",
    "Code1",
    "Code2",
    "Family",
    "CodeLengthMm",
    "ProductionBlankMm",
    "DisplayLengthMm",
    "VolumeM3",
    "VolumeSource",
    "Note",
];
const LINTEL_HEADER: [&str; 8] = [
    "Number",
    "DisplayName",
    "InsulatedCode",
    "PlainCode",
    "ProductionBlankMm",
    "DisplayLengthMm",
    "VolumeM3",
    "Note",
];
const LINTEL_REVISION_HEADER: [&str; 4] = ["Revision", "CreatedAt", "UpdatedAt", "UpdatedBy"];

fn rows<'a>(
    table: &'static str,
    tsv: &'a str,
    header: &[&str],
    extension: &[&str],
) -> Result<Vec<(usize, Vec<&'a str>)>, CatalogError> {
    let mut lines = tsv.strip_prefix('\u{feff}').unwrap_or(tsv).lines();
    let first = lines.next().ok_or_else(|| error(table, 1, "Пустой TSV"))?;
    let actual: Vec<_> = first.trim_end_matches('\r').split('\t').collect();
    let valid = actual == header
        || (!extension.is_empty()
            && actual.len() == header.len() + extension.len()
            && actual[..header.len()] == *header
            && actual[header.len()..] == *extension);
    if !valid {
        return Err(error(table, 1, "Неожиданный заголовок TSV"));
    }
    let mut result = Vec::new();
    for (index, line) in lines.enumerate() {
        let line_number = index + 2;
        let cells: Vec<_> = line.trim_end_matches('\r').split('\t').collect();
        if cells.len() != actual.len() || cells.iter().all(|cell| cell.is_empty()) {
            return Err(error(
                table,
                line_number,
                "Некорректное число колонок или пустая строка",
            ));
        }
        result.push((line_number, cells));
    }
    if result.is_empty() {
        return Err(error(table, 2, "Каталог не содержит изделий"));
    }
    Ok(result)
}

fn parse_blocks(tsv: &str) -> Result<BTreeMap<String, ProductEntry>, CatalogError> {
    let mut entries = BTreeMap::new();
    for (line, cells) in rows("blocks", tsv, &BLOCK_HEADER, &[])? {
        require_text("blocks", line, &cells, &[0, 1, 2, 3, 4, 9])?;
        let family = match cells[4] {
            "STANDARD_BLOCK" => ProductFamily::StandardBlock,
            "COMBINED_BLOCK" => ProductFamily::CombinedBlock,
            "SINGLE_BLOCK" => ProductFamily::SingleBlock,
            _ => return Err(error("blocks", line, "Неизвестное семейство изделия")),
        };
        let volume_m3 = decimal("blocks", line, cells[8])?
            .ok_or_else(|| error("blocks", line, "Пустой VolumeM3"))?;
        let entry = ProductEntry {
            name: cells[0].to_owned(),
            display_name: cells[1].to_owned(),
            code1: cells[2].to_owned(),
            code2: cells[3].to_owned(),
            family,
            code_length_mm: positive_mm("blocks", line, cells[5], false)?,
            production_blank_mm: positive_mm("blocks", line, cells[6], false)?,
            display_length_mm: positive_mm("blocks", line, cells[7], false)?,
            volume_m3: Some(volume_m3),
            volume_source: Some(cells[9].to_owned()),
            note: cells[10].to_owned(),
        };
        if entries.insert(entry.name.clone(), entry).is_some() {
            return Err(error("blocks", line, "Повтор Name"));
        }
    }
    Ok(entries)
}

fn parse_lintels(tsv: &str) -> Result<BTreeMap<String, ProductEntry>, CatalogError> {
    let mut entries = BTreeMap::new();
    for (line, cells) in rows("lintels", tsv, &LINTEL_HEADER, &LINTEL_REVISION_HEADER)? {
        require_text("lintels", line, &cells, &[0, 1, 2, 3])?;
        let number = cells[0]
            .parse::<u32>()
            .map_err(|_| error("lintels", line, "Некорректный Number"))?;
        if number == 0 || cells[1] != format!("Перемычка №{number}") {
            return Err(error(
                "lintels",
                line,
                "Number и DisplayName не согласованы",
            ));
        }
        let entry = ProductEntry {
            name: cells[1].to_owned(),
            display_name: cells[1].to_owned(),
            code1: cells[2].to_owned(),
            code2: cells[3].to_owned(),
            family: ProductFamily::Lintel,
            code_length_mm: None,
            production_blank_mm: positive_mm("lintels", line, cells[4], true)?,
            display_length_mm: positive_mm("lintels", line, cells[5], true)?,
            volume_m3: decimal("lintels", line, cells[6])?,
            volume_source: None,
            note: cells[7].to_owned(),
        };
        if entries.insert(entry.name.clone(), entry).is_some() {
            return Err(error("lintels", line, "Повтор Number"));
        }
    }
    Ok(entries)
}

fn require_text(
    table: &'static str,
    line: usize,
    cells: &[&str],
    indices: &[usize],
) -> Result<(), CatalogError> {
    if indices
        .iter()
        .any(|&i| cells[i].is_empty() || cells[i].trim() != cells[i])
    {
        Err(error(table, line, "Пустое поле или лишние пробелы"))
    } else {
        Ok(())
    }
}

fn positive_mm(
    table: &'static str,
    line: usize,
    value: &str,
    optional: bool,
) -> Result<Option<u32>, CatalogError> {
    if optional && value.is_empty() {
        return Ok(None);
    }
    match value.parse::<u32>() {
        Ok(n) if n > 0 => Ok(Some(n)),
        _ => Err(error(table, line, "Некорректная положительная длина")),
    }
}

fn decimal(table: &'static str, line: usize, value: &str) -> Result<Option<String>, CatalogError> {
    if value.is_empty() {
        return Ok(None);
    }
    let valid = value.bytes().all(|b| b.is_ascii_digit() || b == b'.')
        && value.matches('.').count() <= 1
        && value.parse::<f64>().is_ok_and(|n| n.is_finite() && n > 0.0);
    if valid {
        Ok(Some(value.to_owned()))
    } else {
        Err(error(table, line, "Некорректный десятичный VolumeM3"))
    }
}

fn error(table: &'static str, line: usize, message: &str) -> CatalogError {
    CatalogError {
        table,
        line,
        message: message.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_catalog_resolves_exact_codes_and_keeps_unknown_nodes_unresolved() {
        let catalog = ProductCatalog::bundled().unwrap();
        assert_eq!(catalog.blocks().count(), 25);
        assert_eq!(catalog.lintels().count(), 22);
        let ordinary = catalog.resolve("Рядовой", "П640", false).unwrap();
        assert_eq!(ordinary.code_length_mm, Some(640));
        assert_eq!(ordinary.production_blank_mm, Some(650));
        assert_eq!(
            catalog
                .resolve("Тип 1", "П640 [В-КН-Тип 1]", false)
                .unwrap()
                .family,
            ProductFamily::StandardBlock
        );
        let bridge = catalog
            .resolve(
                "Перемычка №201",
                "П1600 [С-НЧ-Тип 10.1] [Н-КН-Тип 8.1]",
                true,
            )
            .unwrap();
        assert_eq!(bridge.production_blank_mm, None);
        assert_eq!(
            catalog
                .resolve("Перемычка №222", "П1920 [Н-0-Тип 8]", true)
                .unwrap()
                .family,
            ProductFamily::Lintel
        );
        for node in ["L", "T", "X"] {
            assert!(matches!(
                catalog.resolve(node, "П640", false),
                Err(UnresolvedProduct::UnknownType { .. })
            ));
        }
    }

    #[test]
    fn lookup_does_not_guess_code_or_bridge_identity() {
        let catalog = ProductCatalog::bundled().unwrap();
        assert!(matches!(
            catalog.resolve("Рядовой", "", false),
            Err(UnresolvedProduct::MissingCode { .. })
        ));
        assert!(matches!(
            catalog.resolve("Рядовой", "П320", false),
            Err(UnresolvedProduct::UnknownCode { .. })
        ));
        assert!(matches!(
            catalog.resolve("Рядовой", "П640", true),
            Err(UnresolvedProduct::BridgeMismatch { .. })
        ));
        assert!(matches!(
            catalog.resolve("", "П640", false),
            Err(UnresolvedProduct::MissingType)
        ));
    }

    #[test]
    fn parser_rejects_bad_header_duplicate_name_and_numeric_data() {
        let blocks = include_str!("../profiles/sup-block-catalog.tsv");
        let lintels = include_str!("../profiles/sup-lintel-catalog.tsv");
        assert_eq!(
            ProductCatalog::parse("wrong\n", lintels).unwrap_err().line,
            1
        );
        let first = blocks.lines().nth(1).unwrap();
        let duplicate = format!("{blocks}{first}\n");
        assert_eq!(
            ProductCatalog::parse(&duplicate, lintels)
                .unwrap_err()
                .message,
            "Повтор Name"
        );
        let bad_length = blocks.replacen("\t640\t650\t640\t", "\tbad\t650\t640\t", 1);
        assert_eq!(
            ProductCatalog::parse(&bad_length, lintels)
                .unwrap_err()
                .line,
            2
        );
    }
}

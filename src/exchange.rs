//! Нейтральная граница fb-layout/1. Рабочая геометрия нормализуется до 0,01 мм.
use crate::api::{ApiFailure, Beam, BeamGeometry, LayoutRequest, Volume};
use crate::layout::Profile;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExchangeRequest {
    pub format: String,
    pub kind: String,
    pub request_id: String,
    pub snapshot_hash: String,
    pub coordinate_system: CoordinateSystem,
    pub profile: VersionedRef,
    pub catalog: VersionedRef,
    pub scope: Scope,
    pub model: Model,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CoordinateSystem {
    pub id: String,
    pub units: String,
    pub coordinate_quantum_mm: f64,
    pub z0_mm: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VersionedRef {
    pub id: String,
    pub revision: String,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scope {
    pub mode: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wall_ids: Option<Vec<String>>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Model {
    pub wall_volumes: Vec<WallVolume>,
    pub openings: Vec<Opening>,
    pub beams: Vec<ExchangeBeam>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub saved_blocks: Option<Vec<crate::lamella::SavedBlock>>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AxisPrism {
    pub start_xy_mm: [f64; 2],
    pub end_xy_mm: [f64; 2],
    pub bottom_start_mm: f64,
    pub bottom_end_mm: f64,
    pub top_start_mm: f64,
    pub top_end_mm: f64,
    pub left_thickness_mm: f64,
    pub right_thickness_mm: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WallVolume {
    pub id: String,
    pub purpose: String,
    pub volume: AxisPrism,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Opening {
    pub id: String,
    pub purpose: String,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub is_outside: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wall_ids: Option<Vec<String>>,
    pub volume: AxisPrism,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExchangeBeam {
    pub id: String,
    pub start_mm: [f64; 3],
    pub end_mm: [f64; 3],
    pub width_mm: f64,
    pub height_mm: f64,
    pub height_direction: [f64; 3],
}

fn error(code: &str, message: &str, id: Option<&str>) -> ApiFailure {
    ApiFailure::new(code, message, id.map(str::to_owned))
}
fn valid_id(id: &str) -> bool {
    !id.trim().is_empty()
}

pub fn parse_request(text: &str) -> Result<ExchangeRequest, ApiFailure> {
    let request: ExchangeRequest =
        serde_json::from_str(text).map_err(|e| error("INVALID_EXCHANGE", &e.to_string(), None))?;
    request.to_layout_request()?;
    Ok(request)
}

impl AxisPrism {
    fn convert(&self, id: &str, purpose: i32, opening: &str) -> Result<Volume, ApiFailure> {
        let mut normalized = self.clone();
        normalized.start_xy_mm = normalized.start_xy_mm.map(crate::precision::mm);
        normalized.end_xy_mm = normalized.end_xy_mm.map(crate::precision::mm);
        for value in [
            &mut normalized.bottom_start_mm, &mut normalized.bottom_end_mm,
            &mut normalized.top_start_mm, &mut normalized.top_end_mm,
            &mut normalized.left_thickness_mm, &mut normalized.right_thickness_mm,
        ] {
            *value = crate::precision::mm(*value);
        }
        normalized.convert_normalized(id, purpose, opening)
    }

    fn convert_normalized(&self, id: &str, purpose: i32, opening: &str) -> Result<Volume, ApiFailure> {
        let values = [
            self.start_xy_mm[0],
            self.start_xy_mm[1],
            self.end_xy_mm[0],
            self.end_xy_mm[1],
            self.bottom_start_mm,
            self.bottom_end_mm,
            self.top_start_mm,
            self.top_end_mm,
            self.left_thickness_mm,
            self.right_thickness_mm,
        ];
        if values.iter().any(|v| !v.is_finite())
            || self.start_xy_mm == self.end_xy_mm
            || self.top_start_mm <= self.bottom_start_mm
            || self.top_end_mm <= self.bottom_end_mm
            || self.left_thickness_mm < 0.0
            || self.right_thickness_mm < 0.0
            || self.left_thickness_mm + self.right_thickness_mm <= 0.0
        {
            return Err(error("INVALID_GEOMETRY", "Некорректная призма", Some(id)));
        }
        if self.left_thickness_mm != self.right_thickness_mm {
            return Err(error(
                "UNSUPPORTED_GEOMETRY",
                "Асимметричная толщина пока не поддержана ядром",
                Some(id),
            ));
        }
        Ok(Volume {
            guid: id.into(),
            start_xmm: self.start_xy_mm[0],
            start_ymm: self.start_xy_mm[1],
            end_xmm: self.end_xy_mm[0],
            end_ymm: self.end_xy_mm[1],
            start_bottom_zmm: self.bottom_start_mm,
            end_bottom_zmm: self.bottom_end_mm,
            start_top_zmm: self.top_start_mm,
            end_top_zmm: self.top_end_mm,
            thickness_mm: crate::precision::mm(self.left_thickness_mm + self.right_thickness_mm),
            purpose_type: purpose,
            opening_type: opening.into(),
            is_outside: false,
        })
    }
}

impl ExchangeRequest {
    pub fn validate_profile(&self, profile: &Profile) -> Result<(), ApiFailure> {
        if self.profile.id != profile.profile_id || self.profile.revision != profile.revision {
            return Err(error(
                "PROFILE_MISMATCH",
                "Запрошенная редакция профиля не совпадает с загруженной",
                None,
            ));
        }
        if self.catalog.id != profile.catalog_version || self.catalog.revision != profile.revision {
            return Err(error(
                "CATALOG_MISMATCH",
                "Запрошенная редакция каталога не совпадает с профилем прототипа",
                None,
            ));
        }
        Ok(())
    }

    pub fn to_layout_request(&self) -> Result<LayoutRequest, ApiFailure> {
        if self.format != "fb-layout/1" || !matches!(self.kind.as_str(),"layout_request"|"blocks_layout_request"|"lamella_layout_request") {
            return Err(error(
                "UNSUPPORTED_VERSION",
                "Ожидается запрос fb-layout/1",
                None,
            ));
        }
        if self.kind!="lamella_layout_request"&&self.model.saved_blocks.is_some() {
            return Err(error("INVALID_EXCHANGE","saved_blocks допустимы только при отдельной раскладке ламелей",None));
        }
        if self.kind=="lamella_layout_request"&&self.model.saved_blocks.is_none() {
            return Err(error("INVALID_EXCHANGE","Для отдельной раскладки ламелей требуется массив saved_blocks",None));
        }
        if !valid_id(&self.request_id)
            || self.snapshot_hash.len() != 64
            || !self
                .snapshot_hash
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
            || !valid_id(&self.coordinate_system.id)
            || !self.coordinate_system.z0_mm.is_finite()
            || !valid_id(&self.profile.id)
            || !valid_id(&self.profile.revision)
            || !valid_id(&self.catalog.id)
            || !valid_id(&self.catalog.revision)
        {
            return Err(error(
                "INVALID_EXCHANGE",
                "Некорректная идентичность запроса или системы координат",
                None,
            ));
        }
        if self.coordinate_system.units != "mm"
            || self.coordinate_system.coordinate_quantum_mm != 0.01
        {
            return Err(error(
                "UNSUPPORTED_COORDINATES",
                "Единицы должны быть mm, квант 0.01 mm",
                None,
            ));
        }
        if self.scope.mode != "full" {
            return Err(error(
                "UNSUPPORTED_SCOPE",
                "Поддерживается только полная раскладка здания",
                None,
            ));
        }
        if self.scope.wall_ids.is_some() {
            return Err(error("INVALID_SCOPE", "full не содержит wall_ids", None));
        }
        let mut ids = BTreeSet::new();
        for id in self
            .model
            .wall_volumes
            .iter()
            .map(|w| w.id.as_str())
            .chain(self.model.openings.iter().map(|o| o.id.as_str()))
            .chain(self.model.beams.iter().map(|b| b.id.as_str()))
        {
            if !valid_id(id) || !ids.insert(id) {
                return Err(error(
                    "DUPLICATE_SOURCE_ID",
                    "Пустой или повторный ID источника",
                    Some(id),
                ));
            }
        }
        let mut walls = Vec::new();
        for wall in &self.model.wall_volumes {
            let purpose = match wall.purpose.as_str() {
                "fb_wall" | "fb_console" => 1,
                "context" => {
                    return Err(error(
                        "UNSUPPORTED_WALL_PURPOSE",
                        "Геометрия контекстных стен пока не участвует в ограничениях ядра",
                        Some(&wall.id),
                    ))
                }
                _ => {
                    return Err(error(
                        "INVALID_PURPOSE",
                        "Неизвестное назначение стены",
                        Some(&wall.id),
                    ))
                }
            };
            walls.push(wall.volume.convert(&wall.id, purpose,
                if wall.purpose == "fb_console" { "CONSOLE" } else { "" })?);
        }
        let mut openings = Vec::new();
        for opening in &self.model.openings {
            if opening.wall_ids.is_some() {
                return Err(error(
                    "UNSUPPORTED_OPENING_BINDING",
                    "Явная привязка проёма к стенам пока не поддержана",
                    Some(&opening.id),
                ));
            }
            let purpose = match opening.purpose.as_str() {
                "opening" => "OPENING",
                "window" => "WINDOW",
                "door" => "DOOR",
                "partition_opening" => "PARTITION_OPENING",
                "console" => "CONSOLE",
                "wall_trim" => {
                    return Err(error(
                        "UNSUPPORTED_OPENING_PURPOSE",
                        "wall_trim пока не поддержан",
                        Some(&opening.id),
                    ))
                }
                _ => {
                    return Err(error(
                        "INVALID_PURPOSE",
                        "Неизвестное назначение проёма",
                        Some(&opening.id),
                    ))
                }
            };
            let mut volume = opening.volume.convert(&opening.id, 2, purpose)?;
            volume.is_outside = opening.is_outside;
            openings.push(volume);
        }
        let mut beams = Vec::new();
        for beam in &self.model.beams {
            let axis: [f64; 3] = std::array::from_fn(|i| beam.end_mm[i] - beam.start_mm[i]);
            let length = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
            let norm = beam.height_direction.iter().map(|v| v * v).sum::<f64>();
            let dot = axis
                .iter()
                .zip(beam.height_direction)
                .map(|(a, b)| a * b)
                .sum::<f64>();
            if beam
                .start_mm
                .iter()
                .chain(&beam.end_mm)
                .chain(&beam.height_direction)
                .any(|v| !v.is_finite())
                || !beam.width_mm.is_finite()
                || !beam.height_mm.is_finite()
                || beam.width_mm <= 0.0
                || beam.height_mm <= 0.0
                || !length.is_finite()
                || length <= 0.0
                || (norm - 1.0).abs() > 1e-6
                || dot.abs() / length > 1e-6
            {
                return Err(error(
                    "INVALID_BEAM_FRAME",
                    "Некорректная ось или направление высоты балки",
                    Some(&beam.id),
                ));
            }
            let mut beam = beam.clone();
            beam.start_mm = beam.start_mm.map(crate::precision::mm);
            beam.end_mm = beam.end_mm.map(crate::precision::mm);
            beam.width_mm = crate::precision::mm(beam.width_mm);
            beam.height_mm = crate::precision::mm(beam.height_mm);
            if beam.start_mm == beam.end_mm || beam.width_mm <= 0.0 || beam.height_mm <= 0.0 {
                return Err(error("INVALID_BEAM_FRAME", "Балка вырождается на сетке 0,01 мм", Some(&beam.id)));
            }
            beams.push(Beam {
                guid: beam.id.clone(),
                start_xmm: beam.start_mm[0],
                start_ymm: beam.start_mm[1],
                start_zmm: beam.start_mm[2],
                end_xmm: beam.end_mm[0],
                end_ymm: beam.end_mm[1],
                end_zmm: beam.end_mm[2],
                geometry: BeamGeometry {
                    width_mm: beam.width_mm,
                    height_mm: beam.height_mm,
                    height_direction_x: beam.height_direction[0],
                    height_direction_y: beam.height_direction[1],
                    height_direction_z: beam.height_direction[2],
                },
            });
        }
        let request = LayoutRequest {
            schema_version: 1,
            request_id: self.request_id.clone(),
            project_name: String::new(),
            project_id: None,
            snapshot_hash: self.snapshot_hash.clone(),
            z0_mm: crate::precision::mm(self.coordinate_system.z0_mm),
            wall_volumes: walls,
            opening_volumes: openings,
            beams,
            metadata: Value::Null,
        };
        request.validate()?;
        Ok(request)
    }
}

fn prism(volume: &Volume) -> AxisPrism {
    AxisPrism {
        start_xy_mm: [volume.start_xmm, volume.start_ymm],
        end_xy_mm: [volume.end_xmm, volume.end_ymm],
        bottom_start_mm: volume.start_bottom_zmm,
        bottom_end_mm: volume.end_bottom_zmm,
        top_start_mm: volume.start_top_zmm,
        top_end_mm: volume.end_top_zmm,
        left_thickness_mm: volume.thickness_mm / 2.0,
        right_thickness_mm: volume.thickness_mm / 2.0,
    }
}

/// Адаптер исследовательского снимка. Метаданные приложения не входят в нейтральную модель.
pub fn from_layout_request(
    request: &LayoutRequest,
    profile: &Profile,
) -> Result<ExchangeRequest, ApiFailure> {
    request.validate()?;
    let walls = request
        .wall_volumes
        .iter()
        .map(|w| {
            Ok(WallVolume {
                id: w.guid.clone(),
                purpose: match w.purpose_type {
                    1 if w.opening_type == "CONSOLE" => "fb_console",
                    1 => "fb_wall",
                    0 => "context",
                    _ => {
                        return Err(error(
                            "UNSUPPORTED_WALL_PURPOSE",
                            "Назначение стены нельзя перенести без потерь",
                            Some(&w.guid),
                        ))
                    }
                }
                .into(),
                volume: prism(w),
            })
        })
        .collect::<Result<Vec<_>, ApiFailure>>()?;
    let openings = request
        .opening_volumes
        .iter()
        .map(|o| {
            Ok(Opening {
                id: o.guid.clone(),
                purpose: match o.opening_type.as_str() {
                    "OPENING" => "opening",
                    "WINDOW" => "window",
                    "DOOR" => "door",
                    "PARTITION_OPENING" => "partition_opening",
                    "CONSOLE" => "console",
                    _ => {
                        return Err(error(
                            "UNSUPPORTED_OPENING_PURPOSE",
                            "Назначение проёма нельзя перенести без потерь",
                            Some(&o.guid),
                        ))
                    }
                }
                .into(),
                wall_ids: None,
                is_outside: o.is_outside,
                volume: prism(o),
            })
        })
        .collect::<Result<Vec<_>, ApiFailure>>()?;
    let converted = ExchangeRequest {
        format: "fb-layout/1".into(),
        kind: "layout_request".into(),
        request_id: if request.request_id.is_empty() {
            format!("snapshot-{}", request.snapshot_hash)
        } else {
            request.request_id.clone()
        },
        snapshot_hash: request.snapshot_hash.clone(),
        coordinate_system: CoordinateSystem {
            id: "snapshot-world".into(),
            units: "mm".into(),
            coordinate_quantum_mm: 0.01,
            z0_mm: request.z0_mm,
        },
        profile: VersionedRef {
            id: profile.profile_id.clone(),
            revision: profile.revision.clone(),
        },
        catalog: VersionedRef {
            id: profile.catalog_version.clone(),
            revision: profile.revision.clone(),
        },
        scope: Scope {
            mode: "full".into(),
            wall_ids: None,
        },
        model: Model {
            wall_volumes: walls,
            openings,
            beams: request
                .beams
                .iter()
                .map(|b| ExchangeBeam {
                    id: b.guid.clone(),
                    start_mm: [b.start_xmm, b.start_ymm, b.start_zmm],
                    end_mm: [b.end_xmm, b.end_ymm, b.end_zmm],
                    width_mm: b.geometry.width_mm,
                    height_mm: b.geometry.height_mm,
                    height_direction: [
                        b.geometry.height_direction_x,
                        b.geometry.height_direction_y,
                        b.geometry.height_direction_z,
                    ],
                })
                .collect(),
            saved_blocks: None,
        },
    };
    converted.to_layout_request()?;
    Ok(converted)
}

fn result_envelope(request: &ExchangeRequest) -> Value {
    let mut result = json!({"format":"fb-layout/1", "kind":"layout_result", "request_id":request.request_id,
        "snapshot_hash":request.snapshot_hash, "coordinate_system":request.coordinate_system,
        "profile":request.profile, "catalog":request.catalog, "scope":request.scope});
    crate::precision::normalize_result(&mut result);
    result
}
pub fn success_result(
    request: &ExchangeRequest,
    blocks: Vec<Value>,
    beam_adjustments: Vec<Value>,
) -> Value {
    let mut result = result_envelope(request);
    result["status"] = json!("success");
    result["blocks"] = json!(blocks);
    result["beam_adjustments"] = json!(beam_adjustments);
    crate::precision::normalize_result(&mut result);
    result
}
pub fn success_result_with_warnings(
    request: &ExchangeRequest,
    blocks: Vec<Value>,
    beam_adjustments: Vec<Value>,
    warnings: &[ApiFailure],
) -> Value {
    let mut result = success_result(request, blocks, beam_adjustments);
    if !warnings.is_empty() {
        result["warnings"] = json!(warnings.iter().map(diagnostic_value).collect::<Vec<_>>());
    }
    result
}

pub fn diagnostic_value(diagnostic: &ApiFailure) -> Value {
    let mut sources: BTreeSet<_> = diagnostic.source_ids.iter().cloned().collect();
    if let Some(id) = &diagnostic.source_id {
        sources.insert(id.clone());
    }
    let mut value =
        json!({"code":diagnostic.code,"message":diagnostic.message,"source_ids":sources});
    if let Some(course) = diagnostic.course_index {
        value["course_index"] = json!(course);
    }
    value
}

pub fn failure_result(request: &ExchangeRequest, diagnostics: &[ApiFailure]) -> Value {
    let mut result = result_envelope(request);
    result["status"] = json!("failure");
    let values: Vec<_> = diagnostics.iter().map(diagnostic_value).collect();
    result["diagnostics"] = if values.is_empty() {
        json!([{"code":"EMPTY_DIAGNOSTICS","message":"Причина отказа не передана","source_ids":[]}])
    } else {
        json!(values)
    };
    result
}

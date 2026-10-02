use serde::{Deserialize, Serialize};

use crate::domain::{RawBuilding, RawPoint, RawWall};

/// Versioned input boundary for a complete SUP model snapshot.
#[derive(Debug, Clone, Deserialize)]
pub struct LayoutRequest {
    pub schema_version: u32,
    #[serde(default)]
    pub request_id: String,
    #[serde(default)]
    pub project_name: String,
    #[serde(default)]
    pub project_id: Option<String>,
    #[serde(default)]
    pub snapshot_hash: String,
    pub z0_mm: f64,
    pub wall_volumes: Vec<Volume>,
    #[serde(default)]
    pub opening_volumes: Vec<Volume>,
    #[serde(default)]
    pub beams: Vec<Beam>,
    #[serde(default)]
    pub metadata: serde_json::Value,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Volume {
    pub guid: String,
    pub start_xmm: f64,
    pub start_ymm: f64,
    pub end_xmm: f64,
    pub end_ymm: f64,
    pub start_bottom_zmm: f64,
    pub end_bottom_zmm: f64,
    pub start_top_zmm: f64,
    pub end_top_zmm: f64,
    #[serde(default)]
    pub thickness_mm: f64,
    #[serde(default)]
    pub purpose_type: i32,
    #[serde(default)]
    pub opening_type: String,
    #[serde(default)]
    pub is_outside: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Beam {
    pub guid: String,
    pub start_xmm: f64,
    pub start_ymm: f64,
    pub start_zmm: f64,
    pub end_xmm: f64,
    pub end_ymm: f64,
    pub end_zmm: f64,
    pub geometry: BeamGeometry,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BeamGeometry {
    pub width_mm: f64,
    pub height_mm: f64,
    #[serde(default)]
    pub height_direction_x: f64,
    #[serde(default)]
    pub height_direction_y: f64,
    #[serde(default)]
    pub height_direction_z: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct ApiFailure {
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_id: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub source_ids: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub course_index: Option<i64>,
    #[serde(skip_serializing_if = "is_one")]
    pub occurrences: usize,
}

fn is_one(value: &usize) -> bool {
    *value == 1
}

impl ApiFailure {
    pub fn new(
        code: impl Into<String>,
        message: impl Into<String>,
        source_id: Option<String>,
    ) -> Self {
        Self {
            code: code.into(),
            message: message.into(),
            source_id,
            source_ids: Vec::new(),
            course_index: None,
            occurrences: 1,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum LayoutResponse<T: Serialize> {
    Success {
        schema_version: u32,
        request_id: String,
        snapshot_hash: String,
        profile_id: String,
        profile_revision: String,
        catalog_version: String,
        blocks: Vec<T>,
        elapsed_ms: u128,
    },
    Failure {
        schema_version: u32,
        request_id: String,
        snapshot_hash: String,
        profile_id: String,
        profile_revision: String,
        catalog_version: String,
        diagnostics: Vec<ApiFailure>,
    },
}

impl LayoutRequest {
    pub fn validate(&self) -> Result<(), ApiFailure> {
        if self.schema_version != 1 {
            return Err(ApiFailure::new(
                "UNSUPPORTED_VERSION",
                format!("schema_version={} is unsupported", self.schema_version),
                None,
            ));
        }
        if self.wall_volumes.is_empty() {
            return Err(ApiFailure::new(
                "EMPTY_MODEL",
                "wall_volumes is empty",
                None,
            ));
        }
        if !self.z0_mm.is_finite() {
            return Err(ApiFailure::new(
                "INVALID_COORDINATE",
                "z0_mm must be finite",
                None,
            ));
        }
        Ok(())
    }

    pub fn raw_building(&self) -> RawBuilding {
        let walls = self
            .wall_volumes
            .iter()
            .filter(|w| w.purpose_type == 1)
            .map(|w| RawWall {
                id: w.guid.clone(),
                start: RawPoint {
                    x_mm: w.start_xmm,
                    y_mm: w.start_ymm,
                },
                end: RawPoint {
                    x_mm: w.end_xmm,
                    y_mm: w.end_ymm,
                },
                bottom_start_mm: w.start_bottom_zmm,
                top_start_mm: w.start_top_zmm,
                bottom_end_mm: w.end_bottom_zmm,
                top_end_mm: w.end_top_zmm,
                thickness_mm: w.thickness_mm,
            })
            .collect();
        RawBuilding {
            z0_mm: self.z0_mm,
            walls,
        }
    }
}

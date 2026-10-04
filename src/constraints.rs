//! Геометрические ограничения, вычисляемые до раскладки блоков.
//! Координаты входа заданы в мм, интервалы результата — в 0,01 мм.

use crate::domain::{Point, RawPoint, Topology, WallRun};
use std::collections::{HashMap, HashSet};

const SCALE: f64 = 100.0;
const EPS: f64 = 1e-7;

#[derive(Clone, Debug, PartialEq)]
pub struct RawOpening {
    pub id: String,
    pub start: RawPoint,
    pub end: RawPoint,
    pub bottom_start_mm: f64,
    pub bottom_end_mm: f64,
    pub top_start_mm: f64,
    pub top_end_mm: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RawBeam {
    pub id: String,
    pub start: RawPoint,
    pub end: RawPoint,
    pub bottom_start_mm: f64,
    pub bottom_end_mm: f64,
    pub top_start_mm: f64,
    pub top_end_mm: f64,
    pub width_mm: f64,
    pub height_mm: f64,
    /// Компоненты единичного направления высоты SUP.
    pub height_direction_x: f64,
    pub height_direction_y: f64,
    pub height_direction_z: f64,
}

fn beam_is_inclined(beam: &RawBeam) -> bool {
    (beam.end.x_mm - beam.start.x_mm).hypot(beam.end.y_mm - beam.start.y_mm) > EPS
        && ((beam.bottom_end_mm - beam.bottom_start_mm).abs() > EPS
            || beam.height_direction_x.abs() > EPS
            || beam.height_direction_y.abs() > EPS
            || (beam.height_direction_z.abs() - 1.0).abs() > EPS)
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BeamCutPolicy {
    pub longitudinal_full_wall_thickness: bool,
    pub full_course_clearance: bool,
    pub assembly_clearance_mm: f64,
    /// Распознаёт уже включённые в исходный торец 5 мм шипа на мировой сетке.
    pub affine_world_joint_spike_compensation: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Interval {
    pub start: i64,
    pub end: i64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MaskSource {
    Opening(String),
    Beam(String),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Mask {
    pub run_id: String,
    pub course_index: i64,
    pub interval: Interval,
    pub source: MaskSource,
    pub coverage: MaskCoverage,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MaskCoverage {
    FullSectionVoid,
    PartialDepth,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SolidBox {
    pub u: Interval,
    pub v: Interval,
    pub z: Interval,
}

/// Точный вычет балки в локальных координатах WallRun, 0,01 мм.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BeamBoxCut {
    pub beam_id: String,
    pub volume: SolidBox,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Exclusion {
    None,
    FullVoid { source_ids: Vec<String> },
    Partial { source_ids: Vec<String> },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BeamOrientation {
    Longitudinal,
    Transverse,
    Vertical,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BeamClassification {
    pub beam_id: String,
    pub run_id: String,
    pub orientation: BeamOrientation,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LintelCandidate {
    pub opening_id: String,
    pub run_id: String,
    pub course_index: i64,
    pub span: Interval,
    pub left_support: Option<Interval>,
    pub right_support: Option<Interval>,
}

#[derive(Clone, Debug, Default)]
pub struct Constraints {
    /// Ось консоли от корня в стене к свободному торцу; материал не вычитается.
    pub console_axes: HashMap<String, (RawPoint, RawPoint)>,
    pub masks: Vec<Mask>,
    pub lintel_candidates: Vec<LintelCandidate>,
    pub beam_classifications: Vec<BeamClassification>,
    pub evidence: Vec<ConstraintEvidence>,
    pub beam_cut_policy: BeamCutPolicy,
}

#[derive(Clone, Debug)]
pub struct ConstraintEvidence {
    pub run_id: String,
    pub course_index: i64,
    pub source: MaskSource,
    pub coarse_interval: Interval,
    /// Физическая проекция и граница монтажного выреза до приведения к сетке.
    pub beam_projection: Option<BeamProjectionProvenance>,
    course_z: i64,
    course_height: i64,
    geometry: EvidenceGeometry,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BeamProjectionProvenance {
    pub physical_u: (f64, f64),
    pub assembly_u_before_grid: (f64, f64),
    pub clearance_mm: f64,
}

#[derive(Clone, Debug)]
enum EvidenceGeometry {
    Opening {
        opening: RawOpening,
        frame: Frame,
    },
    Beam {
        beam: RawBeam,
        frame: Frame,
        source_u: Interval,
        assembly_envelope: Option<LocalBox>,
        perpendicular_cut: Option<LocalBox>,
    },
}

#[derive(Clone, Copy, Debug)]
struct LocalBox {
    u_low: f64,
    u_high: f64,
    v_low: f64,
    v_high: f64,
    z_low: f64,
    z_high: f64,
}

impl LocalBox {
    fn classify(self, solid: SolidBox) -> Relation {
        let overlaps = self.u_low < solid.u.end as f64
            && (solid.u.start as f64) < self.u_high
            && self.v_low < solid.v.end as f64
            && (solid.v.start as f64) < self.v_high
            && self.z_low < solid.z.end as f64
            && (solid.z.start as f64) < self.z_high;
        if !overlaps {
            return Relation::None;
        }
        if self.u_low <= solid.u.start as f64 + EPS
            && self.u_high >= solid.u.end as f64 - EPS
            && self.v_low <= solid.v.start as f64 + EPS
            && self.v_high >= solid.v.end as f64 - EPS
            && self.z_low <= solid.z.start as f64 + EPS
            && self.z_high >= solid.z.end as f64 - EPS
        {
            Relation::Full
        } else {
            Relation::Partial
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstraintError {
    pub source_id: String,
    pub kind: ConstraintErrorKind,
    pub message: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConstraintErrorKind {
    InvalidGeometry,
    UnsupportedBeamGeometry,
    AmbiguousOpeningRun,
}

#[derive(Clone, Copy)]
struct LocalPoint {
    u: f64,
    v: f64,
    t: f64,
}

#[derive(Clone, Copy, Debug)]
struct Frame {
    start: Point,
    ux: f64,
    uy: f64,
    length: f64,
    half_thickness: f64,
}

impl Frame {
    fn new(run: &WallRun, thickness: i64) -> Option<Self> {
        let dx = (run.end.x - run.start.x) as f64;
        let dy = (run.end.y - run.start.y) as f64;
        let length = dx.hypot(dy);
        (length > EPS && thickness > 0).then_some(Self {
            start: run.start,
            ux: dx / length,
            uy: dy / length,
            length: run.length as f64,
            half_thickness: thickness as f64 / 2.0,
        })
    }

    fn project(&self, point: RawPoint, t: f64) -> LocalPoint {
        let x = point.x_mm * SCALE - self.start.x as f64;
        let y = point.y_mm * SCALE - self.start.y as f64;
        LocalPoint {
            u: x * self.ux + y * self.uy,
            v: -x * self.uy + y * self.ux,
            t,
        }
    }
}

fn nominal_beam_body_u(
    beam: &RawBeam,
    frame: &Frame,
    run: &WallRun,
    course_index: i64,
    physical_u: (f64, f64),
) -> (f64, f64) {
    // Компенсация относится только к соосным ортогональным торцам источника.
    // Грань, полученная обрезкой по стене, не является торцом балки.
    if !((frame.ux - 1.0).abs() <= EPS && frame.uy.abs() <= EPS
        || (frame.uy - 1.0).abs() <= EPS && frame.ux.abs() <= EPS)
    {
        return physical_u;
    }
    let start = frame.project(beam.start, 0.0);
    let end = frame.project(beam.end, 1.0);
    if start.v.abs() > EPS || end.v.abs() > EPS {
        return physical_u;
    }
    let Ok(Some(residue)) = crate::grid::joint_residue(course_index, run) else {
        return physical_u;
    };
    nominal_axis_bounds(physical_u, (start.u.min(end.u), start.u.max(end.u)), residue as f64)
}

/// Общая нормализация уже закодированного припуска шипов источника.
/// Все координаты в сотых мм; исходный запрос остаётся неизменным.
pub(crate) fn nominal_axis_bounds(
    physical_u: (f64, f64),
    source_u: (f64, f64),
    residue: f64,
) -> (f64, f64) {
    let on_half_module = |u: f64| {
        let phase = (u - residue).rem_euclid(32_000.0);
        phase <= EPS || 32_000.0 - phase <= EPS
    };
    let (mut low, mut high) = physical_u;
    if (low - source_u.0).abs() <= EPS && on_half_module(low + 500.0) {
        low += 500.0;
    }
    if (high - source_u.1).abs() <= EPS && on_half_module(high - 500.0) {
        high -= 500.0;
    }
    if high - low <= EPS {
        physical_u
    } else {
        (low, high)
    }
}

fn quantize(value: f64) -> Option<i64> {
    (value.is_finite() && value >= i64::MIN as f64 && value < i64::MAX as f64)
        .then(|| value.round() as i64)
}

fn interval(start: f64, end: f64, length: f64) -> Option<Interval> {
    let start = quantize(start.max(0.0))?;
    let end = quantize(end.min(length))?;
    (end > start).then_some(Interval { start, end })
}

fn outer_interval(start: f64, end: f64, length: f64) -> Option<Interval> {
    let start = quantize(start.max(0.0).floor())?;
    let end = quantize(end.min(length).ceil())?;
    (end > start).then_some(Interval { start, end })
}

fn inner_interval(start: f64, end: f64, length: f64) -> Option<Interval> {
    let start = quantize(start.max(0.0).ceil())?;
    let end = quantize(end.min(length).floor())?;
    (end > start).then_some(Interval { start, end })
}

fn error(id: &str, message: &str) -> ConstraintError {
    ConstraintError {
        source_id: id.into(),
        kind: ConstraintErrorKind::InvalidGeometry,
        message: message.into(),
    }
}

fn unsupported_beam(id: &str) -> ConstraintError {
    ConstraintError {
        source_id: id.into(),
        kind: ConstraintErrorKind::UnsupportedBeamGeometry,
        message: "Ось и направление высоты балки не задают ортогональную 3D-призму".into(),
    }
}

fn valid_point(p: RawPoint) -> bool {
    p.x_mm.is_finite() && p.y_mm.is_finite()
}

fn valid_z(bottom_start: f64, bottom_end: f64, top_start: f64, top_end: f64) -> bool {
    [bottom_start, bottom_end, top_start, top_end]
        .iter()
        .all(|z| z.is_finite())
        && top_start > bottom_start
        && top_end > bottom_end
}

#[derive(Clone, Copy)]
struct Vec3 {
    x: f64,
    y: f64,
    z: f64,
}

impl Vec3 {
    fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
    fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
    fn scale(self, factor: f64) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
    fn sub(self, other: Self) -> Self {
        self.add(other.scale(-1.0))
    }
}

#[derive(Clone, Copy)]
struct Local3 {
    u: f64,
    v: f64,
    z: f64,
}

fn beam_axes(beam: &RawBeam) -> Option<(Vec3, Vec3, Vec3, f64)> {
    let axis = Vec3 {
        x: beam.end.x_mm - beam.start.x_mm,
        y: beam.end.y_mm - beam.start.y_mm,
        z: beam.bottom_end_mm - beam.bottom_start_mm,
    };
    let length = axis.norm();
    let axis = axis.scale(1.0 / length);
    let height = Vec3 {
        x: beam.height_direction_x,
        y: beam.height_direction_y,
        z: beam.height_direction_z,
    };
    let width = axis.cross(height);
    (length > EPS
        && (height.norm() - 1.0).abs() <= 1e-5
        && axis.dot(height).abs() <= 1e-5
        && (width.norm() - 1.0).abs() <= 1e-5)
        .then_some((axis, height, width, length))
}

// Полуплоскость: положительное значение находится внутри. Координата t
// интерполируется вместе с u/v и сохраняет наклон высот по длине балки.
fn clip(poly: &[LocalPoint], f: impl Fn(LocalPoint) -> f64) -> Vec<LocalPoint> {
    let mut out = Vec::new();
    if poly.is_empty() {
        return out;
    }
    let mut a = *poly.last().unwrap();
    let mut fa = f(a);
    for &b in poly {
        let fb = f(b);
        if (fa >= -EPS) != (fb >= -EPS) {
            let ratio = fa / (fa - fb);
            out.push(LocalPoint {
                u: a.u + (b.u - a.u) * ratio,
                v: a.v + (b.v - a.v) * ratio,
                t: a.t + (b.t - a.t) * ratio,
            });
        }
        if fb >= -EPS {
            out.push(b);
        }
        a = b;
        fa = fb;
    }
    out
}

fn clip3(poly: &[Local3], f: impl Fn(Local3) -> f64) -> Vec<Local3> {
    let mut out = Vec::new();
    if poly.is_empty() {
        return out;
    }
    let mut a = *poly.last().unwrap();
    let mut fa = f(a);
    for &b in poly {
        let fb = f(b);
        if (fa >= -EPS) != (fb >= -EPS) {
            let ratio = fa / (fa - fb);
            out.push(Local3 {
                u: a.u + (b.u - a.u) * ratio,
                v: a.v + (b.v - a.v) * ratio,
                z: a.z + (b.z - a.z) * ratio,
            });
        }
        if fb >= -EPS {
            out.push(b);
        }
        a = b;
        fa = fb;
    }
    out
}

fn beam_point(frame: &Frame, global: Vec3) -> Local3 {
    let x = global.x * SCALE - frame.start.x as f64;
    let y = global.y * SCALE - frame.start.y as f64;
    Local3 {
        u: x * frame.ux + y * frame.uy,
        v: -x * frame.uy + y * frame.ux,
        z: global.z * SCALE,
    }
}

fn clipped_beam(
    frame: &Frame,
    beam: &RawBeam,
    u_low: f64,
    u_high: f64,
    v_low: f64,
    v_high: f64,
    z_low: f64,
    z_high: f64,
) -> Vec<Local3> {
    let (axis, height, width, axis_length) = beam_axes(beam).unwrap();
    let origin = Vec3 {
        x: beam.start.x_mm,
        y: beam.start.y_mm,
        z: beam.bottom_start_mm,
    };
    let mut corners = [Local3 {
        u: 0.0,
        v: 0.0,
        z: 0.0,
    }; 8];
    for a in 0..2 {
        for h in 0..2 {
            for w in 0..2 {
                let global = origin
                    .add(axis.scale(axis_length * a as f64))
                    .add(height.scale(beam.height_mm * h as f64))
                    .add(width.scale(beam.width_mm * (w as f64 - 0.5)));
                corners[a * 4 + h * 2 + w] = beam_point(frame, global);
            }
        }
    }
    // Шесть граней призмы. Их пересечения с шестью плоскостями курса
    // содержат все вершины результата, лежащие на исходной призме.
    const FACES: [[usize; 4]; 6] = [
        [0, 1, 3, 2],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 2, 6, 4],
        [1, 3, 7, 5],
    ];
    let mut points = Vec::new();
    for face in FACES {
        let polygon = face.map(|index| corners[index]);
        let polygon = clip3(&polygon, |p| p.u - u_low);
        let polygon = clip3(&polygon, |p| u_high - p.u);
        let polygon = clip3(&polygon, |p| p.v - v_low);
        let polygon = clip3(&polygon, |p| v_high - p.v);
        let polygon = clip3(&polygon, |p| p.z - z_low);
        let polygon = clip3(&polygon, |p| z_high - p.z);
        points.extend(polygon);
    }
    // Если курс целиком внутри балки, ни одна её грань не попадёт в курс.
    // Добавляем углы объёма курса, лежащие внутри ориентированной призмы.
    for u in [u_low, u_high] {
        for v in [v_low, v_high] {
            for z in [z_low, z_high] {
                let x = frame.start.x as f64 / SCALE + (u * frame.ux - v * frame.uy) / SCALE;
                let y = frame.start.y as f64 / SCALE + (u * frame.uy + v * frame.ux) / SCALE;
                let relative = Vec3 { x, y, z: z / SCALE }.sub(origin);
                let along = relative.dot(axis);
                let up = relative.dot(height);
                let across = relative.dot(width);
                if along >= -EPS
                    && along <= axis_length + EPS
                    && up >= -EPS
                    && up <= beam.height_mm + EPS
                    && across.abs() <= beam.width_mm / 2.0 + EPS
                {
                    points.push(Local3 { u, v, z });
                }
            }
        }
    }
    points
}

fn bounds(poly: &[LocalPoint]) -> Option<(f64, f64)> {
    if poly.is_empty() {
        return None;
    }
    let low = poly.iter().map(|p| p.u).fold(f64::INFINITY, f64::min);
    let high = poly.iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
    (high - low > EPS).then_some((low, high))
}

fn bounds3(poly: &[Local3]) -> Option<(f64, f64)> {
    if poly.is_empty() {
        return None;
    }
    let low = poly.iter().map(|p| p.u).fold(f64::INFINITY, f64::min);
    let high = poly.iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
    (high - low > EPS).then_some((low, high))
}

fn positive_volume(points: &[Local3]) -> bool {
    let Some(&origin) = points.first() else {
        return false;
    };
    let delta = |p: Local3| Vec3 {
        x: p.u - origin.u,
        y: p.v - origin.v,
        z: p.z - origin.z,
    };
    let axis = points
        .iter()
        .skip(1)
        .map(|&p| delta(p))
        .max_by(|a, b| a.norm().total_cmp(&b.norm()))
        .unwrap_or(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
    if axis.norm() <= EPS {
        return false;
    }
    let normal = points
        .iter()
        .skip(1)
        .map(|&p| axis.cross(delta(p)))
        .max_by(|a, b| a.norm().total_cmp(&b.norm()))
        .unwrap_or(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
    if normal.norm() <= EPS * axis.norm() {
        return false;
    }
    points
        .iter()
        .skip(1)
        .map(|&p| normal.dot(delta(p)).abs())
        .fold(0.0, f64::max)
        > EPS * normal.norm()
}

fn beam_contains(frame: &Frame, beam: &RawBeam, point: Local3) -> bool {
    let (axis, height, width, axis_length) = beam_axes(beam).unwrap();
    let origin = Vec3 {
        x: beam.start.x_mm,
        y: beam.start.y_mm,
        z: beam.bottom_start_mm,
    };
    let global = Vec3 {
        x: frame.start.x as f64 / SCALE + (point.u * frame.ux - point.v * frame.uy) / SCALE,
        y: frame.start.y as f64 / SCALE + (point.u * frame.uy + point.v * frame.ux) / SCALE,
        z: point.z / SCALE,
    };
    let relative = global.sub(origin);
    relative.dot(axis) >= -EPS
        && relative.dot(axis) <= axis_length + EPS
        && relative.dot(height) >= -EPS
        && relative.dot(height) <= beam.height_mm + EPS
        && relative.dot(width).abs() <= beam.width_mm / 2.0 + EPS
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum Relation {
    None,
    Full,
    Partial,
}

impl ConstraintEvidence {
    fn classify(&self, solid: SolidBox, assembly: bool) -> Relation {
        let z_low = solid.z.start.max(self.course_z) as f64;
        let z_high = solid.z.end.min(self.course_z + self.course_height) as f64;
        if z_high - z_low <= EPS {
            return Relation::None;
        }
        match &self.geometry {
            EvidenceGeometry::Beam {
                beam,
                frame,
                source_u,
                assembly_envelope,
                perpendicular_cut,
            } => {
                if assembly {
                    if let Some(envelope) = assembly_envelope {
                        return envelope.classify(solid);
                    }
                }
                if let Some(cut) = perpendicular_cut {
                    return cut.classify(solid);
                }
                let u_low = solid.u.start.max(source_u.start) as f64;
                let u_high = solid.u.end.min(source_u.end) as f64;
                let v_low = (solid.v.start as f64).max(-frame.half_thickness);
                let v_high = (solid.v.end as f64).min(frame.half_thickness);
                if u_high - u_low <= EPS || v_high - v_low <= EPS {
                    return Relation::None;
                }
                let clipped =
                    clipped_beam(frame, beam, u_low, u_high, v_low, v_high, z_low, z_high);
                if !positive_volume(&clipped) {
                    return Relation::None;
                }
                let inside_section = u_low <= solid.u.start as f64 + EPS
                    && u_high >= solid.u.end as f64 - EPS
                    && v_low <= solid.v.start as f64 + EPS
                    && v_high >= solid.v.end as f64 - EPS
                    && z_low <= solid.z.start as f64 + EPS
                    && z_high >= solid.z.end as f64 - EPS;
                if inside_section
                    && [solid.u.start, solid.u.end].into_iter().all(|u| {
                        [solid.v.start, solid.v.end].into_iter().all(|v| {
                            [solid.z.start, solid.z.end].into_iter().all(|z| {
                                beam_contains(
                                    frame,
                                    beam,
                                    Local3 {
                                        u: u as f64,
                                        v: v as f64,
                                        z: z as f64,
                                    },
                                )
                            })
                        })
                    })
                {
                    Relation::Full
                } else {
                    Relation::Partial
                }
            }
            EvidenceGeometry::Opening { frame, .. } => LocalBox {
                u_low: self.coarse_interval.start as f64,
                u_high: self.coarse_interval.end as f64,
                v_low: -frame.half_thickness,
                v_high: frame.half_thickness,
                z_low: self.course_z as f64,
                z_high: (self.course_z + self.course_height) as f64,
            }
            .classify(solid),
        }
    }
}

impl Constraints {
    pub(crate) fn opening_axis(&self, id: &str) -> Option<(RawPoint, RawPoint)> {
        if let Some(axis) = self.console_axes.get(id) {
            return Some(*axis);
        }
        self.evidence.iter().find_map(|e| match &e.geometry {
            EvidenceGeometry::Opening { opening, .. } if opening.id == id => {
                Some((opening.start, opening.end))
            }
            _ => None,
        })
    }

    /// Возвращает вычеты балок без расширения до венца.
    /// Поперечный проход вычитается перпендикулярно на всю толщину стены.
    pub fn beam_box_cuts(
        &self,
        run_id: &str,
        course_index: i64,
        solid: SolidBox,
    ) -> Result<Vec<BeamBoxCut>, ConstraintError> {
        if [solid.u, solid.v, solid.z]
            .iter()
            .any(|span| span.start >= span.end)
        {
            return Err(error(
                run_id,
                "Кандидатный объём имеет пустой или обратный интервал",
            ));
        }
        let mut cuts = Vec::new();
        for evidence in self.evidence.iter().filter(|item| {
            item.run_id == run_id
                && item.course_index == course_index
                && item.coarse_interval.start < solid.u.end
                && solid.u.start < item.coarse_interval.end
        }) {
            let EvidenceGeometry::Beam {
                beam,
                frame,
                source_u,
                perpendicular_cut,
                ..
            } = &evidence.geometry
            else {
                continue;
            };
            if evidence.classify(solid, false) == Relation::None {
                continue;
            }
            if let Some(cut) = perpendicular_cut {
                let grid = |low, high, span: Interval| -> Result<Interval, ConstraintError> {
                    Ok(Interval {
                        start: quantize(low)
                            .ok_or_else(|| error(&beam.id, "Вычет балки вне координатной сетки"))?
                            .max(span.start),
                        end: quantize(high)
                            .ok_or_else(|| error(&beam.id, "Вычет балки вне координатной сетки"))?
                            .min(span.end),
                    })
                };
                let volume = SolidBox {
                    u: grid(cut.u_low, cut.u_high, solid.u)?,
                    v: grid(cut.v_low, cut.v_high, solid.v)?,
                    z: grid(cut.z_low, cut.z_high, solid.z)?,
                };
                cuts.push(BeamBoxCut {
                    beam_id: beam.id.clone(),
                    volume,
                });
                continue;
            }
            let (axis, height, width, _) =
                beam_axes(beam).ok_or_else(|| unsupported_beam(&beam.id))?;
            for direction in [axis, height, width] {
                let components = [
                    direction.x * frame.ux + direction.y * frame.uy,
                    -direction.x * frame.uy + direction.y * frame.ux,
                    direction.z,
                ];
                if !components
                    .iter()
                    .all(|c| c.abs() <= EPS || (c.abs() - 1.0).abs() <= EPS)
                {
                    return Err(ConstraintError {
                        source_id: beam.id.clone(),
                        kind: ConstraintErrorKind::UnsupportedBeamGeometry,
                        message:
                            "Точный прямоугольный вычет не поддерживает наклон балки к осям стены"
                                .into(),
                    });
                }
            }
            let points = clipped_beam(
                frame,
                beam,
                solid.u.start.max(source_u.start) as f64,
                solid.u.end.min(source_u.end) as f64,
                (solid.v.start as f64).max(-frame.half_thickness),
                (solid.v.end as f64).min(frame.half_thickness),
                solid.z.start.max(evidence.course_z) as f64,
                solid.z.end.min(evidence.course_z + evidence.course_height) as f64,
            );
            let grid_span = |coordinate: fn(&Local3) -> f64| -> Result<Interval, ConstraintError> {
                let low = points.iter().map(coordinate).fold(f64::INFINITY, f64::min);
                let high = points
                    .iter()
                    .map(coordinate)
                    .fold(f64::NEG_INFINITY, f64::max);
                Ok(Interval {
                    start: quantize(low)
                        .ok_or_else(|| error(&beam.id, "Вычет балки вне координатной сетки"))?,
                    end: quantize(high)
                        .ok_or_else(|| error(&beam.id, "Вычет балки вне координатной сетки"))?,
                })
            };
            let volume = SolidBox {
                u: grid_span(|p| p.u)?,
                v: grid_span(|p| p.v)?,
                z: grid_span(|p| p.z)?,
            };
            if [volume.u, volume.v, volume.z]
                .iter()
                .all(|span| span.start < span.end)
            {
                cuts.push(BeamBoxCut {
                    beam_id: beam.id.clone(),
                    volume,
                });
            }
        }
        cuts.sort_by_key(|cut| {
            (
                cut.beam_id.clone(),
                cut.volume.u.start,
                cut.volume.v.start,
                cut.volume.z.start,
                cut.volume.u.end,
                cut.volume.v.end,
                cut.volume.z.end,
            )
        });
        cuts.dedup();
        Ok(cuts)
    }

    /// Классифицирует фактическое 3D-пересечение кандидата с исходной геометрией.
    /// Интервалы полуоткрытые, координаты в локальной системе WallRun, 0,01 мм.
    pub fn classify_box(
        &self,
        run_id: &str,
        course_index: i64,
        solid: SolidBox,
    ) -> Result<Exclusion, ConstraintError> {
        self.classify_box_mode(run_id, course_index, solid, false)
    }

    /// Проверяет отдельно заданный монтажный вырез. Физический предикат — `classify_box`.
    pub fn classify_assembly_box(
        &self,
        run_id: &str,
        course_index: i64,
        solid: SolidBox,
    ) -> Result<Exclusion, ConstraintError> {
        self.classify_box_mode(run_id, course_index, solid, true)
    }

    fn classify_box_mode(
        &self,
        run_id: &str,
        course_index: i64,
        solid: SolidBox,
        assembly: bool,
    ) -> Result<Exclusion, ConstraintError> {
        if [solid.u, solid.v, solid.z]
            .iter()
            .any(|span| span.start >= span.end)
        {
            return Err(error(
                run_id,
                "Кандидатный объём имеет пустой или обратный интервал",
            ));
        }
        let mut partial = Vec::new();
        let mut full = Vec::new();
        for evidence in self.evidence.iter().filter(|item| {
            item.run_id == run_id
                && item.course_index == course_index
                && item.coarse_interval.start < solid.u.end
                && solid.u.start < item.coarse_interval.end
        }) {
            let id = match &evidence.source {
                MaskSource::Opening(id) => format!("opening:{id}"),
                MaskSource::Beam(id) => format!("beam:{id}"),
            };
            match evidence.classify(solid, assembly) {
                Relation::None => {}
                Relation::Full => full.push(id),
                Relation::Partial => partial.push(id),
            }
        }
        full.sort();
        full.dedup();
        partial.sort();
        partial.dedup();
        if !full.is_empty() {
            Ok(Exclusion::FullVoid { source_ids: full })
        } else if !partial.is_empty() {
            Ok(Exclusion::Partial {
                source_ids: partial,
            })
        } else {
            Ok(Exclusion::None)
        }
    }
}

fn z_at(start: f64, end: f64, t: f64) -> f64 {
    (start + (end - start) * t) * SCALE
}

fn overlapping_z(
    poly: &[LocalPoint],
    course_z: i64,
    height: i64,
    bottom_start: f64,
    bottom_end: f64,
    top_start: f64,
    top_end: f64,
) -> Vec<LocalPoint> {
    if !poly
        .iter()
        .any(|p| z_at(top_start, top_end, p.t) - course_z as f64 > EPS)
        || !poly
            .iter()
            .any(|p| course_z as f64 + height as f64 - z_at(bottom_start, bottom_end, p.t) > EPS)
    {
        return Vec::new();
    }
    let top = course_z as f64 + height as f64;
    let poly = clip(poly, |p| {
        z_at(top_start, top_end, p.t) - course_z as f64 - EPS
    });
    clip(&poly, |p| top - z_at(bottom_start, bottom_end, p.t) - EPS)
}

/// Создаёт маски пустоты и возможные опоры перемычек перед раскладкой.
/// `course.z` — низ курса; все высоты входа абсолютные, в мм.
pub fn build_constraints(
    topology: &Topology,
    openings: &[RawOpening],
    beams: &[RawBeam],
    course_height_mm: f64,
    lintel_support_mm: f64,
) -> Result<Constraints, Vec<ConstraintError>> {
    build_constraints_with_policy(
        topology,
        openings,
        beams,
        course_height_mm,
        lintel_support_mm,
        BeamCutPolicy::default(),
    )
}

pub fn build_constraints_with_policy(
    topology: &Topology,
    openings: &[RawOpening],
    beams: &[RawBeam],
    course_height_mm: f64,
    lintel_support_mm: f64,
    policy: BeamCutPolicy,
) -> Result<Constraints, Vec<ConstraintError>> {
    let mut errors = Vec::new();
    if !policy.assembly_clearance_mm.is_finite() || policy.assembly_clearance_mm < 0.0 {
        errors.push(error(
            "beam-cut-policy",
            "Монтажный зазор должен быть конечным неотрицательным числом",
        ));
    }
    let course_height = quantize(course_height_mm * SCALE).filter(|v| *v > 0);
    let support = quantize(lintel_support_mm * SCALE).filter(|v| *v > 0);
    if course_height.is_none() {
        errors.push(error(
            "courses",
            "Высота курса должна быть положительным конечным числом",
        ));
    }
    if support.is_none() {
        errors.push(error(
            "lintel",
            "Опирание перемычки должно быть положительным конечным числом",
        ));
    }
    for opening in openings {
        if opening.id.is_empty()
            || !valid_point(opening.start)
            || !valid_point(opening.end)
            || !valid_z(
                opening.bottom_start_mm,
                opening.bottom_end_mm,
                opening.top_start_mm,
                opening.top_end_mm,
            )
            || (opening.end.x_mm - opening.start.x_mm).hypot(opening.end.y_mm - opening.start.y_mm)
                <= EPS
        {
            errors.push(error(&opening.id, "Некорректная геометрия проёма"));
        }
    }
    for beam in beams {
        if beam.id.is_empty()
            || !valid_point(beam.start)
            || !valid_point(beam.end)
            || [
                beam.bottom_start_mm,
                beam.bottom_end_mm,
                beam.top_start_mm,
                beam.top_end_mm,
            ]
            .iter()
            .any(|z| !z.is_finite())
            || !beam.width_mm.is_finite()
            || beam.width_mm <= 0.0
            || !beam.height_mm.is_finite()
            || beam.height_mm <= 0.0
            || !beam.height_direction_x.is_finite()
            || !beam.height_direction_y.is_finite()
            || !beam.height_direction_z.is_finite()
            || (beam.end.x_mm - beam.start.x_mm).hypot(beam.end.y_mm - beam.start.y_mm)
                + (beam.bottom_end_mm - beam.bottom_start_mm).abs()
                <= EPS
        {
            errors.push(error(&beam.id, "Некорректная геометрия балки"));
        } else if !beam_is_inclined(beam)
            && (beam_axes(beam).is_none()
                || (beam.top_start_mm
                    - beam.bottom_start_mm
                    - beam.height_mm * beam.height_direction_z)
                    .abs()
                    > 0.01
                || (beam.top_end_mm
                    - beam.bottom_end_mm
                    - beam.height_mm * beam.height_direction_z)
                    .abs()
                    > 0.01)
        {
            errors.push(unsupported_beam(&beam.id));
        }
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    let course_height = course_height.unwrap();
    let support = support.unwrap();
    // Keep the request untouched; course-expanded dimensions belong to constraints.
    let expanded_openings: Vec<_> = openings
        .iter()
        .cloned()
        .map(|mut opening| {
            let z0 = topology.z0 as f64;
            let step = course_height as f64;
            let floor = |z: f64| (z0 + ((z * SCALE - z0) / step).floor() * step) / SCALE;
            let ceil = |z: f64| (z0 + ((z * SCALE - z0) / step).ceil() * step) / SCALE;
            opening.bottom_start_mm = floor(opening.bottom_start_mm);
            opening.bottom_end_mm = floor(opening.bottom_end_mm);
            opening.top_start_mm = ceil(opening.top_start_mm);
            opening.top_end_mm = ceil(opening.top_end_mm);
            opening
        })
        .collect();
    for opening in &expanded_openings {
        if [
            opening.bottom_start_mm,
            opening.bottom_end_mm,
            opening.top_start_mm,
            opening.top_end_mm,
        ]
        .iter()
        .any(|z| quantize(z * SCALE).is_none())
        {
            errors.push(error(
                &opening.id,
                "Расширенные границы проёма вне координатной сетки",
            ));
        }
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    let openings = expanded_openings.as_slice();
    let mut result = Constraints {
        beam_cut_policy: policy,
        ..Constraints::default()
    };
    let mut attached_openings = HashSet::new();
    let mut opening_runs: HashMap<(&str, i64), HashSet<&str>> = HashMap::new();
    let mut run_sections: HashMap<(String, i64), (Interval, Interval)> = HashMap::new();

    for course in &topology.courses {
        for run in &course.runs {
            let mut source_frames = Vec::new();
            for source in &run.sources {
                let Some(wall) = topology.walls.iter().find(|w| w.id == source.wall_id) else {
                    errors.push(error(
                        &source.wall_id,
                        "Для источника прогона отсутствует стена",
                    ));
                    continue;
                };
                if source.start_offset < 0
                    || source.end_offset > run.length
                    || source.start_offset >= source.end_offset
                {
                    errors.push(error(&run.id, "Некорректный интервал источника прогона"));
                    continue;
                }
                let Some(frame) = Frame::new(run, wall.thickness) else {
                    errors.push(error(&run.id, "Прогон имеет нулевую длину или толщину"));
                    continue;
                };
                source_frames.push((frame, source.start_offset as f64, source.end_offset as f64));
            }
            let Some(frame) = source_frames.first().map(|item| item.0) else {
                errors.push(error(&run.id, "Прогон не содержит проверенных источников"));
                continue;
            };
            if source_frames
                .iter()
                .any(|item| (item.0.half_thickness - frame.half_thickness).abs() > EPS)
            {
                errors.push(error(
                    &run.id,
                    "В одном прогоне объединены стены разной толщины",
                ));
                continue;
            }
            run_sections.insert(
                (run.id.clone(), course.index),
                (
                    Interval {
                        start: (-frame.half_thickness).floor() as i64,
                        end: frame.half_thickness.ceil() as i64,
                    },
                    Interval {
                        start: course.z,
                        end: course.z + course_height,
                    },
                ),
            );
            for opening in openings {
                let a = frame.project(opening.start, 0.0);
                let b = frame.project(opening.end, 1.0);
                let lo = a.u.min(b.u).max(0.0);
                let hi = a.u.max(b.u).min(frame.length);
                if hi - lo <= EPS
                    || !source_frames.iter().any(|(source_frame, from, to)| {
                        a.v.abs() <= source_frame.half_thickness + EPS
                            && b.v.abs() <= source_frame.half_thickness + EPS
                            && hi > *from + EPS
                            && lo < *to - EPS
                    })
                {
                    continue;
                }
                attached_openings.insert(opening.id.as_str());
                opening_runs
                    .entry((&opening.id, course.index))
                    .or_default()
                    .insert(&run.id);
                let mut poly = vec![a, b];
                poly = clip(&poly, |p| p.u);
                poly = clip(&poly, |p| frame.length - p.u);
                let occupied = overlapping_z(
                    &poly,
                    course.z,
                    course_height,
                    opening.bottom_start_mm,
                    opening.bottom_end_mm,
                    opening.top_start_mm,
                    opening.top_end_mm,
                );
                if let Some((from, to)) = bounds(&occupied) {
                    if let Some(coarse) = outer_interval(from, to, frame.length) {
                        // Any intersected course is cleared through its full height.
                        let interval = coarse;
                        let coverage = MaskCoverage::FullSectionVoid;
                        result.masks.push(Mask {
                            run_id: run.id.clone(),
                            course_index: course.index,
                            interval,
                            source: MaskSource::Opening(opening.id.clone()),
                            coverage,
                        });
                        result.evidence.push(ConstraintEvidence {
                            run_id: run.id.clone(),
                            course_index: course.index,
                            source: MaskSource::Opening(opening.id.clone()),
                            coarse_interval: interval,
                            beam_projection: None,
                            course_z: course.z,
                            course_height,
                            geometry: EvidenceGeometry::Opening {
                                opening: opening.clone(),
                                frame,
                            },
                        });
                    }
                }
                let max_top = poly
                    .iter()
                    .map(|point| z_at(opening.top_start_mm, opening.top_end_mm, point.t))
                    .fold(f64::NEG_INFINITY, f64::max);
                // Ряды перемычек чередуются через венец: 1/3/5 для
                // проёма до метра, 1/3/5/7/9 для более широкого проёма.
                let row = ((course.z as f64 - max_top) / course_height as f64).floor();
                let opening_width = (opening.end.x_mm - opening.start.x_mm)
                    .hypot(opening.end.y_mm - opening.start.y_mm)
                    * SCALE;
                let row_count = if opening_width <= 100_000.0 + EPS {
                    3
                } else {
                    5
                };
                let flat_top = (opening.top_start_mm - opening.top_end_mm).abs() <= EPS;
                if course.z as f64 + EPS >= max_top
                    && opening_width <= 150_000.0 + EPS
                    && row >= 0.0
                    && ((flat_top && row < (row_count * 2) as f64 && row as i64 % 2 == 0)
                        || (!flat_top && row == 0.0))
                {
                    if let Some(span) = interval(lo, hi, frame.length) {
                        let left_support = (lo + EPS >= support as f64)
                            .then(|| interval(lo - support as f64, lo, frame.length))
                            .flatten();
                        let right_support = (hi + support as f64 <= frame.length + EPS)
                            .then(|| interval(hi, hi + support as f64, frame.length))
                            .flatten();
                        result.lintel_candidates.push(LintelCandidate {
                            opening_id: opening.id.clone(),
                            run_id: run.id.clone(),
                            course_index: course.index,
                            span,
                            left_support,
                            right_support,
                        });
                    }
                }
            }
            for beam in beams {
                let dx = beam.end.x_mm - beam.start.x_mm;
                let dy = beam.end.y_mm - beam.start.y_mm;
                if beam_is_inclined(beam) {
                    continue;
                }
                let axial = (dx * frame.ux + dy * frame.uy).abs();
                let across = (-dx * frame.uy + dy * frame.ux).abs();
                let orientation = if dx.hypot(dy) <= EPS {
                    BeamOrientation::Vertical
                } else if axial >= across {
                    BeamOrientation::Longitudinal
                } else {
                    BeamOrientation::Transverse
                };
                let assembly_longitudinal = !policy.full_course_clearance
                    && policy.longitudinal_full_wall_thickness
                    && orientation == BeamOrientation::Longitudinal;
                let full_course_clearance =
                    policy.full_course_clearance && orientation != BeamOrientation::Transverse;
                // Размер разделяющего разреза совпадает с номинальным телом балки.
                let clearance = 0.0;
                let z_scan_clearance = if assembly_longitudinal {
                    clearance
                } else {
                    0.0
                };
                let mut parts = Vec::new();
                for (source_frame, from, to) in &source_frames {
                    let clipped = clipped_beam(
                        source_frame,
                        beam,
                        *from,
                        *to,
                        -source_frame.half_thickness,
                        source_frame.half_thickness,
                        course.z as f64 - z_scan_clearance,
                        (course.z + course_height) as f64 + z_scan_clearance,
                    );
                    let Some((mut low_u, mut high_u)) = bounds3(&clipped) else {
                        continue;
                    };
                    let low_v = clipped.iter().map(|p| p.v).fold(f64::INFINITY, f64::min);
                    let high_v = clipped
                        .iter()
                        .map(|p| p.v)
                        .fold(f64::NEG_INFINITY, f64::max);
                    let low_z = clipped.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
                    let high_z = clipped
                        .iter()
                        .map(|p| p.z)
                        .fold(f64::NEG_INFINITY, f64::max);
                    if high_v - low_v <= EPS || high_z - low_z <= EPS || !positive_volume(&clipped)
                    {
                        continue;
                    }
                    let physical_u = (low_u, high_u);
                    if policy.affine_world_joint_spike_compensation
                        && orientation == BeamOrientation::Longitudinal
                    {
                        (low_u, high_u) =
                            nominal_beam_body_u(beam, source_frame, run, course.index, physical_u);
                    }
                    // Для прохода через стену ширина не зависит от угла в плане:
                    // прямоугольник центрируется в пересечении исходных осей.
                    let perpendicular_cut = if orientation == BeamOrientation::Transverse {
                        let start_u = (beam.start.x_mm * SCALE - source_frame.start.x as f64)
                            * source_frame.ux
                            + (beam.start.y_mm * SCALE - source_frame.start.y as f64)
                                * source_frame.uy;
                        let start_v = -(beam.start.x_mm * SCALE - source_frame.start.x as f64)
                            * source_frame.uy
                            + (beam.start.y_mm * SCALE - source_frame.start.y as f64)
                                * source_frame.ux;
                        let delta_u = dx * source_frame.ux + dy * source_frame.uy;
                        let delta_v = -dx * source_frame.uy + dy * source_frame.ux;
                        let center_u = start_u - start_v * delta_u / delta_v;
                        low_u = (center_u - beam.width_mm * SCALE / 2.0).max(*from);
                        high_u = (center_u + beam.width_mm * SCALE / 2.0).min(*to);
                        if high_u - low_u <= EPS {
                            continue;
                        }
                        Some(LocalBox {
                            u_low: low_u,
                            u_high: high_u,
                            v_low: -source_frame.half_thickness,
                            v_high: source_frame.half_thickness,
                            z_low: low_z,
                            z_high: high_z,
                        })
                    } else {
                        None
                    };
                    // Любое пересечение балки является разделяющим разрезом изделия,
                    // а не несквозным карманом по толщине или высоте.
                    let raw_assembly_envelope = {
                        Some(LocalBox {
                            u_low: low_u,
                            u_high: high_u,
                            v_low: -source_frame.half_thickness,
                            v_high: source_frame.half_thickness,
                            z_low: course.z as f64,
                            z_high: (course.z + course_height) as f64,
                        })
                    };
                    let raw_mask_low = raw_assembly_envelope.map_or(low_u, |shape| shape.u_low);
                    let raw_mask_high = raw_assembly_envelope.map_or(high_u, |shape| shape.u_high);
                    let assembly_envelope = if full_course_clearance {
                        raw_assembly_envelope.and_then(|shape| {
                            outer_interval(shape.u_low, shape.u_high, frame.length).map(|grid| {
                                LocalBox {
                                    u_low: grid.start as f64,
                                    u_high: grid.end as f64,
                                    ..shape
                                }
                            })
                        })
                    } else {
                        raw_assembly_envelope
                    };
                    if assembly_envelope.is_some_and(|shape| shape.z_high - shape.z_low <= EPS) {
                        continue;
                    }
                    let mask_low = assembly_envelope.map_or(low_u, |box_| box_.u_low);
                    let mask_high = assembly_envelope.map_or(high_u, |box_| box_.u_high);
                    let coarse = if full_course_clearance {
                        outer_interval(mask_low, mask_high, frame.length)
                    } else {
                        interval(mask_low, mask_high, frame.length)
                    };
                    if let Some(piece) = coarse {
                        parts.push(piece);
                        result.evidence.push(ConstraintEvidence {
                            run_id: run.id.clone(),
                            course_index: course.index,
                            source: MaskSource::Beam(beam.id.clone()),
                            coarse_interval: piece,
                            beam_projection: Some(BeamProjectionProvenance {
                                physical_u,
                                assembly_u_before_grid: (raw_mask_low, raw_mask_high),
                                clearance_mm: clearance / SCALE,
                            }),
                            course_z: course.z,
                            course_height,
                            geometry: EvidenceGeometry::Beam {
                                beam: beam.clone(),
                                frame: *source_frame,
                                source_u: Interval {
                                    start: *from as i64,
                                    end: *to as i64,
                                },
                                assembly_envelope,
                                perpendicular_cut,
                            },
                        });
                        if full_course_clearance {
                            let full = inner_interval(mask_low, mask_high, frame.length);
                            let mut push_piece = |span: Interval| {
                                result.masks.push(Mask {
                                    run_id: run.id.clone(),
                                    course_index: course.index,
                                    interval: span,
                                    source: MaskSource::Beam(beam.id.clone()),
                                    coverage: MaskCoverage::PartialDepth,
                                })
                            };
                            if let Some(inner) = full {
                                if piece.start < inner.start {
                                    push_piece(Interval {
                                        start: piece.start,
                                        end: inner.start,
                                    });
                                }
                                push_piece(inner);
                                if inner.end < piece.end {
                                    push_piece(Interval {
                                        start: inner.end,
                                        end: piece.end,
                                    });
                                }
                            } else {
                                push_piece(piece);
                            }
                        }
                    }
                }
                if parts.is_empty() {
                    continue;
                }
                if !full_course_clearance {
                    // Keep source boundaries: a merged mask can cross several evidence
                    // boxes although each source clears the entire wall section.
                    parts.sort_by_key(|part| (part.start, part.end));
                    parts.dedup();
                    // Для поперечных проходов границы источников также важны:
                    // общий интервал может быть покрыт несколькими evidence,
                    // тогда проверка одного evidence не доказывает FullVoid.
                    for interval in parts {
                        result.masks.push(Mask {
                            run_id: run.id.clone(),
                            course_index: course.index,
                            interval,
                            source: MaskSource::Beam(beam.id.clone()),
                            coverage: MaskCoverage::PartialDepth,
                        });
                    }
                }
                if !result
                    .beam_classifications
                    .iter()
                    .any(|c| c.beam_id == beam.id && c.run_id == run.id)
                {
                    result.beam_classifications.push(BeamClassification {
                        beam_id: beam.id.clone(),
                        run_id: run.id.clone(),
                        orientation,
                    });
                }
            }
        }
    }
    for opening in openings {
        if !attached_openings.contains(opening.id.as_str()) {
            errors.push(error(&opening.id, "Проём не пересекает ни одну стену"));
        }
    }
    for ((opening_id, course_index), runs) in opening_runs {
        if runs.len() > 1 {
            let collinear = topology
                .courses
                .iter()
                .find(|c| c.index == course_index)
                .is_some_and(|course| {
                    let selected: Vec<_> = course
                        .runs
                        .iter()
                        .filter(|r| runs.contains(r.id.as_str()))
                        .collect();
                    let Some(first) = selected.first() else {
                        return false;
                    };
                    let dx = i128::from(first.end.x) - i128::from(first.start.x);
                    let dy = i128::from(first.end.y) - i128::from(first.start.y);
                    selected.iter().all(|r| {
                        [r.start, r.end].iter().all(|p| {
                            (i128::from(p.x) - i128::from(first.start.x)) * dy
                                == (i128::from(p.y) - i128::from(first.start.y)) * dx
                        })
                    })
                });
            if collinear {
                continue;
            }
            errors.push(ConstraintError {
                source_id: opening_id.into(),
                kind: ConstraintErrorKind::AmbiguousOpeningRun,
                message: format!(
                    "Проём пересекает {} разных конструктивных прогонов в курсе {}",
                    runs.len(),
                    course_index
                ),
            });
        }
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    for index in 0..result.masks.len() {
        let mask = &result.masks[index];
        let Some((v, z)) = run_sections.get(&(mask.run_id.clone(), mask.course_index)) else {
            continue;
        };
        let classification = result
            .classify_assembly_box(
                &mask.run_id,
                mask.course_index,
                SolidBox {
                    u: mask.interval,
                    v: *v,
                    z: *z,
                },
            )
            .map_err(|error| vec![error])?;
        if matches!(classification, Exclusion::FullVoid { .. }) {
            result.masks[index].coverage = MaskCoverage::FullSectionVoid;
        }
    }
    for index in 0..result.lintel_candidates.len() {
        let candidate = &result.lintel_candidates[index];
        let Some((v, z)) = run_sections.get(&(candidate.run_id.clone(), candidate.course_index))
        else {
            continue;
        };
        let blocked = |support: Interval| {
            !matches!(
                result.classify_assembly_box(
                    &candidate.run_id,
                    candidate.course_index,
                    SolidBox {
                        u: support,
                        v: *v,
                        z: *z
                    }
                ),
                Ok(Exclusion::None)
            )
        };
        let left_blocked = candidate.left_support.is_some_and(blocked);
        let right_blocked = candidate.right_support.is_some_and(blocked);
        if left_blocked {
            result.lintel_candidates[index].left_support = None;
        }
        if right_blocked {
            result.lintel_candidates[index].right_support = None;
        }
    }
    result.masks.sort_by(|a, b| {
        (&a.run_id, a.course_index, a.interval.start).cmp(&(
            &b.run_id,
            b.course_index,
            b.interval.start,
        ))
    });
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{Course, CourseEdge, NormalizedWall, RunSource};

    fn point(x: i64, y: i64) -> Point {
        Point { x, y }
    }
    fn raw(x: f64, y: f64) -> RawPoint {
        RawPoint { x_mm: x, y_mm: y }
    }

    fn topology() -> Topology {
        let wall = NormalizedWall {
            id: "wall".into(),
            start: point(0, 0),
            end: point(400_000, 0),
            bottom_start: 0,
            top_start: 100_000,
            bottom_end: 0,
            top_end: 100_000,
            thickness: 20_000,
        };
        let courses = (0..4)
            .map(|index| Course {
                index,
                z: index * 20_000,
                edges: vec![CourseEdge {
                    edge_id: "edge".into(),
                    wall_id: "wall".into(),
                    start: wall.start,
                    end: wall.end,
                }],
                runs: vec![WallRun {
                    id: "run".into(),
                    start: wall.start,
                    end: wall.end,
                    length: 400_000,
                    sources: vec![RunSource {
                        wall_id: "wall".into(),
                        edge_id: "edge".into(),
                        start: wall.start,
                        end: wall.end,
                        start_offset: 0,
                        end_offset: 400_000,
                    }],
                }],
                vertices: Vec::new(),
            })
            .collect();
        Topology {
            z0: 0,
            walls: vec![wall],
            vertices: Vec::new(),
            edges: Vec::new(),
            courses,
            vertical_links: Vec::new(),
        }
    }

    fn opening() -> RawOpening {
        RawOpening {
            id: "opening".into(),
            start: raw(1000.0, 0.0),
            end: raw(2000.0, 0.0),
            bottom_start_mm: 0.0,
            bottom_end_mm: 0.0,
            top_start_mm: 200.0,
            top_end_mm: 200.0,
        }
    }

    fn beam(start: RawPoint, end: RawPoint, width_mm: f64) -> RawBeam {
        RawBeam {
            id: "beam".into(),
            start,
            end,
            bottom_start_mm: 0.0,
            bottom_end_mm: 0.0,
            top_start_mm: 200.0,
            top_end_mm: 200.0,
            width_mm,
            height_mm: 200.0,
            height_direction_x: 0.0,
            height_direction_y: 0.0,
            height_direction_z: 1.0,
        }
    }

    #[test]
    fn touching_course_or_wall_does_not_mask() {
        let outside = beam(raw(1000.0, 200.0), raw(2000.0, 200.0), 200.0);
        let result =
            build_constraints(&topology(), &[opening()], &[outside], 200.0, 300.0).unwrap();
        assert_eq!(result.masks.len(), 1);
        assert_eq!(result.masks[0].course_index, 0);
        assert_eq!(
            result.masks[0].interval,
            Interval {
                start: 100_000,
                end: 200_000
            }
        );
        assert!(result.beam_classifications.is_empty());
    }

    #[test]
    fn longitudinal_and_transverse_beams_use_real_width() {
        let long = beam(raw(1000.0, 50.0), raw(2000.0, 50.0), 200.0);
        let mut cross = beam(raw(1500.0, -200.0), raw(1500.0, 200.0), 300.0);
        cross.id = "cross".into();
        let result = build_constraints(&topology(), &[], &[long, cross], 200.0, 300.0).unwrap();
        assert_eq!(result.beam_classifications.len(), 2);
        assert_eq!(
            result.beam_classifications[0].orientation,
            BeamOrientation::Longitudinal
        );
        assert_eq!(
            result.beam_classifications[1].orientation,
            BeamOrientation::Transverse
        );
        assert!(result
            .masks
            .iter()
            .any(|m| m.source == MaskSource::Beam("beam".into())
                && m.interval
                    == Interval {
                        start: 100_000,
                        end: 200_000
                    }));
        assert!(result
            .masks
            .iter()
            .any(|m| m.source == MaskSource::Beam("cross".into())
                && m.interval
                    == Interval {
                        start: 135_000,
                        end: 165_000
                    }));
    }

    #[test]
    fn opening_provides_lintel_support_on_alternating_courses_above_top() {
        let result = build_constraints(&topology(), &[opening()], &[], 200.0, 300.0).unwrap();
        assert_eq!(result.lintel_candidates.len(), 2);
        assert_eq!(result.lintel_candidates[1].course_index, 3);
        let candidate = &result.lintel_candidates[0];
        assert_eq!(candidate.course_index, 1);
        assert_eq!(
            candidate.span,
            Interval {
                start: 100_000,
                end: 200_000
            }
        );
        assert_eq!(
            candidate.left_support,
            Some(Interval {
                start: 70_000,
                end: 100_000
            })
        );
        assert_eq!(
            candidate.right_support,
            Some(Interval {
                start: 200_000,
                end: 230_000
            })
        );
    }

    #[test]
    fn nonaligned_opening_expands_relative_to_building_z0_without_changing_input() {
        let mut topology = topology();
        topology.z0 = 5_000;
        for course in &mut topology.courses {
            course.z += 5_000;
        }
        let mut input = opening();
        input.bottom_start_mm = 70.0;
        input.bottom_end_mm = 70.0;
        input.top_start_mm = 300.0;
        input.top_end_mm = 300.0;
        let result = build_constraints(&topology, &[input.clone()], &[], 200.0, 300.0).unwrap();
        let courses: Vec<_> = result
            .masks
            .iter()
            .map(|m| (m.course_index, m.coverage))
            .collect();
        assert_eq!(
            courses,
            vec![
                (0, MaskCoverage::FullSectionVoid),
                (1, MaskCoverage::FullSectionVoid)
            ]
        );
        assert_eq!(input.bottom_start_mm, 70.0);
        assert_eq!(input.top_start_mm, 300.0);
        assert!(matches!(
            result
                .classify_box(
                    "run",
                    0,
                    SolidBox {
                        u: Interval {
                            start: 110_000,
                            end: 120_000
                        },
                        v: Interval {
                            start: -10_000,
                            end: 10_000
                        },
                        z: Interval {
                            start: 5_000,
                            end: 25_000
                        },
                    }
                )
                .unwrap(),
            Exclusion::FullVoid { .. }
        ));
    }

    #[test]
    fn sloped_top_expands_to_courses_and_clears_their_height() {
        let mut opening = opening();
        opening.top_start_mm = 300.0;
        opening.top_end_mm = 500.0;
        let result = build_constraints(&topology(), &[opening], &[], 200.0, 300.0).unwrap();
        assert!(result.masks.iter().any(|m| m.course_index == 2
            && m.interval
                == Interval {
                    start: 100_000,
                    end: 200_000
                }));
        assert_eq!(result.lintel_candidates[0].course_index, 3);
        let masks: Vec<_> = result
            .masks
            .iter()
            .filter(|m| m.course_index == 1)
            .map(|m| (m.interval, m.coverage))
            .collect();
        assert_eq!(
            masks,
            vec![(
                Interval {
                    start: 100_000,
                    end: 200_000
                },
                MaskCoverage::FullSectionVoid
            )]
        );
    }

    #[test]
    fn slanted_opening_clears_full_height_of_each_intersected_course() {
        let mut opening = opening();
        opening.bottom_end_mm = 300.0;
        opening.top_start_mm = 300.0;
        opening.top_end_mm = 600.0;
        let result = build_constraints(&topology(), &[opening], &[], 200.0, 300.0).unwrap();
        let masks: Vec<_> = result
            .masks
            .iter()
            .filter(|m| m.course_index == 1)
            .map(|m| (m.interval, m.coverage))
            .collect();
        assert_eq!(
            masks,
            vec![(
                Interval {
                    start: 100_000,
                    end: 200_000
                },
                MaskCoverage::FullSectionVoid
            )]
        );
        let full = SolidBox {
            u: Interval {
                start: 140_000,
                end: 150_000,
            },
            v: Interval {
                start: -10_000,
                end: 10_000,
            },
            z: Interval {
                start: 20_000,
                end: 40_000,
            },
        };
        assert!(matches!(
            result.classify_box("run", 1, full).unwrap(),
            Exclusion::FullVoid { .. }
        ));
        assert!(matches!(
            result
                .classify_box(
                    "run",
                    1,
                    SolidBox {
                        u: Interval {
                            start: 110_000,
                            end: 120_000
                        },
                        ..full
                    }
                )
                .unwrap(),
            Exclusion::FullVoid { .. }
        ));
        assert!(result
            .evidence
            .iter()
            .filter(|e| e.course_index == 1)
            .all(|e| e.source == MaskSource::Opening("opening".into())));
    }

    #[test]
    fn inclined_height_direction_is_ignored_without_geometry_error() {
        let mut beam = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 200.0);
        beam.height_direction_z = 0.8;
        let result = build_constraints(&topology(), &[], &[beam], 200.0, 300.0).unwrap();
        assert!(result.masks.is_empty());
        assert!(result.evidence.is_empty());
    }

    #[test]
    fn vertical_column_occupies_three_courses_with_its_section() {
        let mut column = beam(raw(1500.0, 0.0), raw(1500.0, 0.0), 100.0);
        column.bottom_end_mm = 600.0;
        column.top_start_mm = 0.0;
        column.top_end_mm = 600.0;
        column.height_direction_x = 1.0;
        column.height_direction_z = 0.0;
        let result = build_constraints(&topology(), &[], &[column], 200.0, 300.0).unwrap();
        assert_eq!(
            result.beam_classifications[0].orientation,
            BeamOrientation::Vertical
        );
        assert_eq!(
            result
                .masks
                .iter()
                .map(|mask| mask.course_index)
                .collect::<Vec<_>>(),
            vec![0, 1, 2]
        );
        assert!(result.masks.iter().all(|mask| mask.interval
            == Interval {
                start: 150_000,
                end: 170_000
            }));
    }

    #[test]
    fn inclined_axis_and_height_section_are_ignored() {
        let mut inclined = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 200.0);
        inclined.bottom_end_mm = 500.0;
        inclined.height_direction_x = -1.0 / 5.0_f64.sqrt();
        inclined.height_direction_z = 2.0 / 5.0_f64.sqrt();
        inclined.top_start_mm = inclined.height_mm * inclined.height_direction_z;
        inclined.top_end_mm = 500.0 + inclined.top_start_mm;
        let result = build_constraints(&topology(), &[], &[inclined], 200.0, 300.0).unwrap();
        assert!(result.beam_classifications.is_empty());
        assert!(result.masks.is_empty());
        assert!(result.evidence.is_empty());
    }

    #[test]
    fn transparent_i_split_preserves_one_opening_and_beam_across_sources() {
        let mut topology = topology();
        for course in &mut topology.courses {
            course.edges = vec![
                CourseEdge {
                    edge_id: "left".into(),
                    wall_id: "wall".into(),
                    start: point(0, 0),
                    end: point(200_000, 0),
                },
                CourseEdge {
                    edge_id: "right".into(),
                    wall_id: "wall".into(),
                    start: point(200_000, 0),
                    end: point(400_000, 0),
                },
            ];
            course.runs[0].sources = vec![
                RunSource {
                    wall_id: "wall".into(),
                    edge_id: "left".into(),
                    start: point(0, 0),
                    end: point(200_000, 0),
                    start_offset: 0,
                    end_offset: 200_000,
                },
                RunSource {
                    wall_id: "wall".into(),
                    edge_id: "right".into(),
                    start: point(200_000, 0),
                    end: point(400_000, 0),
                    start_offset: 200_000,
                    end_offset: 400_000,
                },
            ];
        }
        let mut opening = opening();
        opening.start = raw(1500.0, 0.0);
        opening.end = raw(1950.0, 0.0);
        let crossing_beam = beam(raw(2000.0, -200.0), raw(2000.0, 200.0), 300.0);
        let result =
            build_constraints(&topology, &[opening], &[crossing_beam], 200.0, 300.0).unwrap();
        assert_eq!(result.lintel_candidates.len(), 2);
        assert_eq!(
            result.lintel_candidates[0].span,
            Interval {
                start: 150_000,
                end: 195_000
            }
        );
        assert_eq!(
            result.lintel_candidates[0].right_support,
            Some(Interval {
                start: 195_000,
                end: 225_000
            })
        );
        assert_eq!(
            result
                .masks
                .iter()
                .filter(|mask| mask.course_index == 0)
                .count(),
            3
        );
        let beam_masks: Vec<_> = result
            .masks
            .iter()
            .filter(|mask| mask.course_index == 0 && mask.source == MaskSource::Beam("beam".into()))
            .collect();
        assert_eq!(beam_masks.iter().map(|mask| mask.interval).collect::<Vec<_>>(),
            vec![Interval {start:185_000,end:200_000}, Interval {start:200_000,end:215_000}]);
        assert!(beam_masks.iter().all(|mask| mask.coverage == MaskCoverage::FullSectionVoid));
        assert!(result.masks.iter().all(|mask| mask.run_id == "run"));
        let longitudinal = beam(raw(1500.0, 0.0), raw(2500.0, 0.0), 200.0);
        let expanded = build_constraints_with_policy(
            &topology,
            &[],
            &[longitudinal],
            200.0,
            300.0,
            BeamCutPolicy {
                longitudinal_full_wall_thickness: true,
                ..BeamCutPolicy::default()
            },
        )
        .unwrap();
        let masks: Vec<_> = expanded
            .masks
            .iter()
            .filter(|m| m.course_index == 0)
            .collect();
        assert_eq!(masks.len(), 2);
        assert!(masks
            .iter()
            .all(|m| m.coverage == MaskCoverage::FullSectionVoid));
        assert_eq!(
            masks[0].interval,
            Interval {
                start: 150_000,
                end: 200_000
            }
        );
        assert_eq!(
            masks[1].interval,
            Interval {
                start: 200_000,
                end: 250_000
            }
        );
    }

    #[test]
    fn opening_through_collinear_runs_preserves_both_volume_masks() {
        let mut topology = topology();
        for course in &mut topology.courses {
            course.runs = vec![
                WallRun {
                    id: "left".into(),
                    start: point(0, 0),
                    end: point(200_000, 0),
                    length: 200_000,
                    sources: vec![RunSource {
                        wall_id: "wall".into(),
                        edge_id: "left".into(),
                        start: point(0, 0),
                        end: point(200_000, 0),
                        start_offset: 0,
                        end_offset: 200_000,
                    }],
                },
                WallRun {
                    id: "right".into(),
                    start: point(200_000, 0),
                    end: point(400_000, 0),
                    length: 200_000,
                    sources: vec![RunSource {
                        wall_id: "wall".into(),
                        edge_id: "right".into(),
                        start: point(200_000, 0),
                        end: point(400_000, 0),
                        start_offset: 0,
                        end_offset: 200_000,
                    }],
                },
            ];
        }
        let mut opening = opening();
        opening.start = raw(1500.0, 0.0);
        opening.end = raw(2500.0, 0.0);
        let result = build_constraints(&topology, &[opening], &[], 200.0, 300.0).unwrap();
        assert!(result.masks.iter().any(|m| m.run_id == "left"));
        assert!(result.masks.iter().any(|m| m.run_id == "right"));
    }

    #[test]
    fn opening_width_selects_three_or_five_lintel_rows() {
        let mut t = topology();
        let template = t.courses[0].clone();
        t.courses = (0..12)
            .map(|index| {
                let mut course = template.clone();
                course.index = index;
                course.z = index * 20_000;
                course
            })
            .collect();
        let narrow = build_constraints(&t, &[opening()], &[], 200.0, 200.0).unwrap();
        assert_eq!(
            narrow
                .lintel_candidates
                .iter()
                .map(|l| l.course_index)
                .collect::<Vec<_>>(),
            vec![1, 3, 5]
        );
        let mut wide = opening();
        wide.end = raw(2500.0, 0.0);
        let wide = build_constraints(&t, &[wide], &[], 200.0, 200.0).unwrap();
        assert_eq!(
            wide.lintel_candidates
                .iter()
                .map(|l| l.course_index)
                .collect::<Vec<_>>(),
            vec![1, 3, 5, 7, 9]
        );
        for available in [4, 5, 8, 9] {
            let mut short = t.clone();
            short.courses.truncate(available + 1);
            for (end, requested) in [(2000., vec![1, 3, 5]), (2500., vec![1, 3, 5, 7, 9])] {
                let mut source = opening();
                source.end = raw(end, 0.);
                let actual = build_constraints(&short, &[source], &[], 200., 200.).unwrap();
                let expected: Vec<_> = requested.into_iter()
                    .filter(|index| *index <= available as i64).collect();
                assert_eq!(actual.lintel_candidates.iter().map(|c| c.course_index)
                    .collect::<Vec<_>>(), expected, "available={available}, end={end}");
            }
        }
    }

    #[test]
    fn beam_160_in_wall_193_is_partial_and_touch_is_not_collision() {
        let mut topology = topology();
        topology.walls[0].thickness = 19_300;
        let narrow = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 160.0);
        let result = build_constraints(&topology, &[], &[narrow], 200.0, 300.0).unwrap();
        let mask = result
            .masks
            .iter()
            .find(|mask| mask.course_index == 0)
            .unwrap();
        assert_eq!(mask.coverage, MaskCoverage::FullSectionVoid);
        let whole = SolidBox {
            u: Interval {
                start: 120_000,
                end: 130_000,
            },
            v: Interval {
                start: -9_650,
                end: 9_650,
            },
            z: Interval {
                start: 0,
                end: 20_000,
            },
        };
        assert!(matches!(
            result.classify_box("run", 0, whole).unwrap(),
            Exclusion::Partial { .. }
        ));
        let interior = SolidBox {
            v: Interval {
                start: -5_000,
                end: 5_000,
            },
            ..whole
        };
        assert!(matches!(
            result.classify_box("run", 0, interior).unwrap(),
            Exclusion::FullVoid { .. }
        ));
        let touching = SolidBox {
            u: Interval {
                start: 200_000,
                end: 210_000,
            },
            ..whole
        };
        assert_eq!(
            result.classify_box("run", 0, touching).unwrap(),
            Exclusion::None
        );
        let cuts = result.beam_box_cuts("run", 0, whole).unwrap();
        assert_eq!(
            cuts,
            vec![BeamBoxCut {
                beam_id: "beam".into(),
                volume: SolidBox {
                    v: Interval {
                        start: -8_000,
                        end: 8_000
                    },
                    ..whole
                },
            }]
        );
        assert!(result.beam_box_cuts("run", 0, touching).unwrap().is_empty());
        assert!(result.lintel_candidates.is_empty());
    }

    #[test]
    fn transverse_pocket_clears_wall_thickness_and_keeps_exact_height() {
        let mut topology = topology();
        topology.walls[0].thickness = 19_300;
        let mut transverse = beam(raw(1500.0, -800.0), raw(1500.0, 76.5), 160.0);
        transverse.bottom_start_mm = 30.0;
        transverse.bottom_end_mm = 30.0;
        transverse.top_start_mm = 150.0;
        transverse.top_end_mm = 150.0;
        transverse.height_mm = 120.0;
        let constraints = build_constraints(&topology, &[], &[transverse], 200.0, 300.0).unwrap();
        let whole = SolidBox {
            u: Interval {
                start: 140_000,
                end: 160_000,
            },
            v: Interval {
                start: -9_650,
                end: 9_650,
            },
            z: Interval {
                start: 0,
                end: 20_000,
            },
        };
        let cuts = constraints.beam_box_cuts("run", 0, whole).unwrap();
        assert_eq!(
            cuts,
            vec![BeamBoxCut {
                beam_id: "beam".into(),
                volume: SolidBox {
                    u: Interval {
                        start: 142_000,
                        end: 158_000
                    },
                    v: Interval {
                        start: -9_650,
                        end: 9_650
                    },
                    z: Interval {
                        start: 3_000,
                        end: 15_000
                    },
                },
            }]
        );
        assert_eq!(whole.v.end, cuts[0].volume.v.end);
        assert!(matches!(
            constraints
                .classify_box(
                    "run",
                    0,
                    SolidBox {
                        v: Interval {
                            start: 7_650,
                            end: 9_650
                        },
                        ..whole
                    }
                )
                .unwrap(),
            Exclusion::Partial { .. }
        ));
        assert_eq!(
            constraints
                .classify_box(
                    "run",
                    0,
                    SolidBox {
                        z: Interval {
                            start: 15_000,
                            end: 20_000
                        },
                        ..whole
                    }
                )
                .unwrap(),
            Exclusion::None
        );
        assert!(constraints.lintel_candidates.is_empty());
    }

    #[test]
    fn angled_beam_cut_is_diagnostic_instead_of_a_bounding_box() {
        let diagonal = beam(raw(1000.0, -80.0), raw(2000.0, 80.0), 160.0);
        let constraints = build_constraints(&topology(), &[], &[diagonal], 200.0, 300.0).unwrap();
        let solid = SolidBox {
            u: Interval {
                start: 100_000,
                end: 200_000,
            },
            v: Interval {
                start: -10_000,
                end: 10_000,
            },
            z: Interval {
                start: 0,
                end: 20_000,
            },
        };
        assert_eq!(
            constraints.beam_box_cuts("run", 0, solid).unwrap_err().kind,
            ConstraintErrorKind::UnsupportedBeamGeometry
        );
    }

    #[test]
    fn longitudinal_assembly_envelope_keeps_physical_z_extent() {
        let mut topology = topology();
        topology.walls[0].thickness = 19_300;
        let mut narrow = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 160.0);
        narrow.bottom_start_mm = 50.0;
        narrow.bottom_end_mm = 50.0;
        narrow.top_start_mm = 150.0;
        narrow.top_end_mm = 150.0;
        narrow.height_mm = 100.0;
        let result = build_constraints_with_policy(
            &topology,
            &[],
            &[narrow],
            200.0,
            300.0,
            BeamCutPolicy {
                longitudinal_full_wall_thickness: true,
                full_course_clearance: false,
                assembly_clearance_mm: 10.0,
                ..BeamCutPolicy::default()
            },
        )
        .unwrap();
        let mask = result
            .masks
            .iter()
            .find(|mask| mask.course_index == 0)
            .unwrap();
        assert_eq!(
            mask.interval,
            Interval {
                start: 100_000,
                end: 200_000
            }
        );
        assert_eq!(mask.coverage, MaskCoverage::FullSectionVoid);
        let inside = SolidBox {
            u: Interval {
                start: 120_000,
                end: 130_000,
            },
            v: Interval {
                start: -9_650,
                end: 9_650,
            },
            z: Interval {
                start: 6_000,
                end: 14_000,
            },
        };
        assert!(matches!(
            result.classify_box("run", 0, inside).unwrap(),
            Exclusion::Partial { .. }
        ));
        assert!(matches!(
            result.classify_assembly_box("run", 0, inside).unwrap(),
            Exclusion::FullVoid { .. }
        ));
        let added = SolidBox {
            u: Interval {
                start: 100_100,
                end: 100_900,
            },
            z: Interval {
                start: 4_100,
                end: 4_900,
            },
            ..inside
        };
        assert_eq!(
            result.classify_box("run", 0, added).unwrap(),
            Exclusion::None
        );
        assert!(matches!(
            result.classify_assembly_box("run", 0, added).unwrap(),
            Exclusion::FullVoid { .. }
        ));
        let outside = SolidBox {
            z: Interval {
                start: 0,
                end: 3_000,
            },
            ..inside
        };
        assert_eq!(
            result.classify_assembly_box("run", 0, outside).unwrap(),
            Exclusion::FullVoid { source_ids: vec!["beam:beam".into()] }
        );
    }

    #[test]
    fn nominal_beam_does_not_reserve_a_nonintersecting_next_course() {
        let mut near_top = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 160.0);
        near_top.bottom_start_mm = 95.0;
        near_top.bottom_end_mm = 95.0;
        near_top.top_start_mm = 195.0;
        near_top.top_end_mm = 195.0;
        near_top.height_mm = 100.0;
        let result = build_constraints_with_policy(
            &topology(),
            &[],
            &[near_top],
            200.0,
            300.0,
            BeamCutPolicy {
                longitudinal_full_wall_thickness: true,
                full_course_clearance: false,
                assembly_clearance_mm: 10.0,
                ..BeamCutPolicy::default()
            },
        )
        .unwrap();
        let above = SolidBox {
            u: Interval {
                start: 120_000,
                end: 130_000,
            },
            v: Interval {
                start: -10_000,
                end: 10_000,
            },
            z: Interval {
                start: 20_000,
                end: 20_400,
            },
        };
        assert_eq!(
            result.classify_box("run", 1, above).unwrap(),
            Exclusion::None
        );
        assert_eq!(result.classify_assembly_box("run", 1, above).unwrap(), Exclusion::None);
        assert!(!result.masks.iter().any(|mask| mask.course_index == 1));
    }

    #[test]
    fn full_course_clearance_reserves_partial_height_beam_and_removes_support() {
        let mut partial = beam(raw(1200.0, 0.0), raw(1450.0, 0.0), 160.0);
        partial.bottom_start_mm = 200.0;
        partial.bottom_end_mm = 200.0;
        partial.top_start_mm = 250.0;
        partial.top_end_mm = 250.0;
        partial.height_mm = 50.0;
        let mut opening = opening();
        opening.start = raw(1400.0, 0.0);
        opening.end = raw(1600.0, 0.0);
        let result = build_constraints_with_policy(
            &topology(),
            &[opening],
            &[partial],
            200.0,
            300.0,
            BeamCutPolicy {
                longitudinal_full_wall_thickness: false,
                full_course_clearance: true,
                assembly_clearance_mm: 0.0,
                ..BeamCutPolicy::default()
            },
        )
        .unwrap();
        assert!(result.masks.iter().any(|mask| mask.course_index == 1
            && mask.source == MaskSource::Beam("beam".into())
            && mask.coverage == MaskCoverage::FullSectionVoid));
        assert!(!result
            .masks
            .iter()
            .any(|mask| mask.course_index == 0 && mask.source == MaskSource::Beam("beam".into())));
        assert_eq!(result.lintel_candidates[0].left_support, None);
        assert!(result.lintel_candidates[0].right_support.is_some());
    }

    #[test]
    fn sloped_beam_is_ignored_even_with_full_course_clearance() {
        let mut sloped = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 160.0);
        sloped.bottom_end_mm = 400.0;
        sloped.height_direction_x = -2.0 / 29.0_f64.sqrt();
        sloped.height_direction_z = 5.0 / 29.0_f64.sqrt();
        sloped.height_mm = 100.0;
        sloped.top_start_mm = 100.0 * sloped.height_direction_z;
        sloped.top_end_mm = 400.0 + sloped.top_start_mm;
        let result = build_constraints_with_policy(
            &topology(),
            &[],
            &[sloped],
            200.0,
            300.0,
            BeamCutPolicy {
                full_course_clearance: true,
                assembly_clearance_mm: 5.0,
                ..BeamCutPolicy::default()
            },
        )
        .unwrap();
        assert!(result.masks.is_empty());
        assert!(result.evidence.is_empty());
        assert!(result.beam_classifications.is_empty());
    }

    #[test]
    fn transverse_diagonal_uses_width_height_and_no_lintel_or_clearance() {
        let mut crossing = beam(raw(1200.0, -600.0), raw(1800.0, 600.0), 160.0);
        crossing.bottom_start_mm = 50.0;
        crossing.bottom_end_mm = 50.0;
        crossing.top_start_mm = 150.0;
        crossing.top_end_mm = 150.0;
        crossing.height_mm = 100.0;
        let constraints = build_constraints_with_policy(
            &topology(),
            &[],
            &[crossing],
            200.0,
            300.0,
            BeamCutPolicy {
                full_course_clearance: true,
                assembly_clearance_mm: 5.0,
                ..BeamCutPolicy::default()
            },
        )
        .unwrap();
        let solid = SolidBox {
            u: Interval {
                start: 100_000,
                end: 200_000,
            },
            v: Interval {
                start: -10_000,
                end: 10_000,
            },
            z: Interval {
                start: 0,
                end: 20_000,
            },
        };
        assert_eq!(
            constraints.beam_box_cuts("run", 0, solid).unwrap(),
            vec![BeamBoxCut {
                beam_id: "beam".into(),
                volume: SolidBox {
                    u: Interval {
                        start: 142_000,
                        end: 158_000
                    },
                    v: solid.v,
                    z: Interval {
                        start: 5_000,
                        end: 15_000
                    },
                },
            }]
        );
        assert!(constraints.lintel_candidates.is_empty());
        assert!(constraints.masks.iter().all(|mask| mask.course_index == 0));
        let above = SolidBox {
            u: Interval { start: 142_000, end: 158_000 },
            z: Interval {
                start: 15_000,
                end: 20_000,
            },
            ..solid
        };
        assert_eq!(
            constraints.classify_assembly_box("run", 0, above).unwrap(),
            Exclusion::FullVoid { source_ids: vec!["beam:beam".into()] }
        );
        let beside = SolidBox {
            u: Interval {
                start: 158_000,
                end: 158_500,
            },
            ..solid
        };
        assert_eq!(
            constraints.classify_box("run", 0, beside).unwrap(),
            Exclusion::None
        );
    }

    #[test]
    fn mirrored_vertical_height_direction_keeps_the_same_physical_beam_volume() {
        for (start, end) in [
            (raw(1000.0, 0.0), raw(2000.0, 0.0)),
            (raw(1500.0, -400.0), raw(1500.0, 400.0)),
        ] {
            let positive = beam(start, end, 160.0);
            let negative = RawBeam {
                bottom_start_mm: 200.0,
                bottom_end_mm: 200.0,
                top_start_mm: 0.0,
                top_end_mm: 0.0,
                height_direction_z: -1.0,
                ..positive.clone()
            };
            let upward = build_constraints(&topology(), &[], &[positive], 200.0, 300.0).unwrap();
            let downward = build_constraints(&topology(), &[], &[negative], 200.0, 300.0).unwrap();
            assert_eq!(upward.masks, downward.masks);
            assert_eq!(upward.beam_classifications, downward.beam_classifications);
            let solid = SolidBox {
                u: Interval {
                    start: 100_000,
                    end: 200_000,
                },
                v: Interval {
                    start: -10_000,
                    end: 10_000,
                },
                z: Interval {
                    start: 0,
                    end: 20_000,
                },
            };
            let expected = upward.beam_box_cuts("run", 0, solid).unwrap();
            assert!(!expected.is_empty());
            assert_eq!(expected, downward.beam_box_cuts("run", 0, solid).unwrap());
            assert_eq!(
                upward.classify_box("run", 0, solid).unwrap(),
                downward.classify_box("run", 0, solid).unwrap()
            );
        }
    }

    #[test]
    fn horizontal_beam_with_rolled_height_is_ignored() {
        let mut rolled = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 160.0);
        rolled.height_direction_y = 0.6;
        rolled.height_direction_z = 0.8;
        rolled.top_start_mm = 160.0;
        rolled.top_end_mm = 160.0;
        let result = build_constraints(&topology(), &[], &[rolled], 200.0, 300.0).unwrap();
        assert!(result.masks.is_empty());
        assert!(result.evidence.is_empty());
    }

    #[test]
    fn real_b09_and_reverse_b27_keep_nominal_body_and_physical_endpoints() {
        for (y, start, end, expected) in [
            (
                9920.0,
                -896.5,
                3205.0,
                Interval {
                    start: 0,
                    end: 320_000,
                },
            ),
            (
                320.0,
                12485.0,
                10235.0,
                Interval {
                    start: 1_024_000,
                    end: 1_248_000,
                },
            ),
        ] {
            let mut topology = topology();
            topology.walls[0].start.y = (y * SCALE) as i64;
            topology.walls[0].end = point(1_600_000, (y * SCALE) as i64);
            for course in &mut topology.courses {
                let run = &mut course.runs[0];
                run.start = topology.walls[0].start;
                run.end = topology.walls[0].end;
                run.length = 1_600_000;
                run.sources[0].start = run.start;
                run.sources[0].end = run.end;
                run.sources[0].end_offset = run.length;
            }
            let source = beam(raw(start, y), raw(end, y), 160.0);
            let policy = BeamCutPolicy {
                longitudinal_full_wall_thickness: true,
                affine_world_joint_spike_compensation: true,
                ..BeamCutPolicy::default()
            };
            let result = build_constraints_with_policy(
                &topology,
                &[],
                &[source.clone()],
                200.0,
                300.0,
                policy,
            )
            .unwrap();
            assert_eq!(result.masks[0].interval, expected);
            let projection = result.evidence[0].beam_projection.unwrap();
            assert_eq!(
                projection.physical_u,
                (start.min(end).max(0.0) * SCALE, start.max(end) * SCALE)
            );
            assert_eq!(
                projection.assembly_u_before_grid,
                (expected.start as f64, expected.end as f64)
            );
            let adjacent = SolidBox {
                u: Interval {
                    start: expected.end,
                    end: expected.end + 32_000,
                },
                v: Interval {
                    start: -10_000,
                    end: 10_000,
                },
                z: Interval {
                    start: 0,
                    end: 20_000,
                },
            };
            assert_eq!(
                result.classify_assembly_box("run", 0, adjacent).unwrap(),
                Exclusion::None
            );
            let without_policy = build_constraints_with_policy(
                &topology,
                &[],
                &[source],
                200.0,
                300.0,
                BeamCutPolicy {
                    affine_world_joint_spike_compensation: false,
                    ..policy
                },
            )
            .unwrap();
            assert_eq!(
                without_policy.masks[0].interval.end,
                (start.max(end) * SCALE) as i64
            );
        }
    }

    #[test]
    fn compensated_endpoint_guard_preserves_off_grid_and_wall_clip_boundaries() {
        let run = WallRun {
            id: "run".into(),
            start: point(0, 992_000),
            end: point(1_600_000, 992_000),
            length: 1_600_000,
            sources: vec![],
        };
        let frame = Frame::new(&run, 19_300).unwrap();
        for end in [3205.01, 3204.99, 3200.0, 3206.0] {
            let source = beam(raw(-896.5, 9920.0), raw(end, 9920.0), 160.0);
            let physical = (0.0, end * SCALE);
            assert_eq!(
                nominal_beam_body_u(&source, &frame, &run, 0, physical),
                physical
            );
        }
        let source = beam(raw(-896.5, 9920.0), raw(4000.0, 9920.0), 160.0);
        let clipped = (0.0, 320_500.0);
        assert_eq!(
            nominal_beam_body_u(&source, &frame, &run, 0, clipped),
            clipped
        );
        let off_axis = beam(raw(-896.5, 9920.01), raw(3205.0, 9920.01), 160.0);
        assert_eq!(
            nominal_beam_body_u(&off_axis, &frame, &run, 0, clipped),
            clipped
        );
    }

    #[test]
    fn longitudinal_clearance_does_not_shorten_adjacent_body() {
        for full_course_clearance in [false, true] {
            let result = build_constraints_with_policy(
                &topology(),
                &[],
                &[beam(raw(1280.0, 0.0), raw(1920.0, 0.0), 160.0)],
                200.0,
                300.0,
                BeamCutPolicy {
                    longitudinal_full_wall_thickness: true,
                    full_course_clearance,
                    assembly_clearance_mm: 5.0,
                    ..BeamCutPolicy::default()
                },
            )
            .unwrap();
            assert_eq!(
                result.masks[0].interval,
                Interval {
                    start: 128_000,
                    end: 192_000
                }
            );
            let adjacent = SolidBox {
                u: Interval {
                    start: 96_000,
                    end: 128_000,
                },
                v: Interval {
                    start: -10_000,
                    end: 10_000,
                },
                z: Interval {
                    start: 0,
                    end: 20_000,
                },
            };
            assert_eq!(
                result.classify_assembly_box("run", 0, adjacent).unwrap(),
                Exclusion::None
            );
        }
    }

    #[test]
    fn full_course_grid_rounds_both_fractional_bounds_once() {
        let policy = BeamCutPolicy {
            longitudinal_full_wall_thickness: false,
            full_course_clearance: true,
            assembly_clearance_mm: 0.0,
            ..BeamCutPolicy::default()
        };
        let positive = beam(raw(1000.001, 0.0), raw(2000.001, 0.0), 160.0);
        let result =
            build_constraints_with_policy(&topology(), &[], &[positive], 200.0, 300.0, policy)
                .unwrap();
        let mask = result
            .masks
            .iter()
            .find(|mask| mask.course_index == 0)
            .unwrap();
        assert_eq!(
            mask.interval,
            Interval {
                start: 100_000,
                end: 200_001
            }
        );
        assert_eq!(mask.coverage, MaskCoverage::FullSectionVoid);
        let provenance = result
            .evidence
            .iter()
            .find(|item| item.course_index == 0)
            .unwrap()
            .beam_projection
            .unwrap();
        assert!(provenance.assembly_u_before_grid.0 > mask.interval.start as f64);
        assert!(provenance.assembly_u_before_grid.1 < mask.interval.end as f64);
        let fringe = SolidBox {
            u: Interval {
                start: 100_000,
                end: 100_001,
            },
            v: Interval {
                start: -10_000,
                end: 10_000,
            },
            z: Interval {
                start: 0,
                end: 20_000,
            },
        };
        assert!(matches!(
            result.classify_assembly_box("run", 0, fringe).unwrap(),
            Exclusion::FullVoid { .. }
        ));
        assert!(matches!(
            result.classify_box("run", 0, fringe).unwrap(),
            Exclusion::Partial { .. }
        ));

        let mut negative_topology = topology();
        for wall in &mut negative_topology.walls {
            wall.start.x -= 400_000;
            wall.end.x -= 400_000;
        }
        for course in &mut negative_topology.courses {
            for edge in &mut course.edges {
                edge.start.x -= 400_000;
                edge.end.x -= 400_000;
            }
            for run in &mut course.runs {
                run.start.x -= 400_000;
                run.end.x -= 400_000;
                for source in &mut run.sources {
                    source.start.x -= 400_000;
                    source.end.x -= 400_000;
                }
            }
        }
        let negative = beam(raw(-2000.001, 0.0), raw(-1000.001, 0.0), 160.0);
        let result = build_constraints_with_policy(
            &negative_topology,
            &[],
            &[negative],
            200.0,
            300.0,
            policy,
        )
        .unwrap();
        let mask = result
            .masks
            .iter()
            .find(|mask| mask.course_index == 0)
            .unwrap();
        assert_eq!(
            mask.interval,
            Interval {
                start: 199_999,
                end: 300_000
            }
        );
        assert_eq!(mask.coverage, MaskCoverage::FullSectionVoid);

        let exact = beam(raw(1000.0, 0.0), raw(2000.0, 0.0), 160.0);
        let result =
            build_constraints_with_policy(&topology(), &[], &[exact], 200.0, 300.0, policy)
                .unwrap();
        let mask = result
            .masks
            .iter()
            .find(|mask| mask.course_index == 0)
            .unwrap();
        assert_eq!(
            mask.interval,
            Interval {
                start: 100_000,
                end: 200_000
            }
        );
    }
}

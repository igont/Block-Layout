//! Проверенная геометрия здания. Целые координаты измеряются в 0,01 мм.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawPoint {
    pub x_mm: f64,
    pub y_mm: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RawWall {
    pub id: String,
    pub start: RawPoint,
    pub end: RawPoint,
    pub bottom_start_mm: f64,
    pub top_start_mm: f64,
    pub bottom_end_mm: f64,
    pub top_end_mm: f64,
    pub thickness_mm: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RawBuilding {
    pub z0_mm: f64,
    pub walls: Vec<RawWall>,
}

#[derive(
    Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, serde::Serialize, serde::Deserialize,
)]
pub struct Point {
    pub x: i64,
    pub y: i64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NormalizedWall {
    pub id: String,
    pub start: Point,
    pub end: Point,
    pub bottom_start: i64,
    pub top_start: i64,
    pub bottom_end: i64,
    pub top_end: i64,
    pub thickness: i64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NormalizedBuilding {
    pub z0: i64,
    pub walls: Vec<NormalizedWall>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Vertex {
    pub id: String,
    pub point: Point,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Edge {
    pub id: String,
    pub wall_id: String,
    pub start_vertex_id: String,
    pub end_vertex_id: String,
    pub start: Point,
    pub end: Point,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CourseEdge {
    pub edge_id: String,
    pub wall_id: String,
    pub start: Point,
    pub end: Point,
}

/// Исходный активный участок и его интервал на канонической оси WallRun.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RunSource {
    pub wall_id: String,
    pub edge_id: String,
    pub start: Point,
    pub end: Point,
    pub start_offset: i64,
    pub end_offset: i64,
}

/// Максимальный непрерывный прямой участок текущего венца.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WallRun {
    pub id: String,
    pub start: Point,
    pub end: Point,
    pub length: i64,
    pub sources: Vec<RunSource>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ray {
    pub edge_id: String,
    pub direction_deg: u16,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CourseVertex {
    pub id: String,
    pub point: Point,
    pub rays: Vec<Ray>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Course {
    pub index: i64,
    pub z: i64,
    pub edges: Vec<CourseEdge>,
    pub runs: Vec<WallRun>,
    pub vertices: Vec<CourseVertex>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerticalLink {
    pub lower_index: i64,
    pub upper_index: i64,
    pub wall_id: String,
    pub start: Point,
    pub end: Point,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Topology {
    pub z0: i64,
    pub walls: Vec<NormalizedWall>,
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Edge>,
    pub courses: Vec<Course>,
    pub vertical_links: Vec<VerticalLink>,
}

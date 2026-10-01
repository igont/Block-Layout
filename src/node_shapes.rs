//! Геометрия ортогонального каталога: TestAddon/GDL/Блок FB.xml, Script_3D.
//! Упрощённое тело без шипов и верхних технологических отверстий; размеры в мм.
use crate::layout::Block;
use crate::solid_geometry::{box_mesh, subtract, Mesh, Plane};

type P = [f64; 2];
const AXES: [[f64; 3]; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];

/// Предупреждение о доказанном пересечении исходных профилей T1.
/// Постоянный текст позволяет агрегировать предупреждения по использованным узлам.
pub fn node_profile_warning(block: &Block) -> Option<crate::api::ApiFailure> {
    if block.product_key.as_deref() != Some("Type10_1") {
        return None;
    }
    let mut warning = crate::api::ApiFailure::new(
        "CATALOG_NODE_PROFILE_OVERLAP",
        "Исходные GDL-профили T1 Type8/8.1 и Type10.1 пересекаются: 14.593680 мм², 919.401846 мм³ на каждую сопрягаемую пару при высоте 63 мм; осевой зазор 0.296027 мм. Геометрия сохранена, согласование профиля требуется",
        Some(block.id.clone()),
    );
    warning.source_ids = block.source_ids.clone();
    warning.course_index = Some(block.course_index);
    Some(warning)
}

/// Переносит координаты производственных врезок на ось полученного фрагмента.
pub(crate) fn relocate_node_cuts(
    block: &mut Block,
    from_centimm: i64,
    to_centimm: i64,
) -> Result<(), crate::api::ApiFailure> {
    let issue = |message: &str| crate::api::ApiFailure::new("INVALID_PRODUCT_CODE", message, Some(block.id.clone()));
    let mut relocated = Vec::new();
    for cut in &block.cuts {
        if !cut.starts_with("Type") {
            relocated.push(cut.clone());
            continue;
        }
        let mut fields = cut.splitn(4, ':');
        let kind = fields.next().unwrap_or("");
        let location = fields.next().unwrap_or("");
        let side = fields.next().ok_or_else(|| issue("Отсутствует сторона врезки"))?;
        let sources = fields.next().ok_or_else(|| issue("Отсутствуют источники врезки"))?;
        let position = if let Some(x) = location.strip_prefix('x') {
            x.parse::<i64>().ok().filter(|x| *x > 0 && *x <= 3126)
                .map(|x| (x - 1) * 32_000)
        } else {
            location.strip_prefix('p').and_then(|p| p.parse::<i64>().ok())
        }.filter(|p| *p >= 0 && *p <= block.length_centimm)
            .ok_or_else(|| issue("Некорректная координата врезки"))?;
        if position >= from_centimm && position <= to_centimm {
            relocated.push(format!("{kind}:p{}:{side}:{sources}", position - from_centimm));
        } else {
            // Торцевые профили также могут оставить материал обработки внутри
            // фрагмента, даже когда сама координата узла уже удалена.
            let reach = if matches!(kind, "Type6" | "Type7_1") { 9_650 } else { 9_705 };
            if position.saturating_add(reach) <= from_centimm || position.saturating_sub(reach) >= to_centimm {
                continue;
            }
            return Err(crate::api::ApiFailure::new("NODE_CUT_CROSSES_SPLIT",
                "Разрез оставляет часть производственной врезки за границей её координаты",
                Some(block.id.clone())));
        }
    }
    block.cuts = relocated;
    Ok(())
}

#[derive(Clone, Copy)]
struct Frame {
    origin: P,
    x: P,
    y: P,
}
impl Frame {
    fn new() -> Self {
        Self {
            origin: [0., -96.5],
            x: [1., 0.],
            y: [0., 1.],
        }
    }
    fn point(self, p: P) -> P {
        [
            self.origin[0] + p[0] * self.x[0] + p[1] * self.y[0],
            self.origin[1] + p[0] * self.x[1] + p[1] * self.y[1],
        ]
    }
    fn shift(&mut self, x: f64, y: f64) {
        self.origin = self.point([x, y]);
    }
    fn scale(&mut self, x: f64, y: f64) {
        self.x = [self.x[0] * x, self.x[1] * x];
        self.y = [self.y[0] * y, self.y[1] * y];
    }
    fn rotate(&mut self, degrees: f64) {
        let (s, c) = degrees.to_radians().sin_cos();
        let (x, y) = (self.x, self.y);
        self.x = [c * x[0] + s * y[0], c * x[1] + s * y[1]];
        self.y = [-s * x[0] + c * y[0], -s * x[1] + c * y[1]];
    }
}

struct Cut<'a> {
    name: &'a str,
    slot: usize,
    face: usize,
    position_mm: Option<f64>,
}
fn cuts(block: &Block, last: usize) -> Result<Vec<Cut<'_>>, String> {
    let mut result = Vec::new();
    for text in &block.cuts {
        let mut fields = text.split(':');
        let name = fields.next().unwrap_or("");
        if text.starts_with("arm:") && text.ends_with(":distal") {
            continue; // node_geometry.rs:230; реальные внешние вычеты делает materializer.
        }
        if !name.starts_with("Type") {
            return Err(format!("Неизвестный fixed-запил узла: {text}"));
        }
        let number = |field: Option<&str>, prefix| -> Result<usize, String> {
            field
                .and_then(|v| v.strip_prefix(prefix))
                .and_then(|v| v.parse().ok())
                .ok_or_else(|| format!("Некорректный каталожный запил: {text}"))
        };
        let location = fields.next().unwrap_or("");
        let position_mm = location.strip_prefix('p').map(|p| p.parse::<i64>()
            .map(|p| p as f64 / 100.).map_err(|_| format!("Некорректная координата: {text}"))).transpose()?;
        let slot = if let Some(position) = position_mm {
            if position < 0. || position > block.length_centimm as f64 / 100. {
                return Err(format!("Врезка за пределами детали: {text}"));
            }
            if position == 0. { 1 } else if position == block.length_centimm as f64 / 100. { last } else { 2 }
        } else { number(Some(location), 'x')? };
        let mut face = number(fields.next(), 'y')?;
        if slot == 0 || slot > last || !(1..=3).contains(&face) {
            return Err(format!("Запил вне слотов изделия: {text}"));
        }
        // BlockFbGdlAdapter.hpp:97: legacy y для Type8/8.1 меняется 1 <-> 3.
        if matches!(name, "Type8" | "Type8_1") {
            face = 4 - face;
        }
        if name == "Type7_1" {
            result.extend([
                Cut {
                    name: "Type6",
                    slot,
                    face: 1,
                    position_mm,
                },
                Cut {
                    name: "Type6",
                    slot,
                    face: 3,
                    position_mm,
                },
            ]);
        } else {
            result.push(Cut { name, slot, face, position_mm });
        }
    }
    Ok(result)
}

// Порядок преобразований совпадает с GDL: CreateBlock_EdgeTypeCuts_Open:506.
fn polygons(cut: &Cut<'_>, last: usize, length: f64) -> Result<Vec<Vec<P>>, String> {
    let (i, j) = (cut.slot, cut.face);
    let mut f = Frame::new();
    let internal_coordinate = cut.position_mm.is_some() && cut.name == "Type6";
    if j == 1 && (i > 1 || internal_coordinate) {
        f.shift(0., 193.);
        f.scale(1., -1.);
    }
    if i == last && j == 1 && !internal_coordinate {
        f.scale(-1., 1.);
        f.shift(-length, 0.);
    }
    if i == last && j == 3 && !internal_coordinate {
        f.rotate(180.);
        f.shift(-length, -193.);
    }
    let polygon: Vec<P> = match cut.name {
        "Type1" | "Type3" => {
            if cut.name == "Type3" {
                f.shift(0., 193.);
                f.scale(1., -1.);
            }
            f.shift(0., 96.5);
            vec![
                [-96.5, 96.5],
                [-25.6513, 30.1089],
                [-37.6786, -67.8462],
                [56.3063, -56.3063],
                [96.5, -96.5],
                [-196.5, -96.5],
                [-196.5, 96.5],
            ]
        }
        "Type2" | "Type4" => {
            if cut.name == "Type2" {
                f.shift(0., 193.);
                f.scale(1., -1.);
            }
            f.shift(0., 96.5);
            vec![
                [-96.5, 96.5],
                [-26.1387, 26.1387],
                [67.8462, 37.6786],
                [56.3063, -56.3063],
                [96.5, -96.5],
                [-196.5, -96.5],
                [-196.5, 96.5],
            ]
        }
        "Type5" | "Type5_1" => {
            if j != 2 || (i != 1 && i != last) {
                return Err("Type5.1 требует торцевой слот y2".into());
            }
            f.shift(0., 96.5);
            if i == last {
                f.rotate(180.);
                f.shift(-length, 0.);
            }
            vec![
                [96.5, -96.5],
                [74.8042, -74.8042],
                [97.0433, 0.],
                [74.8042, 74.8042],
                [96.5, 96.5],
                [-1000., 96.5],
                [-1000., -96.5],
            ]
        }
        "Type6" => {
            if (!internal_coordinate && (i == 1 || i == last)) || j == 2 {
                return Err("Type6 требует внутренний слот y1/y3".into());
            }
            // CBE_Type6:233; edgeType_depth=0 для фиксированного каталога.
            f.shift(cut.position_mm.unwrap_or((i as f64 - 1.) * 320.) - 320., 0.);
            f.rotate(90.);
            f.shift(96.5, -320.);
            vec![
                [96.5, -96.5],
                [73.1958, -73.1958],
                [95.4349, 0.],
                [73.1958, 73.1958],
                [96.5, 96.5],
                [1000., 96.5],
                [1000., -96.5],
            ]
        }
        "Type8" | "Type8_1" => {
            if cut.name == "Type8_1" {
                f.scale(1., -1.);
                f.shift(0., -193.);
            }
            vec![
                [-500., 0.],
                [-500., 84.4517],
                [5., 84.4517],
                [83.5172, 95.1001],
                [74.268, 22.24],
                [96.5, 0.],
            ]
        }
        "Type10_1" => {
            if j != 2 || (i != 1 && i != last) {
                return Err("Type10.1 требует торцевой слот y2".into());
            }
            // CBE_Type10_1:305 и _CBE_Type10_1:321, оба зеркальных прохода.
            if i == last {
                f.rotate(180.);
                f.shift(-length * 2., -193.);
            }
            f.shift((i as f64 - 2.) * 320., 0.);
            f.rotate(90.);
            f.shift(96.5, -416.5);
            let p = vec![
                [500., 0.],
                [500., 500.],
                [0., 500. - 5. * (5. / 500.)],
                [0., 84.4517 - 5. * (5. / 84.4517)],
                [83.5172, 95.1001],
                [74.268, 22.24],
                [96.5, 0.],
            ];
            let a = p.iter().map(|&p| f.point(p)).collect();
            f.scale(-1., 1.);
            return Ok(vec![a, p.into_iter().map(|p| f.point(p)).collect()]);
        }
        _ => return Err(format!("Геометрия запила {} не поддержана", cut.name)),
    };
    Ok(vec![polygon.into_iter().map(|p| f.point(p)).collect()])
}

fn cross(a: P, b: P, c: P) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

// Ушная триангуляция простого контура: каждая призма вычитается через три плоскости.
fn triangles(mut polygon: Vec<P>) -> Result<Vec<[P; 3]>, String> {
    let area: f64 = (0..polygon.len())
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    if area < 0. {
        polygon.reverse();
    }
    let mut result = Vec::new();
    while polygon.len() > 3 {
        let ear = (0..polygon.len())
            .find(|&i| {
                let n = polygon.len();
                let (a, b, c) = (polygon[(i + n - 1) % n], polygon[i], polygon[(i + 1) % n]);
                cross(a, b, c) > 1e-8
                    && !polygon.iter().enumerate().any(|(k, &p)| {
                        k != (i + n - 1) % n
                            && k != i
                            && k != (i + 1) % n
                            && cross(a, b, p) >= -1e-8
                            && cross(b, c, p) >= -1e-8
                            && cross(c, a, p) >= -1e-8
                    })
            })
            .ok_or_else(|| "Не удалось триангулировать каталог запила".to_owned())?;
        let n = polygon.len();
        result.push([
            polygon[(ear + n - 1) % n],
            polygon[ear],
            polygon[(ear + 1) % n],
        ]);
        polygon.remove(ear);
    }
    result.push([polygon[0], polygon[1], polygon[2]]);
    Ok(result)
}

/// Материализует fixed-node каталог в WORLD мм, сохраняя номинальную ось изделия.
pub fn node_stock(block: &Block, width_mm: f64, height_mm: f64) -> Result<Vec<Mesh>, String> {
    if !width_mm.is_finite()
        || !height_mm.is_finite()
        || (width_mm - 193.).abs() > 1e-7
        || (height_mm - 63.).abs() > 1e-7
    {
        return Err("Каталог узлов поддерживает сечение 193x63 мм".into());
    }
    let product = block
        .product_key
        .as_deref()
        .or_else(|| block.cuts.iter().find(|cut| cut.starts_with("Type"))
            .and_then(|cut| cut.split(':').next()))
        .ok_or("У узла нет product_key")?;
    if !matches!(
        product,
        "Type1"
            | "Type2"
            | "Type3"
            | "Type4"
            | "Type5"
            | "Type5_1"
            | "Type6"
            | "Type7_1"
            | "Type8"
            | "Type8_1"
            | "Type10_1"
    ) {
        return Err(format!("Геометрия изделия {product} не поддержана"));
    }
    let length = block.length_centimm as f64 / 100.;
    if !length.is_finite() || length <= 0. {
        return Err("Длина узла должна быть положительной".into());
    }
    let last = ((length / 320.).round() as usize + 1).max(2);
    let cuts = cuts(block, last)?;
    let is_l = |c: &Cut<'_>| matches!(c.name, "Type1" | "Type2" | "Type3" | "Type4");
    let begin = if cuts.iter().any(|c| c.slot == 1 && is_l(c)) {
        96.5
    } else {
        0.
    };
    let end = if cuts.iter().any(|c| c.slot == last && is_l(c)) {
        96.5
    } else {
        0.
    };
    // XML CreateBlock:620: simplified lengthConstr-10 для L, иначе nominal.
    let mut meshes = vec![box_mesh(
        [-begin, -96.5, 0.],
        AXES,
        [length + begin + end, 193., 63.],
    )];
    for cut in &cuts {
        for polygon in polygons(cut, last, length)? {
            for tri in triangles(polygon)? {
                let planes: Vec<_> = (0..3)
                    .map(|i| {
                        let (a, b) = (tri[i], tri[(i + 1) % 3]);
                        let normal = [b[1] - a[1], a[0] - b[0], 0.];
                        Plane {
                            normal,
                            offset: normal[0] * a[0] + normal[1] * a[1],
                        }
                    })
                    .collect();
                meshes = meshes.iter().flat_map(|m| subtract(m, &planes)).collect();
            }
        }
    }
    if meshes.is_empty() {
        return Err("Каталожные запилы удалили всё изделие".into());
    }
    let (s, c) = (block.rotation_deg as f64).to_radians().sin_cos();
    for mesh in &mut meshes {
        for p in &mut mesh.vertices {
            let (x, y) = (p[0], p[1]);
            p[0] = block.start.x as f64 / 100. + c * x - s * y;
            p[1] = block.start.y as f64 / 100. + s * x + c * y;
            p[2] += block.z_centimm as f64 / 100.;
        }
    }
    Ok(meshes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid_geometry::{clip, volume};

    fn block(product: &str, length: i64, rotation: u16, origin: P, cuts: &[&str]) -> Block {
        serde_json::from_value(serde_json::json!({
            "id":"test","wall_id":"w","edge_id":"e","course_index":0,
            "z_centimm":0,"start":{"x":(origin[0]*100.) as i64,"y":(origin[1]*100.) as i64},
            "end":{"x":0,"y":0},"length_centimm":length*100,"kind":"node",
            "product_key":product,"catalog_status":"KnownPattern","rotation_deg":rotation,
            "is_bridge":false,"hide_spikes_left":false,"hide_spikes_right":false,
            "cuts":cuts,"source_ids":["w"]
        }))
        .unwrap()
    }

    fn intersection_volume(a: &Mesh, b: &Mesh) -> f64 {
        let mut result = Some(a.clone());
        for face in &b.faces {
            let (p, q, r) = (
                b.vertices[face[0]],
                b.vertices[face[1]],
                b.vertices[face[2]],
            );
            let u = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
            let v = [r[0] - p[0], r[1] - p[1], r[2] - p[2]];
            let n = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            result = result.and_then(|mesh| {
                clip(
                    &mesh,
                    &Plane {
                        normal: n,
                        offset: n[0] * p[0] + n[1] * p[1] + n[2] * p[2],
                    },
                )
            });
        }
        result.as_ref().map_or(0., volume)
    }

    #[test]
    fn literal_gdl_profiles_preserve_source_intersections_without_hiding_them() {
        let cases = [
            vec![
                block("Type2", 320, 0, [0., 0.], &["Type2:x1:y3:w"]),
                block("Type1", 640, 90, [0., 0.], &["Type1:x1:y1:w"]),
            ],
            vec![
                block("Type3", 640, 0, [0., 0.], &["Type3:x1:y3:w"]),
                block("Type4", 320, 90, [0., 0.], &["Type4:x1:y1:w"]),
            ],
            vec![
                block("Type6", 640, 180, [320., 0.], &["Type6:x2:y1:w"]),
                block("Type5_1", 640, 90, [0., 0.], &["Type5_1:x1:y2:w"]),
            ],
            vec![
                block("Type8_1", 640, 0, [0., 0.], &["Type8_1:x1:y1:w"]),
                block("Type10_1", 320, 90, [0., 0.], &["Type10_1:x1:y2:w"]),
                block("Type8", 640, 180, [0., 0.], &["Type8:x1:y3:w"]),
            ],
            vec![
                block(
                    "Type7_1",
                    640,
                    180,
                    [320., 0.],
                    &["Type6:x2:y1:w", "Type6:x2:y3:w"],
                ),
                block("Type5_1", 640, 90, [0., 0.], &["Type5_1:x1:y2:w"]),
                block("Type5_1", 640, 270, [0., 0.], &["Type5_1:x1:y2:w"]),
            ],
        ];
        for (case, blocks) in cases.iter().enumerate() {
            let parts: Vec<_> = blocks
                .iter()
                .map(|b| node_stock(b, 193., 63.).unwrap())
                .collect();
            for (i, a) in parts.iter().enumerate() {
                assert!(a.iter().map(volume).sum::<f64>() > 0.);
                for (j, b) in parts.iter().enumerate().skip(i + 1) {
                    let overlap: f64 = a
                        .iter()
                        .flat_map(|a| b.iter().map(move |b| intersection_volume(a, b)))
                        .sum();
                    // Это измеренные недостатки исходного GDL, а не допуск приёмки.
                    // Независимое 2D-вычитание исходных контуров Shapely даёт эти объёмы.
                    // Type8/10.1 исходного GDL имеют разные переходы (5,84.4517)
                    // и (0,84.155675): известный каталог даёт 919.402 мм3 на пару.
                    let expected = match case {
                        1 => 0.22578934941867534,
                        3 if j == i + 1 => 919.401845966571,
                        _ => 0.,
                    };
                    assert!(
                        (overlap - expected).abs() < 1e-6,
                        "case {case}, pair {i}/{j}: overlap {overlap} mm3, GDL {expected}"
                    );
                }
            }
        }
    }
    // Пересечение луча (x=0,z=31.5) с полупространствами выпуклого тела.
    fn axis_interval(mesh: &Mesh) -> Option<(f64, f64)> {
        let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
        for face in &mesh.faces {
            let p = mesh.vertices[face[0]];
            let normal = face[1..].windows(2).find_map(|pair| {
                let (q, r) = (mesh.vertices[pair[0]], mesh.vertices[pair[1]]);
                let (u, v) = (
                    [q[0] - p[0], q[1] - p[1], q[2] - p[2]],
                    [r[0] - p[0], r[1] - p[1], r[2] - p[2]],
                );
                let n = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                (n.iter().map(|x| x * x).sum::<f64>() > 1e-12).then_some(n)
            })?;
            let bound = normal[0] * p[0] + normal[1] * p[1] + normal[2] * (p[2] - 31.5);
            if normal[1].abs() < 1e-8 {
                if bound < -1e-6 {
                    return None;
                }
            } else if normal[1] > 0. {
                hi = hi.min(bound / normal[1]);
            } else {
                lo = lo.max(bound / normal[1]);
            }
        }
        (hi > lo + 1e-8).then_some((lo, hi))
    }
    #[test]
    fn literal_gdl_t_shapes_expose_source_axis_gaps_at_193_by_63() {
        let cases = [
            (
                vec![
                    block("Type6", 640, 180, [320., 0.], &["Type6:x2:y1:w"]),
                    block("Type5_1", 640, 90, [0., 0.], &["Type5_1:x1:y2:w"]),
                ],
                (95.4349, 97.0433),
            ),
            (
                vec![
                    block("Type8_1", 640, 0, [0., 0.], &["Type8_1:x1:y1:w"]),
                    block("Type10_1", 320, 90, [0., 0.], &["Type10_1:x1:y2:w"]),
                    block("Type8", 640, 180, [0., 0.], &["Type8:x1:y3:w"]),
                ],
                (12.0483, 12.344327196610607),
            ),
        ];
        for (blocks, expected) in cases {
            let mut spans: Vec<_> = blocks
                .iter()
                .flat_map(|b| node_stock(b, 193., 63.).unwrap())
                .filter_map(|m| axis_interval(&m))
                .collect();
            spans.sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut end = spans[0].1;
            let mut gaps = Vec::new();
            for (a, b) in spans.into_iter().skip(1) {
                if a > end + 1e-7 {
                    gaps.push((end, a));
                }
                end = end.max(b);
            }
            assert_eq!(gaps.len(), 1);
            assert!((gaps[0].0 - expected.0).abs() < 1e-7);
            assert!((gaps[0].1 - expected.1).abs() < 1e-7);
        }
    }
    #[test]
    fn distal_clip_exports_only_finished_node_stock() {
        let original = block("Type2", 320, 0, [0., 0.], &["Type2:x1:y3:w"]);
        let expected = node_stock(&original, 193., 63.).unwrap();
        let mut clipped = original;
        clipped.length_centimm = 29300;
        clipped.catalog_nominal_centimm = Some(32000);
        clipped.cuts.push("arm:w:distal".into());
        let actual = node_stock(&clipped, 193., 63.).unwrap();
        assert!(actual.iter().flat_map(|m| &m.vertices).all(|p| p[0] <= 293.000001));
        let before: f64 = expected.iter().map(crate::solid_geometry::volume).sum();
        let after: f64 = actual.iter().map(crate::solid_geometry::volume).sum();
        assert!(after < before);
        clipped.cuts.push("Type2:xBAD:y3:w".into());
        assert!(node_stock(&clipped, 193., 63.).is_err());
    }
    #[test]
    fn concave_cut_preserves_polygon_area() {
        let p = vec![[0., 0.], [4., 0.], [4., 4.], [2., 2.], [0., 4.]];
        let sum: f64 = triangles(p)
            .unwrap()
            .iter()
            .map(|t| cross(t[0], t[1], t[2]) / 2.)
            .sum();
        assert!((sum - 12.).abs() < 1e-8);
    }
    #[test]
    fn profile_warning_identifies_only_the_t1_stem() {
        let stem = block("Type10_1", 320, 90, [0., 0.], &["Type10_1:x1:y2:w"]);
        let before = node_stock(&stem, 193., 63.).unwrap();
        let warning = node_profile_warning(&stem).unwrap();
        assert_eq!(warning.code, "CATALOG_NODE_PROFILE_OVERLAP");
        assert_eq!(warning.source_id, Some(stem.id.clone()));
        assert_eq!(warning.source_ids, stem.source_ids);
        assert_eq!(warning.course_index, Some(stem.course_index));
        assert_eq!(warning.occurrences, 1);
        assert_eq!(node_stock(&stem, 193., 63.).unwrap(), before);
        assert!(
            node_profile_warning(&block("Type5_1", 640, 90, [0., 0.], &["Type5_1:x1:y2:w"]))
                .is_none()
        );
    }
    #[test]
    fn type5_end_cut_mirrors_the_begin_cut() {
        let a = polygons(
            &Cut {
                name: "Type5_1",
                slot: 1,
                face: 2,
                position_mm: None,
            },
            2,
            320.,
        )
        .unwrap();
        let b = polygons(
            &Cut {
                name: "Type5_1",
                slot: 2,
                face: 2,
                position_mm: None,
            },
            2,
            320.,
        )
        .unwrap();
        for (a, b) in a[0].iter().zip(&b[0]) {
            assert!((a[0] + b[0] - 320.).abs() < 1e-8);
            assert!((a[1] + b[1]).abs() < 1e-8);
        }
    }

    #[test]
    fn relocated_internal_profile_keeps_its_shape_on_a_shorter_part() {
        let original = block("Type6", 640, 0, [0., 0.], &["Type6:x2:y1:w"]);
        let mut shortened = block("Type6", 640, 0, [0., 0.], &["Type6:p16000:y1:w"]);
        shortened.length_centimm = 48000;
        let original_volume: f64 = node_stock(&original, 193., 63.).unwrap().iter().map(crate::solid_geometry::volume).sum();
        let actual_volume: f64 = node_stock(&shortened, 193., 63.).unwrap().iter().map(crate::solid_geometry::volume).sum();
        assert!((original_volume - actual_volume - 160. * 193. * 63.).abs() < 1e-4);
    }
    #[test]
    fn all_catalog_profiles_triangulate() {
        for (name, slot, face) in [
            ("Type1", 1, 1),
            ("Type2", 1, 3),
            ("Type3", 1, 3),
            ("Type4", 1, 1),
            ("Type5_1", 1, 2),
            ("Type6", 2, 1),
            ("Type6", 2, 3),
            ("Type8", 1, 1),
            ("Type8_1", 1, 3),
            ("Type10_1", 1, 2),
        ] {
            for p in polygons(&Cut { name, slot, face, position_mm: None }, 3, 640.).unwrap() {
                assert!(!triangles(p).unwrap().is_empty());
            }
        }
    }
}

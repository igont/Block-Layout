//! Замкнутые выпуклые тела и точные плоские вычеты. Координаты в мм.
use serde::{Deserialize, Serialize};

const EPS: f64 = 1e-7;
const VOLUME_EPS: f64 = 1e-6;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mesh {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<Vec<usize>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub normal: [f64; 3],
    pub offset: f64,
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn delta(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    add(a, scale(b, -1.0))
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Оси задают ориентацию исходного прямоугольного изделия.
pub fn box_mesh(origin: [f64; 3], axes: [[f64; 3]; 3], size: [f64; 3]) -> Mesh {
    let mut vertices = Vec::with_capacity(8);
    for i in 0..8 {
        let mut point = origin;
        for j in 0..3 {
            point = add(point, scale(axes[j], size[j] * ((i >> j) & 1) as f64));
        }
        vertices.push(point);
    }
    let mut mesh = Mesh {
        vertices,
        faces: vec![
            vec![0, 4, 6, 2],
            vec![1, 3, 7, 5],
            vec![0, 1, 5, 4],
            vec![2, 6, 7, 3],
            vec![0, 2, 3, 1],
            vec![4, 5, 7, 6],
        ],
    };
    if dot(cross(axes[0], axes[1]), axes[2]) < 0.0 {
        for face in &mut mesh.faces {
            face.reverse();
        }
    }
    mesh
}

pub fn bounds(mesh: &Mesh) -> Option<([f64; 3], [f64; 3])> {
    if mesh.vertices.is_empty() || mesh.vertices.iter().flatten().any(|v| !v.is_finite()) {
        return None;
    }
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for point in &mesh.vertices {
        for j in 0..3 {
            low[j] = low[j].min(point[j]);
            high[j] = high[j].max(point[j]);
        }
    }
    Some((low, high))
}

/// Объём замкнутого тела. Перенос начала уменьшает потерю точности в world.
pub fn volume(mesh: &Mesh) -> f64 {
    let Some(&origin) = mesh.vertices.first() else {
        return 0.0;
    };
    let mut six = 0.0;
    for face in &mesh.faces {
        if face.len() < 3 || face.iter().any(|&i| i >= mesh.vertices.len()) {
            return 0.0;
        }
        let a = delta(mesh.vertices[face[0]], origin);
        for pair in face[1..].windows(2) {
            six += dot(
                a,
                cross(
                    delta(mesh.vertices[pair[0]], origin),
                    delta(mesh.vertices[pair[1]], origin),
                ),
            );
        }
    }
    if six.is_finite() {
        six.abs() / 6.0
    } else {
        0.0
    }
}

fn index_vertex(mesh: &mut Mesh, point: [f64; 3]) -> usize {
    if let Some(i) = mesh
        .vertices
        .iter()
        .position(|&v| dot(delta(v, point), delta(v, point)) <= EPS * EPS)
    {
        i
    } else {
        mesh.vertices.push(point);
        mesh.vertices.len() - 1
    }
}

fn push_face(mesh: &mut Mesh, points: Vec<[f64; 3]>) {
    let mut indices: Vec<_> = points.into_iter().map(|p| index_vertex(mesh, p)).collect();
    indices.dedup();
    if indices.len() > 1 && indices.first() == indices.last() {
        indices.pop();
    }
    if indices.len() < 3 {
        return;
    }
    let origin = mesh.vertices[indices[0]];
    let area = indices[1..].windows(2).fold([0.0; 3], |sum, pair| {
        add(
            sum,
            cross(
                delta(mesh.vertices[pair[0]], origin),
                delta(mesh.vertices[pair[1]], origin),
            ),
        )
    });
    if dot(area, area) > EPS.powi(4) {
        mesh.faces.push(indices);
    }
}

/// Округление тела сохраняет общие вершины и удаляет схлопнувшиеся грани.
/// Простая замена координат оставляет разные индексы у одной точки.
pub fn quantized(mesh: &Mesh) -> Mesh {
    let mut result = Mesh { vertices: Vec::new(), faces: Vec::new() };
    for face in &mesh.faces {
        push_face(&mut result, face.iter().map(|&i| {
            mesh.vertices[i].map(crate::precision::mm)
        }).collect());
    }
    let mut used = vec![false; result.vertices.len()];
    for face in &result.faces {
        for &i in face { used[i] = true; }
    }
    let mut remap = vec![0; used.len()];
    let mut vertices = Vec::new();
    for (i, &point) in result.vertices.iter().enumerate() {
        if used[i] {
            remap[i] = vertices.len();
            vertices.push(point);
        }
    }
    for face in &mut result.faces {
        for i in face { *i = remap[*i]; }
    }
    result.vertices = vertices;
    result
}

/// Пересечение с полупространством dot(normal, point) <= offset.
/// Каждая новая граница закрывается гранью с наружной нормалью plane.normal.
pub fn clip(mesh: &Mesh, plane: &Plane) -> Option<Mesh> {
    bounds(mesh)?;
    let norm = dot(plane.normal, plane.normal).sqrt();
    if !norm.is_finite() || norm <= EPS || !plane.offset.is_finite() {
        return None;
    }
    let normal = scale(plane.normal, 1.0 / norm);
    let offset = plane.offset / norm;
    let distances: Vec<_> = mesh
        .vertices
        .iter()
        .map(|&p| dot(normal, p) - offset)
        .collect();
    if distances.iter().all(|&d| d <= EPS) {
        return (volume(mesh) > VOLUME_EPS).then(|| mesh.clone());
    }
    if distances.iter().all(|&d| d >= -EPS) {
        return None;
    }
    let mut result = Mesh {
        vertices: Vec::new(),
        faces: Vec::new(),
    };
    for face in &mesh.faces {
        if face.len() < 3 || face.iter().any(|&i| i >= mesh.vertices.len()) {
            return None;
        }
        let mut polygon = Vec::new();
        let mut previous = *face.last()?;
        for &current in face {
            let a = mesh.vertices[previous];
            let b = mesh.vertices[current];
            let da = distances[previous];
            let db = distances[current];
            if (da <= 0.0) != (db <= 0.0) {
                polygon.push(add(a, scale(delta(b, a), da / (da - db))));
            }
            if db <= 0.0 {
                polygon.push(b);
            }
            previous = current;
        }
        push_face(&mut result, polygon);
    }
    let mut cap: Vec<_> = result
        .vertices
        .iter()
        .copied()
        .filter(|&p| (dot(normal, p) - offset).abs() <= EPS)
        .collect();
    if cap.len() >= 3 {
        let center = scale(
            cap.iter().fold([0.0; 3], |sum, &p| add(sum, p)),
            1.0 / cap.len() as f64,
        );
        let reference = if normal[0].abs() < 0.8 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let tangent = cross(reference, normal);
        let tangent = scale(tangent, 1.0 / dot(tangent, tangent).sqrt());
        let second = cross(normal, tangent);
        cap.sort_by(|&a, &b| {
            let a = delta(a, center);
            let b = delta(b, center);
            dot(a, second)
                .atan2(dot(a, tangent))
                .total_cmp(&dot(b, second).atan2(dot(b, tangent)))
        });
        push_face(&mut result, cap);
    }
    (volume(&result) > VOLUME_EPS).then_some(result)
}

/// Вычитает выпуклую пустоту, заданную пересечением полупространств.
/// Каждый фрагмент остаётся замкнутым и выпуклым; их внутренние объёмы не пересекаются.
pub fn subtract(mesh: &Mesh, void_planes: &[Plane]) -> Vec<Mesh> {
    let mut pieces = Vec::new();
    let mut remaining = Some(mesh.clone());
    for plane in void_planes {
        let Some(current) = remaining.take() else {
            break;
        };
        let outside = Plane {
            normal: scale(plane.normal, -1.0),
            offset: -plane.offset,
        };
        if let Some(piece) = clip(&current, &outside) {
            pieces.push(piece);
        }
        remaining = clip(&current, plane);
    }
    pieces
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    const AXES: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    fn assert_closed(mesh: &Mesh) {
        let mut edges = BTreeMap::<(usize, usize), (usize, i32)>::new();
        for face in &mesh.faces {
            for i in 0..face.len() {
                let (a, b) = (face[i], face[(i + 1) % face.len()]);
                let entry = edges.entry((a.min(b), a.max(b))).or_default();
                entry.0 += 1;
                entry.1 += if a < b { 1 } else { -1 };
            }
        }
        assert!(edges.values().all(|&value| value == (2, 0)), "{edges:?}");
        assert!(mesh.vertices.iter().flatten().all(|v| v.is_finite()));
    }

    #[test]
    fn cube_volume_and_slanted_clip_have_closed_faces() {
        let cube = box_mesh([0.0; 3], AXES, [10.0; 3]);
        assert_eq!(volume(&cube), 1000.0);
        assert_closed(&cube);
        let half = clip(
            &cube,
            &Plane {
                normal: [1.0, 1.0, 0.0],
                offset: 10.0,
            },
        )
        .unwrap();
        assert!((volume(&half) - 500.0).abs() < 1e-8);
        assert_closed(&half);
        assert_eq!(bounds(&half), Some(([0.0; 3], [10.0; 3])));
    }

    #[test]
    fn quantized_sloping_lamella_welds_collapsed_edge_and_stays_closed() {
        let blank = box_mesh([-96.5, 1944.28, 4032.0], AXES, [15.0, 615.72, 63.0]);
        let slope = 63.0 / 142.26;
        let clipped = clip(&blank, &Plane {
            normal: [0.0, -slope, 1.0],
            offset: 4032.002 - slope * 1944.28,
        }).unwrap();
        assert!(clipped.vertices.len() > 8, "Срез должен создать короткое ребро");
        let rounded = quantized(&clipped);
        assert_eq!(rounded.vertices.len(), 8);
        assert_eq!(rounded.faces.len(), 6);
        assert_closed(&rounded);
        assert!((volume(&rounded) - 15.0 * 63.0 * (615.72 - 142.26 / 2.0)).abs() < 1e-6);
        assert_eq!(quantized(&rounded), rounded);
    }

    #[test]
    fn subtract_interior_cube_preserves_independent_volume() {
        let cube = box_mesh([0.0; 3], AXES, [10.0; 3]);
        let mut planes = Vec::new();
        for axis in AXES {
            planes.push(Plane {
                normal: axis,
                offset: 7.5,
            });
            planes.push(Plane {
                normal: scale(axis, -1.0),
                offset: -2.5,
            });
        }
        let pieces = subtract(&cube, &planes);
        assert_eq!(pieces.len(), 6);
        assert!((pieces.iter().map(volume).sum::<f64>() - 875.0).abs() < 1e-8);
        for piece in &pieces {
            assert_closed(piece);
        }
    }

    #[test]
    fn beam_pocket_keeps_skin_and_vertical_material() {
        let wall = box_mesh([0.0; 3], AXES, [160.0, 193.0, 200.0]);
        let planes = [
            Plane {
                normal: [1.0, 0.0, 0.0],
                offset: 160.0,
            },
            Plane {
                normal: [-1.0, 0.0, 0.0],
                offset: 0.0,
            },
            Plane {
                normal: [0.0, 1.0, 0.0],
                offset: 173.0,
            },
            Plane {
                normal: [0.0, -1.0, 0.0],
                offset: 0.0,
            },
            Plane {
                normal: [0.0, 0.0, 1.0],
                offset: 150.0,
            },
            Plane {
                normal: [0.0, 0.0, -1.0],
                offset: -30.0,
            },
        ];
        let pieces = subtract(&wall, &planes);
        assert!(
            (pieces.iter().map(volume).sum::<f64>()
                - (160.0 * 193.0 * 200.0 - 160.0 * 173.0 * 120.0))
                .abs()
                < 1e-6
        );
        assert!(pieces
            .iter()
            .any(|p| bounds(p) == Some(([0.0, 173.0, 0.0], [160.0, 193.0, 200.0]))));
        for piece in &pieces {
            assert_closed(piece);
        }
    }

    #[test]
    fn touching_plane_zero_thickness_and_far_world_are_stable() {
        let cube = box_mesh([1e6, -1e6, 1e6], AXES, [10.0; 3]);
        let half = clip(
            &cube,
            &Plane {
                normal: [2.0, 0.0, 0.0],
                offset: 2_000_010.0,
            },
        )
        .unwrap();
        assert_eq!(volume(&half), 500.0);
        assert_closed(&half);
        assert!(clip(
            &cube,
            &Plane {
                normal: [1.0, 0.0, 0.0],
                offset: 1e6
            }
        )
        .is_none());
        assert_eq!(
            clip(
                &cube,
                &Plane {
                    normal: [1.0, 0.0, 0.0],
                    offset: 1e6 + 10.0
                }
            ),
            Some(cube)
        );
    }
}

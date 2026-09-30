//! Мировые координаты швов обычной сетки на ортогональных осях.

use crate::domain::{Point, WallRun};

const HALF_MODULE_CENTIMM: i64 = 32_000;
const MODULE_CENTIMM: i128 = 64_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GridError {
    UnsupportedDirection(u16),
    UnsupportedRunAxis { dx: i128, dy: i128 },
}

/// Остаток координаты очередного ordinary-шва от канонического начала run.
/// Для диагонали мирового остатка нет: её ось имеет отдельный локальный origin.
pub fn joint_residue(course_index: i64, run: &WallRun) -> Result<Option<i64>, GridError> {
    let dx = i128::from(run.end.x) - i128::from(run.start.x);
    let dy = i128::from(run.end.y) - i128::from(run.start.y);
    let (world_seam, origin) = if dx != 0 && dy == 0 {
        (
            i128::from(HALF_MODULE_CENTIMM) * (i128::from(course_index) + 1)
                - i128::from(run.start.y),
            i128::from(run.start.x),
        )
    } else if dx == 0 && dy != 0 {
        (
            i128::from(HALF_MODULE_CENTIMM) * i128::from(course_index) - i128::from(run.start.x),
            i128::from(run.start.y),
        )
    } else if dx != 0 && dx.abs() == dy.abs() {
        return Ok(None);
    } else {
        return Err(GridError::UnsupportedRunAxis { dx, dy });
    };
    Ok(Some((world_seam - origin).rem_euclid(MODULE_CENTIMM) as i64))
}

/// Проверяет физический шов в мировой точке без округления координат.
pub fn is_world_joint(
    course_index: i64,
    point: Point,
    direction_deg: u16,
) -> Result<Option<bool>, GridError> {
    let (axis_coordinate, world_seam) = match direction_deg {
        0 | 180 => (
            i128::from(point.x),
            i128::from(HALF_MODULE_CENTIMM) * (i128::from(course_index) + 1) - i128::from(point.y),
        ),
        90 | 270 => (
            i128::from(point.y),
            i128::from(HALF_MODULE_CENTIMM) * i128::from(course_index) - i128::from(point.x),
        ),
        45 | 135 | 225 | 315 => return Ok(None),
        other => return Err(GridError::UnsupportedDirection(other)),
    };
    Ok(Some(
        (axis_coordinate - world_seam).rem_euclid(MODULE_CENTIMM) == 0,
    ))
}

#[cfg(test)]
mod tests {
    use super::{is_world_joint, joint_residue, GridError};
    use crate::domain::{Point, WallRun};

    fn run(start: (i64, i64), end: (i64, i64)) -> WallRun {
        WallRun {
            id: "test".into(),
            start: Point {
                x: start.0,
                y: start.1,
            },
            end: Point { x: end.0, y: end.1 },
            length: 100_000,
            sources: vec![],
        }
    }

    #[test]
    fn origin_seams_are_x320_and_y0_on_even_course() {
        assert_eq!(
            joint_residue(0, &run((0, 0), (128_000, 0))),
            Ok(Some(32_000))
        );
        assert_eq!(joint_residue(0, &run((0, 0), (0, 128_000))), Ok(Some(0)));
        assert_eq!(
            is_world_joint(0, Point { x: 32_000, y: 0 }, 0),
            Ok(Some(true))
        );
        assert_eq!(is_world_joint(0, Point { x: 0, y: 0 }, 90), Ok(Some(true)));
        assert_eq!(is_world_joint(0, Point { x: 0, y: 0 }, 0), Ok(Some(false)));
    }

    #[test]
    fn odd_and_upper_course_flip_both_seams_globally() {
        let horizontal = run((0, 0), (128_000, 0));
        let vertical = run((0, 0), (0, 128_000));
        assert_eq!(joint_residue(1, &horizontal), Ok(Some(0)));
        assert_eq!(joint_residue(1, &vertical), Ok(Some(32_000)));
        assert_eq!(joint_residue(40, &horizontal), Ok(Some(32_000)));
        assert_eq!(joint_residue(41, &horizontal), Ok(Some(0)));
        assert_eq!(is_world_joint(41, Point { x: 0, y: 0 }, 0), Ok(Some(true)));
    }

    #[test]
    fn perpendicular_shift_and_local_origin_change_residue_exactly() {
        assert_eq!(
            joint_residue(0, &run((0, 32_000), (128_000, 32_000))),
            Ok(Some(0))
        );
        assert_eq!(
            joint_residue(0, &run((32_000, 0), (32_000, 128_000))),
            Ok(Some(32_000))
        );
        assert_eq!(
            joint_residue(0, &run((10_000, 0), (138_000, 0))),
            Ok(Some(22_000))
        );
        assert_eq!(
            joint_residue(0, &run((0, -32_000), (128_000, -32_000))),
            Ok(Some(0))
        );
    }

    #[test]
    fn off_grid_and_negative_coordinates_are_not_snapped() {
        assert_eq!(
            joint_residue(0, &run((0, 1), (128_000, 1))),
            Ok(Some(31_999))
        );
        assert_eq!(
            is_world_joint(0, Point { x: 31_999, y: 1 }, 0),
            Ok(Some(true))
        );
        assert_eq!(
            is_world_joint(0, Point { x: 32_000, y: 1 }, 0),
            Ok(Some(false))
        );
        assert_eq!(
            is_world_joint(0, Point { x: -32_000, y: 0 }, 0),
            Ok(Some(true))
        );
        assert_eq!(
            is_world_joint(0, Point { x: 0, y: -64_000 }, 90),
            Ok(Some(true))
        );
    }

    #[test]
    fn diagonal_has_no_world_seam_and_invalid_axis_is_typed() {
        assert_eq!(joint_residue(0, &run((0, 0), (64_000, 64_000))), Ok(None));
        assert_eq!(
            is_world_joint(
                0,
                Point {
                    x: 32_000,
                    y: 32_000
                },
                45
            ),
            Ok(None)
        );
        assert_eq!(
            joint_residue(0, &run((0, 0), (64_000, 32_000))),
            Err(GridError::UnsupportedRunAxis {
                dx: 64_000,
                dy: 32_000
            })
        );
        assert_eq!(
            is_world_joint(0, Point { x: 0, y: 0 }, 30),
            Err(GridError::UnsupportedDirection(30))
        );
    }
}

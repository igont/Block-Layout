//! Общая точность размерной геометрии на границах расчёта: 0,01 мм.
use serde_json::Value;

pub const SCALE: f64 = 100.0;
pub const QUANTUM_MM: f64 = 1.0 / SCALE;

/// Целое количество сотых мм, ещё в f64 для проверяемого преобразования в i64.
pub fn centimm(value: f64) -> f64 {
    let units = value * SCALE;
    let lower = units.floor();
    // Десятичная половина, например 1,015, после умножения может оказаться
    // чуть ниже 101,5. Компенсируем только погрешность представления числа.
    let tolerance = (units.abs() * f64::EPSILON * 2.0).max(1e-11);
    if (units - lower - 0.5).abs() <= tolerance {
        if lower % 2.0 == 0.0 {
            lower
        } else {
            lower + 1.0
        }
    } else {
        units.round_ties_even()
    }
}

/// Округление к ближайшей сотой; ровно половина — к чётной сотой.
/// Неконечные значения сохраняются для штатной проверки входной геометрии.
pub fn mm(value: f64) -> f64 {
    let units = centimm(value);
    if !units.is_finite() {
        return value;
    }
    if units == 0.0 {
        0.0
    } else {
        units / SCALE
    }
}

/// Нейтральный JSON использует суффикс _mm для размерных полей.
/// Mesh.vertices — единственное безразмерно названное поле координат.
/// Оси, нормали, индексы, идентичность и текст не изменяются.
pub fn normalize_result(value: &mut Value) {
    match value {
        Value::Object(fields) => {
            for (key, value) in fields {
                if key.ends_with("_mm") || key == "vertices" {
                    normalize_numbers(value);
                } else {
                    normalize_result(value);
                }
            }
        }
        Value::Array(values) => values.iter_mut().for_each(normalize_result),
        _ => {}
    }
}

fn normalize_numbers(value: &mut Value) {
    match value {
        Value::Number(number) if number.is_f64() => {
            *value = Value::from(mm(number.as_f64().unwrap()));
        }
        Value::Array(values) => values.iter_mut().for_each(normalize_numbers),
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimal_rounding_is_signed_idempotent_and_preserves_invalid_values() {
        for (raw, expected) in [
            (2331.000000000002, 2331.0),
            (-251.99999999999844, -252.0),
            (1.015, 1.02),
            (1.025, 1.02),
            (-1.015, -1.02),
            (-1.025, -1.02),
            (1.0149, 1.01),
            (1.0151, 1.02),
            (-0.004, 0.0),
            (999999.995, 1000000.0),
        ] {
            assert_eq!(mm(raw), expected, "{raw}");
            assert_eq!(mm(mm(raw)), expected);
        }
        assert_eq!(mm(-0.004).to_bits(), 0.0_f64.to_bits());
        assert!(mm(f64::NAN).is_nan());
        assert_eq!(mm(f64::INFINITY), f64::INFINITY);
    }

    #[test]
    fn output_rounds_dimensions_and_meshes_but_preserves_directions_and_identity() {
        let axis = std::f64::consts::FRAC_1_SQRT_2;
        let mut value = serde_json::json!({
            "id":"part-1.015", "course_index":41,
            "placement":{"origin_mm":[1.015,-0.004,2331.000000000002],"x_axis":[axis,axis,0.0]},
            "length_mm":640.004, "solid":{"vertices":[[1.015,2.025,3.0]],"faces":[[0,1,2]]},
            "planes_local":[{"normal":[axis,axis,0.0],"offset_mm":1.025}]
        });
        normalize_result(&mut value);
        assert_eq!(
            value["placement"]["origin_mm"],
            serde_json::json!([1.02, 0.0, 2331.0])
        );
        assert_eq!(
            value["solid"]["vertices"],
            serde_json::json!([[1.02, 2.02, 3.0]])
        );
        assert_eq!(value["length_mm"], 640.0);
        assert_eq!(value["planes_local"][0]["offset_mm"], 1.02);
        assert_eq!(value["placement"]["x_axis"][0], axis);
        assert_eq!(value["planes_local"][0]["normal"][0], axis);
        assert_eq!(value["id"], "part-1.015");
        assert_eq!(value["course_index"], 41);
        assert_eq!(value["solid"]["faces"], serde_json::json!([[0, 1, 2]]));
        let once = value.clone();
        normalize_result(&mut value);
        assert_eq!(value, once);
    }
}

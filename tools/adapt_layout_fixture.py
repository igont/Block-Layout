"""Перенос исследовательского снимка в fb-layout/1 без изменения исходного файла.

Геометрические поля совпадают с exchange::from_layout_request. Метаданные
приложения исключены; стандартный CLI отдельно проверяет геометрию и профиль.
Этот адаптер не создаёт эталоны раскладки или подтверждение каталога.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys


def read_json(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def prism(volume):
    thickness = volume["thicknessMm"] / 2
    return {
        "start_xy_mm": [volume["startXmm"], volume["startYmm"]],
        "end_xy_mm": [volume["endXmm"], volume["endYmm"]],
        "bottom_start_mm": volume["startBottomZmm"],
        "bottom_end_mm": volume["endBottomZmm"],
        "top_start_mm": volume["startTopZmm"],
        "top_end_mm": volume["endTopZmm"],
        "left_thickness_mm": thickness,
        "right_thickness_mm": thickness,
    }


def adapt(source, profile):
    if source["schema_version"] != 1:
        raise ValueError("Поддерживается только исследовательский снимок schema_version=1")
    snapshot_hash = source["snapshot_hash"]
    if not isinstance(snapshot_hash, str) or not re.fullmatch("[0-9a-f]{64}", snapshot_hash):
        raise ValueError("snapshot_hash должен содержать 64 шестнадцатеричных символа")
    walls = []
    for wall in source["wall_volumes"]:
        if wall.get("purposeType", 0) != 1:
            raise ValueError(f"Назначение стены {wall['guid']} не поддержано прототипом")
        walls.append({"id": wall["guid"], "purpose": "fb_console" if wall.get("openingType") == "CONSOLE" else "fb_wall", "volume": prism(wall)})
    openings = []
    purposes = {"OPENING": "opening", "CONSOLE": "console", "WINDOW": "window",
                "DOOR": "door", "PARTITION_OPENING": "partition_opening"}
    for opening in source.get("opening_volumes", []):
        purpose = purposes.get(opening.get("openingType", ""))
        if purpose is None:
            raise ValueError(f"Назначение проёма {opening['guid']} нельзя перенести без потерь")
        entry = {"id": opening["guid"], "purpose": purpose, "volume": prism(opening)}
        if opening.get("isOutside", False):
            entry["is_outside"] = True
        openings.append(entry)
    beams = []
    for beam in source.get("beams", []):
        geometry = beam["geometry"]
        beams.append({
            "id": beam["guid"],
            "start_mm": [beam["startXmm"], beam["startYmm"], beam["startZmm"]],
            "end_mm": [beam["endXmm"], beam["endYmm"], beam["endZmm"]],
            "width_mm": geometry["widthMm"], "height_mm": geometry["heightMm"],
            "height_direction": [geometry.get("heightDirectionX", 0),
                                 geometry.get("heightDirectionY", 0),
                                 geometry.get("heightDirectionZ", 0)],
        })
    return {
        "format": "fb-layout/1", "kind": "layout_request",
        "request_id": source.get("request_id") or "snapshot-" + snapshot_hash,
        "snapshot_hash": snapshot_hash,
        "coordinate_system": {"id": "snapshot-world", "units": "mm",
                              "coordinate_quantum_mm": 0.01, "z0_mm": source["z0_mm"]},
        "profile": {"id": profile["profile_id"], "revision": profile["revision"]},
        "catalog": {"id": profile["catalog_version"], "revision": profile["revision"]},
        "scope": {"mode": "full"},
        "model": {"wall_volumes": walls, "openings": openings, "beams": beams},
    }


def walls_only(request):
    """Явный первый этап: новый снимок со всеми стенами, без проёмов и балок."""
    request["model"]["openings"] = []
    request["model"]["beams"] = []
    request["model"]["wall_volumes"].sort(key=lambda wall: wall["id"])
    context = {key: request[key] for key in ["format", "coordinate_system", "profile", "catalog", "scope", "model"]}
    canonical = json.dumps(context, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))
    request["snapshot_hash"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    request["request_id"] += "-walls-only"
    return request


def main():
    parser = argparse.ArgumentParser(description="Перенос снимка в нейтральный запрос fb-layout/1")
    parser.add_argument("--input", type=Path, required=True, help="Исходный JSON или JSON.GZ")
    parser.add_argument("--profile", type=Path, required=True, help="Профиль прототипа JSON")
    parser.add_argument("--output", type=Path, required=True, help="Новый запрос JSON")
    parser.add_argument("--walls-only", action="store_true", help="Первый этап: все стены без проёмов и балок; новый хеш снимка")
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve() or args.profile.resolve() == args.output.resolve():
        parser.error("Выходной файл должен отличаться от исходного снимка и профиля")
    try:
        source = read_json(args.input)
        if args.walls_only:
            # Исключение происходит до адаптации: неподдержанные данные других
            # этапов не ограничивают текущий расчёт стен.
            filtered = dict(source, opening_volumes=[], beams=[])
            request = walls_only(adapt(filtered, read_json(args.profile)))
        else:
            request = adapt(source, read_json(args.profile))
        text = json.dumps(request, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        if args.walls_only:
            provenance = {
                "mode": "walls_only", "source_path": str(args.input.resolve()),
                "source_snapshot_hash": source["snapshot_hash"],
                "snapshot_hash": request["snapshot_hash"], "request_id": request["request_id"],
                "excluded": {"openings": len(source.get("opening_volumes", [])), "beams": len(source.get("beams", []))},
            }
            provenance_path = args.output.parent / "walls-source.json"
            if provenance_path.resolve() in [args.input.resolve(), args.profile.resolve(), args.output.resolve()]:
                raise ValueError("Файл происхождения должен отличаться от входа, профиля и запроса")
            provenance_path.write_text(json.dumps(provenance, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
            print("Этап только стены: проёмы и балки намеренно исключены; хеш описывает новый снимок", file=sys.stderr)
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f"Адаптация снимка отклонена: {error}", file=sys.stderr)
        return 1
    print(f"Запрос fb-layout/1 сохранён: {args.output}")
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    sys.exit(main())

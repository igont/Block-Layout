#!/usr/bin/env python3
"""Экспорт текущих моделей Ленинского в воспроизводимые fixtures.

Зависимости существующего Python: python -m pip install --user cbor2 simplejson
Запуск: python tools/export_sup_snapshots.py [--verify]
--verify сравнивает готовые fixtures с актуальными SQLite, ничего не записывая.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import os
import sqlite3
from pathlib import Path

import cbor2
import simplejson as json


PROJECTS = ("Дом", "Баня", "Гараж")
SECTIONS = {
    "wallVolumes": "wall_volumes",
    "openingVolumes": "opening_volumes",
    "beams": "beams",
    "blocks": "observed_blocks",
}
SOURCE_ROOT = Path("C:/SupApplication/Projects/Индивидуальные проекты/Ленинское")
OUTPUT_ROOT = Path(__file__).resolve().parents[1] / "tests" / "fixtures" / "leninskoe"


def json_bytes(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), use_decimal=True).encode("utf-8")


def project_id(project: str) -> str:
    path = SOURCE_ROOT / project / "variant-0" / ".sup" / "project.tsv"
    with path.open("r", encoding="utf-8", newline="") as source:
        rows = csv.DictReader(source, delimiter="\t")
        keys = [row["value"] for row in rows if row["namespace"] == "project" and row["name"] == "projectKey"]
    if len(keys) != 1:
        raise ValueError(f"Ожидался один projectKey: {path}")
    return keys[0]


def digest_part(digest: object, value: bytes) -> None:
    digest.update(len(value).to_bytes(8, "big"))
    digest.update(value)


def read_snapshot(project: str) -> tuple[dict, dict]:
    db = SOURCE_ROOT / project / "variant-0" / ".sup" / "model" / "current.sqlite"
    if not db.is_file():
        raise FileNotFoundError(db)
    connection = sqlite3.connect(db.as_uri() + "?mode=ro", uri=True)
    try:
        connection.execute("PRAGMA query_only=ON")
        connection.execute("BEGIN")
        version = connection.execute("PRAGMA user_version").fetchone()[0]
        sections = {}
        arrays = {}
        digest = hashlib.sha256()
        digest_part(digest, str(version).encode("ascii"))
        for section_id, field in SECTIONS.items():
            row = connection.execute(
                "SELECT section_id,model_id,export_id,position,contract_id,shape,row_count,content_hash "
                "FROM current_section WHERE section_id=?", (section_id,)
            ).fetchone()
            if row is None or row[5] != "array":
                raise ValueError(f"Нет секции-массива {section_id}: {db}")
            info = dict(zip(("section_id", "model_id", "export_id", "position", "contract_id", "shape", "row_count", "content_hash"), row))
            sections[section_id] = info
            digest_part(digest, json_bytes(info))
            items = []
            count = 0
            for ordinal, guid, layer, type_name, payload in connection.execute(
                "SELECT ordinal,guid,layer,type,payload FROM current_entity "
                "WHERE section_id=? ORDER BY ordinal", (section_id,)
            ):
                if ordinal != count:
                    raise ValueError(f"Пропуск ordinal в {section_id}: {ordinal} вместо {count}")
                count += 1
                decoded = cbor2.loads(payload)
                if not isinstance(decoded, dict):
                    raise ValueError(f"Ожидался объект CBOR в {section_id}[{ordinal}]")
                digest_part(digest, json_bytes([section_id, ordinal, guid, layer, type_name]))
                digest_part(digest, payload)
                if section_id != "wallVolumes" or decoded.get("purposeType") == 1:
                    items.append(decoded)
            if count != info["row_count"]:
                raise ValueError(f"Число строк {section_id}: {count} != {info['row_count']}")
            info["selected_count"] = len(items)
            arrays[field] = items
        model_ids = {section["model_id"] for section in sections.values()}
        export_ids = {section["export_id"] for section in sections.values()}
        if len(model_ids) != 1 or len(export_ids) != 1:
            raise ValueError(f"Разделы {project} относятся к разным model_id/export_id")
        walls = arrays["wall_volumes"]
        if not walls:
            raise ValueError(f"Нет стен purposeType=1: {project}")
        z0 = min(min(wall["startBottomZmm"], wall["endBottomZmm"]) for wall in walls)
        if not isinstance(z0, int) or z0 % 63 != 0:
            raise ValueError(f"z0_mm не кратен 63: {project}: {z0}")
        snapshot_hash = digest.hexdigest()
        common = {
            "schema_version": 1,
            "project_name": project,
            "project_id": project_id(project),
            "snapshot_hash": snapshot_hash,
            "z0_mm": z0,
            "metadata": {
                "sqlite_user_version": version,
                "model_id": next(iter(model_ids)),
                "export_id": next(iter(export_ids)),
                "sections": sections,
            },
        }
        inputs = {**common, "wall_volumes": arrays["wall_volumes"], "opening_volumes": arrays["opening_volumes"], "beams": arrays["beams"]}
        observed = {**common, "observed_blocks": arrays["observed_blocks"]}
        return inputs, observed
    finally:
        connection.rollback()
        connection.close()


def load_fixture(path: Path) -> dict:
    with gzip.open(path, "rt", encoding="utf-8") as source:
        return json.load(source, use_decimal=True)


def write_fixture(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0, compresslevel=9) as compressed:
                compressed.write(json_bytes(value))
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="проверить fixtures против текущих SQLite")
    args = parser.parse_args()
    for project in PROJECTS:
        inputs, observed = read_snapshot(project)
        directory = OUTPUT_ROOT / {"Дом": "dom", "Баня": "banya", "Гараж": "garazh"}[project]
        for filename, value in (("input.json.gz", inputs), ("observed.json.gz", observed)):
            path = directory / filename
            if not args.verify:
                write_fixture(path, value)
            restored = load_fixture(path)
            if json_bytes(restored) != json_bytes(value):
                raise ValueError(f"Roundtrip/снимок не совпал: {path}")
            print(f"{project} {filename}: {path.stat().st_size} байт, SHA256={hashlib.sha256(path.read_bytes()).hexdigest()}")
        print(
            f"{project}: wallVolumes={len(inputs['wall_volumes'])}/{inputs['metadata']['sections']['wallVolumes']['row_count']}, "
            f"openingVolumes={len(inputs['opening_volumes'])}, beams={len(inputs['beams'])}, "
            f"observed blocks={len(observed['observed_blocks'])}, z0_mm={inputs['z0_mm']}, snapshot_hash={inputs['snapshot_hash']}"
        )


if __name__ == "__main__":
    main()

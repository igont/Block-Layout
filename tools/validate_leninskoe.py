"""Проверить три frozen SUP snapshots через настоящий fb-layout CLI.

Java observed_blocks — только характеристика. Успешный ответ текущего Rust
Layout проверяется как course_layout_v1. Для будущих физических блоков нужен
отдельный контракт и адаптер; неизвестное представление не получает PASS.
"""

from __future__ import annotations

import argparse
import collections
import gzip
import hashlib
import json
import math
import re
import subprocess
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures" / "leninskoe"
PROJECTS = ("dom", "banya", "garazh")
SCALE = 100  # Координаты Rust измеряются в 0,01 мм.
COURSE = 6300
TOLERANCE = 2


def read_fixture(path: Path) -> dict:
    with gzip.open(path, "rt", encoding="utf-8") as source:
        return json.load(source)


def check_snapshot(input_data: dict, observed: dict) -> list[str]:
    problems: list[str] = []
    for key in ("schema_version", "project_name", "project_id", "snapshot_hash", "z0_mm", "metadata"):
        if input_data.get(key) != observed.get(key):
            problems.append(f"fixture {key} mismatch")
    if input_data.get("schema_version") != 1:
        problems.append("unsupported fixture schema")
    if not re.fullmatch(r"[0-9a-f]{64}", str(input_data.get("snapshot_hash", ""))):
        problems.append("invalid snapshot hash")
    seen: set[str] = set()
    for section, key in (("wallVolumes", "wall_volumes"),
                         ("openingVolumes", "opening_volumes"),
                         ("beams", "beams")):
        metadata = input_data.get("metadata", {}).get("sections", {}).get(section)
        rows = input_data.get(key)
        if not isinstance(metadata, dict) or not isinstance(rows, list):
            problems.append(f"missing {section} data or metadata")
            continue
        if len(rows) != metadata.get("selected_count"):
            problems.append(f"{section} selected_count mismatch")
        for row in rows:
            guid = row.get("guid")
            if not isinstance(guid, str) or not guid.strip():
                problems.append(f"{section} empty guid")
            elif guid in seen:
                problems.append(f"{section} duplicate guid {guid}")
            else:
                seen.add(guid)
    if not isinstance(observed.get("observed_blocks"), list):
        problems.append("missing observed_blocks characterization")
    return problems


def point(value: object) -> tuple[int, int] | None:
    if not isinstance(value, dict) or type(value.get("x")) is not int or type(value.get("y")) is not int:
        return None
    return value["x"], value["y"]


def wall_line(wall: dict) -> tuple[tuple[int, int], tuple[int, int]]:
    return ((round(wall["startXmm"] * SCALE), round(wall["startYmm"] * SCALE)),
            (round(wall["endXmm"] * SCALE), round(wall["endYmm"] * SCALE)))


def on_line(p: tuple[int, int], a: tuple[int, int], b: tuple[int, int]) -> bool:
    dx, dy = b[0] - a[0], b[1] - a[1]
    return abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) <= TOLERANCE * max(abs(dx), abs(dy), 1)


def local_u(p: tuple[int, int], a: tuple[int, int], b: tuple[int, int]) -> float:
    """Физическая длина от общего начала вдоль оси (0,01 мм), включая 45°.

    Ориентация каноническая: все куски одного run используют одну шкалу,
    даже если start/end развёрнуты в разные стороны.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = math.hypot(dx, dy)
    if norm == 0:
        raise ValueError("zero-length component")
    if dx < 0 or (dx == 0 and dy < 0):
        dx, dy = -dx, -dy
    return (p[0] * dx + p[1] * dy) / norm


def components(block: dict) -> list[dict]:
    # Временный Layout: логический node состоит из плеч, хотя start=end.
    return block.get("arms", []) if block.get("arms") else [block]


def check_success(data: dict, input_data: dict, profile: dict) -> list[str]:
    representation = data.get("representation", "course_layout_v1")
    if representation != "course_layout_v1":
        return [f"unsupported result representation {representation}; physical block contract requires separate validator"]
    # Ниже проверяются именно поля временного Layout: cuts left/right и node arms.
    # Их нельзя переносить как требования на будущий physical node output.
    problems: list[str] = []
    blocks = data.get("blocks")
    if not isinstance(blocks, list) or not blocks:
        return ["empty or invalid blocks"]
    if data.get("snapshot_hash") != input_data.get("snapshot_hash"):
        problems.append("result snapshot hash mismatch")
    if data.get("schema_version") != 1:
        problems.append("result schema mismatch")
    if data.get("request_id", "") != input_data.get("request_id", ""):
        problems.append("result request_id mismatch")
    if data.get("diagnostics"):
        problems.append("success contains diagnostics")
    walls = {w["guid"]: w for w in input_data["wall_volumes"]}
    openings = {v["guid"]: v for v in input_data["opening_volumes"]}
    beams = {b["guid"] for b in input_data["beams"]}
    ids: set[str] = set()
    intervals: dict[tuple[int, str], list[tuple[float, float, str]]] = collections.defaultdict(list)
    represented_walls: set[str] = set()
    for block in blocks:
        bid = block.get("id")
        if not isinstance(bid, str) or not bid or bid in ids:
            problems.append(f"missing or duplicate block id {bid}")
        else:
            ids.add(bid)
        course, z = block.get("course_index"), block.get("z_centimm")
        if type(course) is not int or type(z) is not int or z != round(input_data["z0_mm"] * SCALE) + course * COURSE:
            problems.append(f"invalid course/z {bid}")
            continue
        if not bid.startswith(f"c{course}:"):
            problems.append(f"block id/course mismatch {bid}")
        sources = block.get("source_ids")
        if not isinstance(sources, list) or sources != sorted(set(sources)):
            problems.append(f"invalid provenance order/duplicates {bid}")
            continue
        source_walls = []
        for source in sources:
            prefix, _, source_id = source.partition(":")
            if prefix == "wall" and source_id in walls:
                source_walls.append(walls[source_id])
                represented_walls.add(source_id)
            elif prefix == "edge" and source_id:
                pass  # Edge IDs are generated by topology and absent from frozen input.
            elif source in openings and block.get("is_bridge"):
                pass
            else:
                problems.append(f"unknown provenance {source} in {bid}")
        if not source_walls:
            problems.append(f"missing source wall {bid}")
        if block.get("wall_id") != block.get("edge_id") or not isinstance(block.get("edge_id"), str):
            problems.append(f"invalid run identity {bid}")
        kind = block.get("kind")
        is_node = isinstance(kind, str) and kind.startswith("node_")
        bridge = block.get("is_bridge") is True
        if bridge != (kind == "bridge"):
            problems.append(f"bridge kind mismatch {bid}")
        cuts = block.get("cuts")
        if not isinstance(cuts, list) or any(c not in ("left", "right") for c in cuts) or len(cuts) != len(set(cuts)):
            problems.append(f"invalid cuts {bid}")
            continue
        if block.get("hide_spikes_left") != ("left" in cuts) or block.get("hide_spikes_right") != ("right" in cuts):
            problems.append(f"spike/cut mismatch {bid}")
        nominal, length = block.get("catalog_nominal_centimm"), block.get("length_centimm")
        if type(length) is not int or length <= 0:
            problems.append(f"nonpositive length {bid}")
            continue
        if is_node:
            if not block.get("arms") or nominal is not None or cuts or bridge:
                problems.append(f"invalid node fields {bid}")
            if point(block.get("start")) != point(block.get("end")):
                problems.append(f"temporary node anchor mismatch {bid}")
            if length != max((arm.get("length_centimm", 0) for arm in block.get("arms", [])), default=0):
                problems.append(f"node length/arms mismatch {bid}")
        else:
            catalog = profile["bridge_nominal_lengths_centimm"] if bridge else profile["ordinary_nominal_lengths_centimm"]
            if type(nominal) is not int or nominal not in catalog or length > nominal:
                problems.append(f"invalid catalog nominal {bid}")
            if type(nominal) is int and ((length < nominal and not cuts) or (length == nominal and cuts)):
                problems.append(f"cut/nominal mismatch {bid}")
            if not bridge and cuts and length < profile["minimum_cut_centimm"]:
                problems.append(f"cut shorter than profile minimum {bid}")
            if bridge and not any(source in openings for source in sources):
                problems.append(f"bridge without opening provenance {bid}")
            if not bridge and any(source in openings or source in beams for source in sources):
                problems.append(f"ordinary block with opening/beam provenance {bid}")
        for component in components(block):
            a, b = point(component.get("start")), point(component.get("end"))
            clen, edge = component.get("length_centimm"), component.get("edge_id")
            if a is None or b is None or type(clen) is not int or clen <= 0 or not isinstance(edge, str):
                problems.append(f"invalid component geometry {bid}")
                continue
            if math.hypot(b[0] - a[0], b[1] - a[1]) == 0:
                problems.append(f"zero-length component {bid}")
                continue
            if abs(math.hypot(b[0] - a[0], b[1] - a[1]) - clen) > 1:
                problems.append(f"component length mismatch {bid}")
            ca, cb = sorted((local_u(a, a, b), local_u(b, a, b)))
            coverage = []
            for wall in source_walls:
                wa, wb = wall_line(wall)
                if on_line(a, wa, wb) and on_line(b, wa, wb):
                    lo, hi = sorted((local_u(wa, a, b), local_u(wb, a, b)))
                    coverage.append((max(lo, ca), min(hi, cb)))
            cursor = ca
            for lo, hi in sorted(coverage):
                if lo <= cursor + TOLERANCE:
                    cursor = max(cursor, hi)
            if cursor < cb - TOLERANCE:
                problems.append(f"component outside source walls {bid}")
            for opening_id, opening in openings.items():
                oa, ob = wall_line(opening)
                if not (on_line(oa, a, b) and on_line(ob, a, b)):
                    continue
                lo, hi = sorted((local_u(oa, a, b), local_u(ob, a, b)))
                if bridge and opening_id in sources:
                    support = round(profile["lintel_support_mm"] * SCALE)
                    if ca > lo - support + TOLERANCE or cb < hi + support - TOLERANCE:
                        problems.append(f"bridge lacks supports {bid} opening={opening_id}")
                elif not bridge and kind == "ordinary":
                    top = max(opening["startTopZmm"], opening["endTopZmm"]) * SCALE
                    bottom = min(opening["startBottomZmm"], opening["endBottomZmm"]) * SCALE
                    if z < top and z + COURSE > bottom and ca < hi - TOLERANCE and lo < cb - TOLERANCE:
                        problems.append(f"ordinary block intrudes opening {bid} opening={opening_id}")
            if kind == "ordinary":
                for beam in input_data["beams"]:
                    ba, bb = wall_line(beam)
                    if not (on_line(ba, a, b) and on_line(bb, a, b)):
                        continue
                    lo, hi = sorted((local_u(ba, a, b), local_u(bb, a, b)))
                    bottom = min(beam["startZmm"], beam["endZmm"]) * SCALE
                    top = max(beam["startZmm"], beam["endZmm"]) * SCALE + beam["geometry"]["heightMm"] * SCALE
                    if z < top and z + COURSE > bottom and ca < hi - TOLERANCE and lo < cb - TOLERANCE:
                        problems.append(f"ordinary block intrudes longitudinal beam {bid} beam={beam['guid']}")
            intervals[(course, edge)].append((ca, cb, bid))
    for key, pieces in intervals.items():
        pieces.sort()
        for left, right in zip(pieces, pieces[1:]):
            if right[0] < left[1] - TOLERANCE:
                problems.append(f"collision course/run {key}: {left[2]} / {right[2]}")
    for wall_id in sorted(walls.keys() - represented_walls):
        problems.append(f"source wall has no block coverage {wall_id}")
    return problems


def canonical_digest(data: dict) -> str:
    stable = {key: value for key, value in data.items() if key != "elapsed_ms"}
    payload = json.dumps(stable, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def run_once(name: str, binary: Path, profile: Path, input_data: dict) -> tuple[dict | None, int, float, str]:
    with tempfile.TemporaryDirectory(prefix=f"fb-layout-{name}-") as temp_name:
        temp = Path(temp_name)
        request = temp / "request.json"
        result = temp / "result.json"
        request.write_text(json.dumps(input_data, ensure_ascii=False), encoding="utf-8")
        started = time.perf_counter()
        try:
            process = subprocess.run(
                [str(binary), "--request", str(request), "--profile", str(profile),
                 "--result", str(result)], capture_output=True, text=True,
                encoding="utf-8", timeout=120, check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            return None, -1, round((time.perf_counter() - started) * 1000, 2), str(error)
        wall_ms = round((time.perf_counter() - started) * 1000, 2)
        if not result.exists():
            return None, process.returncode, wall_ms, process.stderr
        try:
            data = json.loads(result.read_text(encoding="utf-8"))
        except (UnicodeError, json.JSONDecodeError) as error:
            return None, process.returncode, wall_ms, f"invalid result JSON: {error}; {process.stderr}"
        if not isinstance(data, dict):
            return None, process.returncode, wall_ms, f"result must be JSON object; {process.stderr}"
        return data, process.returncode, wall_ms, process.stderr


def run_project(name: str, binary: Path, profile_path: Path, profile: dict, max_wall_ms: float) -> dict:
    input_data = read_fixture(FIXTURES / name / "input.json.gz")
    observed = read_fixture(FIXTURES / name / "observed.json.gz")
    problems = check_snapshot(input_data, observed)
    first = run_once(name, binary, profile_path, input_data)
    second = run_once(name, binary, profile_path, input_data)
    for index, (data, code, duration, error) in enumerate((first, second), 1):
        if data is None:
            problems.append(f"run {index}: no valid result: exit={code} stderr={error}")
        elif data.get("status") != "success" or code != 0:
            problems.append(f"run {index}: calculation failed status={data.get('status')} exit={code}")
        if data is not None:
            if data.get("snapshot_hash") != input_data.get("snapshot_hash") or data.get("schema_version") != 1:
                problems.append(f"run {index}: result identity mismatch")
            if data.get("status") == "failure" and (not data.get("diagnostics") or data.get("blocks")):
                problems.append(f"run {index}: malformed failure response")
        if duration > max_wall_ms:
            problems.append(f"run {index}: wall time {duration} ms exceeds {max_wall_ms} ms")
    data = first[0]
    if data is not None and second[0] is not None and canonical_digest(data) != canonical_digest(second[0]):
        problems.append("nondeterministic result (excluding elapsed_ms)")
    if data is not None and data.get("status") == "success" and first[1] == 0:
        problems.extend(check_success(data, input_data, profile))
    diagnostics = data.get("diagnostics", []) if isinstance(data, dict) else []
    second_diagnostics = second[0].get("diagnostics", []) if isinstance(second[0], dict) else []
    if not isinstance(diagnostics, list):
        problems.append("invalid diagnostics array")
        diagnostics = []
    if not isinstance(second_diagnostics, list):
        problems.append("second run invalid diagnostics array")
        second_diagnostics = []
    block_count = len(data["blocks"]) if isinstance(data, dict) and isinstance(data.get("blocks"), list) else 0
    return {"project": name, "status": data.get("status") if isinstance(data, dict) else "no_result",
            "representation": data.get("representation", "course_layout_v1") if isinstance(data, dict) else None,
            "wall_ms": [first[2], second[2]],
            "engine_ms": data.get("elapsed_ms") if isinstance(data, dict) else None,
            "computed_blocks": block_count,
            "observed_blocks_characterization": len(observed["observed_blocks"]),
            "result_sha256": canonical_digest(data) if isinstance(data, dict) else None,
            "diagnostic_counts": dict(collections.Counter(d.get("code", "UNKNOWN") if isinstance(d, dict) else "INVALID"
                                                           for d in diagnostics)),
            "diagnostics": diagnostics, "second_run_diagnostics": second_diagnostics,
            "problems": problems}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "target" / "release" / "fb-layout.exe")
    parser.add_argument("--profile", type=Path, default=ROOT / "profiles" / "legacy-observed-study.json")
    parser.add_argument("--max-wall-ms", type=float, default=30_000)
    args = parser.parse_args()
    if not math.isfinite(args.max_wall_ms) or args.max_wall_ms <= 0:
        parser.error("--max-wall-ms must be positive and finite")
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    results = [run_project(name, args.binary, args.profile, profile, args.max_wall_ms) for name in PROJECTS]
    for item in results:
        print(json.dumps(item, ensure_ascii=False))
    return 0 if all(not item["problems"] for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

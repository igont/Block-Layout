"""Проверки самого независимого валидатора frozen snapshots."""

import copy
import importlib.util
import math
import unittest
from pathlib import Path
from unittest.mock import patch


MODULE = Path(__file__).resolve().parents[1] / "tools" / "validate_leninskoe.py"
SPEC = importlib.util.spec_from_file_location("validate_leninskoe", MODULE)
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


def fixture():
    wall = {"guid": "wall-1", "startXmm": 0, "startYmm": 0,
            "endXmm": 1000, "endYmm": 0}
    opening = {"guid": "opening-1", "startXmm": 300, "startYmm": 0,
               "endXmm": 500, "endYmm": 0,
               "startBottomZmm": 0, "endBottomZmm": 0,
               "startTopZmm": 63, "endTopZmm": 63}
    input_data = {"schema_version": 1, "snapshot_hash": "a" * 64, "z0_mm": 0,
                  "wall_volumes": [wall], "opening_volumes": [opening], "beams": []}
    profile = {"ordinary_nominal_lengths_centimm": [30000, 64000],
               "bridge_nominal_lengths_centimm": [64000], "lintel_support_mm": 200,
               "minimum_cut_centimm": 1000}
    def block(name, start, end, course, kind, sources, nominal, cuts=None):
        cuts = cuts or []
        return {"id": name, "wall_id": "run-1", "edge_id": "run-1",
                "course_index": course, "z_centimm": course * 6300,
                "start": {"x": start, "y": 0}, "end": {"x": end, "y": 0},
                "length_centimm": end-start, "kind": kind, "is_bridge": kind == "bridge",
                "hide_spikes_left": "left" in cuts, "hide_spikes_right": "right" in cuts,
                "cuts": cuts, "source_ids": sorted(sources),
                "catalog_nominal_centimm": nominal, "arms": []}
    left = block("c0:left", 0, 30000, 0, "ordinary", ["wall:wall-1", "edge:e1"], 30000)
    right = block("c0:right", 50000, 100000, 0, "ordinary", ["wall:wall-1", "edge:e1"], 64000, ["right"])
    bridge = block("c1:bridge", 10000, 70000, 1, "bridge",
                   ["wall:wall-1", "edge:e1", "opening-1"], 64000, ["right"])
    data = {"status": "success", "schema_version": 1, "request_id": "",
            "snapshot_hash": "a" * 64, "blocks": [left, right, bridge]}
    return data, input_data, profile


class ValidatorTest(unittest.TestCase):
    def test_valid_geometry(self):
        data, inputs, profile = fixture()
        self.assertEqual([], validator.check_success(data, inputs, profile))

    def test_collision_and_unknown_provenance(self):
        data, inputs, profile = fixture()
        bad = copy.deepcopy(data["blocks"][0])
        bad["id"] = "overlap"
        data["blocks"].append(bad)
        data["blocks"][1]["source_ids"].append("wall:missing")
        data["blocks"][1]["source_ids"].sort()
        problems = validator.check_success(data, inputs, profile)
        self.assertTrue(any("collision" in p for p in problems))
        self.assertTrue(any("unknown provenance" in p for p in problems))

    def test_opening_intrusion_and_bridge_support(self):
        data, inputs, profile = fixture()
        data["blocks"][0]["end"]["x"] = 40000
        data["blocks"][0]["length_centimm"] = 40000
        data["blocks"][0]["catalog_nominal_centimm"] = 64000
        data["blocks"][0]["cuts"] = ["right"]
        data["blocks"][0]["hide_spikes_right"] = True
        data["blocks"][2]["start"]["x"] = 20000
        data["blocks"][2]["length_centimm"] = 50000
        problems = validator.check_success(data, inputs, profile)
        self.assertTrue(any("intrudes opening" in p for p in problems))
        self.assertTrue(any("lacks supports" in p for p in problems))

    def test_diagonal_length_and_support_use_true_distance(self):
        data, inputs, profile = fixture()
        inputs["wall_volumes"][0]["endYmm"] = 1000
        opening = inputs["opening_volumes"][0]
        opening["startYmm"], opening["endYmm"] = 300, 500
        diagonal = data["blocks"][0]
        diagonal["end"] = {"x": 30000, "y": 30000}
        diagonal["length_centimm"] = round(math.hypot(30000, 30000))
        diagonal["catalog_nominal_centimm"] = diagonal["length_centimm"]
        profile["ordinary_nominal_lengths_centimm"] = [diagonal["length_centimm"]]
        bridge = data["blocks"][2]
        bridge["start"] = {"x": 15858, "y": 15858}
        bridge["end"] = {"x": 64142, "y": 64142}
        bridge["length_centimm"] = round(math.hypot(48284, 48284))
        bridge["catalog_nominal_centimm"] = 70000
        profile["bridge_nominal_lengths_centimm"] = [70000]
        data["blocks"] = [diagonal, bridge]
        self.assertEqual([], validator.check_success(data, inputs, profile))
        bridge["start"] = {"x": 20000, "y": 20000}
        bridge["length_centimm"] = round(math.hypot(44142, 44142))
        problems = validator.check_success(data, inputs, profile)
        self.assertTrue(any("lacks supports" in p for p in problems))
        diagonal["length_centimm"] = 30000
        problems = validator.check_success(data, inputs, profile)
        self.assertTrue(any("component length mismatch" in p for p in problems))

    def test_digest_ignores_timing_only(self):
        first = {"status": "success", "elapsed_ms": 1, "blocks": [{"id": "a"}]}
        second = {"status": "success", "elapsed_ms": 2, "blocks": [{"id": "a"}]}
        self.assertEqual(validator.canonical_digest(first), validator.canonical_digest(second))
        second["blocks"][0]["id"] = "b"
        self.assertNotEqual(validator.canonical_digest(first), validator.canonical_digest(second))

    def test_physical_representation_requires_own_contract(self):
        data, inputs, profile = fixture()
        data["representation"] = "physical_blocks_v1"
        self.assertTrue(any("unsupported result representation" in problem
                            for problem in validator.check_success(data, inputs, profile)))

    def test_failure_keeps_diagnostics_and_fails(self):
        inputs, observed = {
            "schema_version": 1, "project_name": "demo", "project_id": "p",
            "snapshot_hash": "a" * 64, "z0_mm": 0,
            "metadata": {"sections": {
                "wallVolumes": {"selected_count": 1},
                "openingVolumes": {"selected_count": 0}, "beams": {"selected_count": 0}}},
            "wall_volumes": [{"guid": "wall-1"}], "opening_volumes": [], "beams": []
        }, None
        observed = {key: inputs[key] for key in
                    ("schema_version", "project_name", "project_id", "snapshot_hash", "z0_mm", "metadata")}
        observed["observed_blocks"] = [{"guid": "old-java"}]
        failure = {"status": "failure", "schema_version": 1, "snapshot_hash": "a" * 64,
                   "diagnostics": [{"code": "INVALID_MASK", "message": "ошибка", "source_id": "wall-1"}]}
        with patch.object(validator, "read_fixture", side_effect=[inputs, observed]), \
             patch.object(validator, "run_once", return_value=(failure, 1, 10.0, "")):
            result = validator.run_project("dom", Path("fb-layout"), Path("profile"), {}, 30000)
        self.assertTrue(result["problems"])
        self.assertEqual("failure", result["status"])
        self.assertEqual(1, result["diagnostic_counts"]["INVALID_MASK"])
        self.assertEqual(failure["diagnostics"], result["diagnostics"])
        self.assertEqual(1, result["observed_blocks_characterization"])


if __name__ == "__main__":
    unittest.main()

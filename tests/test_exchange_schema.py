"""Проверка машинного контракта через JSON Schema, отдельно от геометрии ядра."""
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import unittest
import subprocess
import sys
import tempfile

from jsonschema import Draft202012Validator


CONTRACT = Path(__file__).resolve().parents[1] / "Документация" / "Граничные контракты"


def example(name):
    return json.loads((CONTRACT / "examples" / name).read_text(encoding="utf-8"))


class ExchangeSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        schema = json.loads((CONTRACT / "schemas" / "fb-layout-data-v1.schema.json").read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        cls.validator = Draft202012Validator(schema)

    def test_all_document_examples_are_valid(self):
        for name in ["layout-request.v1.json", "layout-result.v1.json",
                     "model-with-beam.v1.json", "layout-failure.v1.json",
                     "layout-result-with-warnings.v1.json", "layout-result-with-convex-cut.v1.json"]:
            with self.subTest(example=name):
                self.validator.validate(example(name))

    def test_warning_only_belongs_to_completed_result(self):
        for name in ["layout-request.v1.json", "layout-failure.v1.json"]:
            with self.subTest(example=name):
                document = example(name)
                document["warnings"] = []
                self.assertFalse(self.validator.is_valid(document))
        document = example("layout-result.v1.json")
        document["warnings"] = []
        self.validator.validate(document)
        document["warnings"] = [{"code": "X", "message": "Нет источников"}]
        self.assertFalse(self.validator.is_valid(document))

    def test_convex_cut_preserves_halfspace_coefficients(self):
        document = example("layout-result-with-convex-cut.v1.json")
        self.validator.validate(document)
        cut = document["blocks"][0]["cuts"][0]
        self.assertEqual(cut["kind"], "convex_cut")
        self.assertEqual(cut["planes_local"][-1], {"normal": [1, 0, 1], "offset_mm": 240})
        # Точка внутри удовлетворяет всем полупространствам; верхняя точка
        # при x=200 исключается именно наклонной плоскостью, а не коробкой.
        inside = [150, 100, 30]
        outside = [200, 100, 60]
        contains = lambda point: all(sum(a*b for a, b in zip(p["normal"], point)) <= p["offset_mm"] for p in cut["planes_local"])
        self.assertTrue(contains(inside))
        self.assertFalse(contains(outside))

    def test_convex_cut_rejects_unknown_geometry_and_wrong_plane_count(self):
        original = example("layout-result-with-convex-cut.v1.json")
        for mutation in ["too_few", "too_many", "unknown_field", "wrong_normal_size", "missing_offset", "no_sources"]:
            with self.subTest(mutation=mutation):
                document = copy.deepcopy(original)
                cut = document["blocks"][0]["cuts"][0]
                if mutation == "too_few":
                    cut["planes_local"] = cut["planes_local"][:3]
                elif mutation == "too_many":
                    cut["planes_local"] = [cut["planes_local"][0]] * 65
                elif mutation == "unknown_field":
                    cut["planes_local"][0]["keep_side"] = "negative"
                elif mutation == "wrong_normal_size":
                    cut["planes_local"][0]["normal"] = [1, 0]
                elif mutation == "missing_offset":
                    del cut["planes_local"][0]["offset_mm"]
                else:
                    cut["source_ids"] = []
                self.assertFalse(self.validator.is_valid(document))

    def test_actual_banya_walls_stage_has_separate_identity_and_preserves_source(self):
        root = CONTRACT.parents[1]
        source = root / "tests" / "fixtures" / "leninskoe" / "banya" / "input.json.gz"
        protected_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        with gzip.open(source, "rt", encoding="utf-8") as stream:
            original = json.load(stream)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "walls-request.v1.json"
            command = [sys.executable, str(root / "tools" / "adapt_layout_fixture.py"),
                       "--input", str(source), "--profile", str(root / "profiles" / "banya-prototype.json"),
                       "--output", str(output), "--walls-only"]
            completed = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", check=True)
            self.assertIn("намеренно исключены", completed.stderr)
            request = json.loads(output.read_text(encoding="utf-8"))
            self.validator.validate(request)
            self.assertEqual(request["model"]["openings"], [])
            self.assertEqual(request["model"]["beams"], [])
            self.assertEqual({wall["id"] for wall in request["model"]["wall_volumes"]},
                             {wall["guid"] for wall in original["wall_volumes"]})
            self.assertEqual(request["coordinate_system"]["coordinate_quantum_mm"], 0.01)
            self.assertEqual(request["coordinate_system"]["z0_mm"], original["z0_mm"])
            self.assertNotEqual(request["snapshot_hash"], original["snapshot_hash"])
            self.assertTrue(request["request_id"].endswith("-walls-only"))
            provenance = json.loads((output.parent / "walls-source.json").read_text(encoding="utf-8"))
            self.assertEqual(provenance["source_snapshot_hash"], original["snapshot_hash"])
            self.assertEqual(provenance["snapshot_hash"], request["snapshot_hash"])
            subprocess.run(command, capture_output=True, text=True, encoding="utf-8", check=True)
            self.assertEqual(json.loads(output.read_text(encoding="utf-8")), request)
            result_path = Path(temporary) / "walls-result.v1.json"
            candidate_path = Path(temporary) / "walls-candidate.json"
            subprocess.run(
                ["cargo", "run", "--quiet", "--", "--request", str(output),
                 "--profile", str(root / "profiles" / "banya-prototype.json"),
                 "--result", str(result_path), "--candidate", str(candidate_path)],
                cwd=root, capture_output=True, text=True, encoding="utf-8", check=True, timeout=180,
            )
            result = json.loads(result_path.read_text(encoding="utf-8"))
            self.validator.validate(result)
            self.assertEqual(result["status"], "success")
            self.assertEqual(result["request_id"], request["request_id"])
            self.assertEqual(result["snapshot_hash"], request["snapshot_hash"])
            blocks = result["blocks"]
            self.assertTrue(blocks)
            self.assertEqual(len({block["id"] for block in blocks}), len(blocks))
            candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
            kinds = {block["kind"] for block in candidate["blocks"]}
            for kind in ["L", "T", "X"]:
                self.assertIn("node_" + kind, kinds, f"Нет узла {kind}")
            self.assertIn("ordinary", kinds)
            self.assertEqual({block["id"] for block in candidate["blocks"]}, {block["id"] for block in blocks})
            excluded_sources = {item["guid"] for item in original["opening_volumes"] + original["beams"]}

            def assert_numbers(values):
                for value in values:
                    if isinstance(value, list):
                        assert_numbers(value)
                    else:
                        self.assertIn(type(value), (int, float))
                        self.assertTrue(math.isfinite(value))

            for block in blocks:
                self.assertNotEqual(block["product"]["category"], "lintel")
                self.assertTrue(set(block["source_ids"]).isdisjoint(excluded_sources))
                self.assertTrue(all(not value.startswith(("opening:", "beam:")) for value in block["source_ids"]))
                shape = block["stock_shape"]
                self.assertIn(shape["kind"], ("box", "mesh"))
                assert_numbers(shape["size_mm"] if shape["kind"] == "box" else shape["vertices_mm"])
                for cut in block["cuts"]:
                    self.assertTrue(set(cut["source_ids"]).isdisjoint(excluded_sources))
                    self.assertIn(cut["kind"], ("plane_cut", "box_cut", "convex_cut"))
                    if cut["kind"] == "plane_cut":
                        assert_numbers([cut["point_local_mm"], cut["normal_local"]])
                    elif cut["kind"] == "box_cut":
                        assert_numbers(cut["size_mm"])
                        assert_numbers(list(cut["frame_local"].values()))
                    else:
                        for plane in cut["planes_local"]:
                            assert_numbers([plane["normal"], plane["offset_mm"]])
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), protected_hash)


if __name__ == "__main__":
    unittest.main()

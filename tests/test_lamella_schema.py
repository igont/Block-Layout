"""Physical lamella contract rejects GDL display flags and off-axis origins."""
import copy
import json
from pathlib import Path
import unittest

from jsonschema import Draft202012Validator


SCHEMA_PATH = Path(__file__).resolve().parents[1] / "Документация" / "Граничные контракты" / "schemas" / "codes-result.v1.schema.json"


class LamellaSchemaTest(unittest.TestCase):
    def setUp(self):
        self.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(self.schema)
        self.lamella = {
            "id": "wall:k0:right:s0", "placement": {
                "center_mm": [320, -89, 31.5],
                "x_axis": [1, 0, 0], "y_axis": [0, 1, 0], "z_axis": [0, 0, 1]},
            "length_mm": 640, "thickness_mm": 15, "height_mm": 63,
            "course_index": 0, "side": "right", "wall_ids": ["wall"],
            "beam_ids": ["beam"], "source_ids": ["wall", "beam"]}
        self.validator = Draft202012Validator({"$defs": self.schema["$defs"], "$ref": "#/$defs/lamella"})

    def test_one_centered_physical_body_and_partial_height(self):
        self.validator.validate(self.lamella)
        self.validator.validate({**self.lamella, "height_mm": 31.5})
        for field, value in [("thickness_mm", 17), ("thickness_mm", 0),
                             ("length_mm", 0), ("height_mm", 64),
                             ("onLeft", True), ("onRight", True)]:
            self.assertFalse(self.validator.is_valid({**self.lamella, field: value}), field)

    def test_requires_center_axes_and_sources(self):
        for field in ["id", "placement", "side", "wall_ids", "beam_ids", "source_ids"]:
            invalid = copy.deepcopy(self.lamella)
            del invalid[field]
            self.assertFalse(self.validator.is_valid(invalid), field)
        for field in ["center_mm", "x_axis", "y_axis", "z_axis"]:
            invalid = copy.deepcopy(self.lamella)
            del invalid["placement"][field]
            self.assertFalse(self.validator.is_valid(invalid), field)
        invalid = copy.deepcopy(self.lamella)
        invalid["placement"]["origin_mm"] = invalid["placement"].pop("center_mm")
        self.assertFalse(self.validator.is_valid(invalid))

    def test_recess_reuses_exact_body_and_requires_instance_reference(self):
        recess = {"kind": "lamella_recess", "lamella_id": self.lamella["id"],
                  **{key: self.lamella[key] for key in ["placement", "length_mm", "thickness_mm", "height_mm"]}}
        validator = Draft202012Validator({"$defs": self.schema["$defs"], "$ref": "#/$defs/trim"})
        validator.validate(recess)
        del recess["lamella_id"]
        self.assertFalse(validator.is_valid(recess))
        array_validator = Draft202012Validator({"$defs": self.schema["$defs"], **self.schema["properties"]["lamellas"]})
        array_validator.validate([])
        array_validator.validate([self.lamella])

    def test_exact_clipped_solid_uses_world_vertices_and_indexed_faces(self):
        solid = {"vertices": [[0, 81.5, 0], [640, 81.5, 0], [0, 81.5, 63],
                              [0, 96.5, 0], [640, 96.5, 0], [0, 96.5, 63]],
                 "faces": [[0, 2, 1], [3, 4, 5], [0, 1, 4, 3], [1, 2, 5, 4], [2, 0, 3, 5]]}
        self.validator.validate({**self.lamella, "solid": solid})
        for invalid in [{"vertices": [], "faces": []},
                        {**solid, "faces": [[0, 0, 1]]},
                        {**solid, "vertices_mm": solid["vertices"]}]:
            self.assertFalse(self.validator.is_valid({**self.lamella, "solid": invalid}))

    def test_lamella_owned_recess_names_saved_blocks_without_replacement_records(self):
        self.validator.validate({**self.lamella, "recess_block_ids": ["saved-half"]})
        for invalid in [[], [""], ["saved-half", "saved-half"], "saved-half"]:
            self.assertFalse(self.validator.is_valid({**self.lamella, "recess_block_ids": invalid}))


class SavedLamellaRequestSchemaTest(unittest.TestCase):
    def setUp(self):
        schema = json.loads(SCHEMA_PATH.with_name("fb-layout-data-v1.schema.json").read_text(encoding="utf-8"))
        self.validator = Draft202012Validator(schema)
        Draft202012Validator.check_schema(schema)
        self.block = {"id": "saved-manual", "placement": {"origin_mm": [420, 0, 0],
                      "x_axis": [1, 0, 0], "y_axis": [0, 1, 0], "z_axis": [0, 0, 1]},
                      "length_mm": 640, "width_mm": 193, "height_mm": 63,
                      "course_index": 0, "wall_ids": [], "solids": [{
                          "vertices": [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
                          "faces": [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]]}]}
        self.request = {"format": "fb-layout/1", "kind": "lamella_layout_request", "request_id": "one",
                        "snapshot_hash": "0" * 64, "coordinate_system": {"id": "world", "units": "mm", "coordinate_quantum_mm": .01, "z0_mm": 0},
                        "profile": {"id": "p", "revision": "1"}, "catalog": {"id": "c", "revision": "1"},
                        "scope": {"mode": "full"}, "model": {"wall_volumes": [], "openings": [], "beams": [], "saved_blocks": [self.block]}}

    def test_independent_modes_and_explicit_saved_blocks(self):
        self.validator.validate(self.request)
        empty = copy.deepcopy(self.request)
        empty["model"]["saved_blocks"] = []
        self.validator.validate(empty)
        del empty["model"]["saved_blocks"]
        self.assertFalse(self.validator.is_valid(empty))
        empty["kind"] = "blocks_layout_request"
        self.validator.validate(empty)
        empty["model"]["saved_blocks"] = []
        self.assertFalse(self.validator.is_valid(empty))

    def test_requires_actual_saved_material_not_only_nominal_box(self):
        for field in ["placement", "length_mm", "course_index", "wall_ids", "solids"]:
            invalid = copy.deepcopy(self.request)
            del invalid["model"]["saved_blocks"][0][field]
            self.assertFalse(self.validator.is_valid(invalid), field)
        invalid = copy.deepcopy(self.request)
        invalid["model"]["saved_blocks"][0]["solids"] = []
        self.assertFalse(self.validator.is_valid(invalid))

    def test_saved_joint_endpoints_are_paired(self):
        invalid = copy.deepcopy(self.request)
        invalid["model"]["saved_blocks"][0]["grid_start_mm"] = [420, 0, 0]
        self.assertFalse(self.validator.is_valid(invalid))
        invalid["model"]["saved_blocks"][0]["grid_end_mm"] = [1060, 0, 0]
        self.validator.validate(invalid)


if __name__ == "__main__":
    unittest.main()

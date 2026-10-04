"""Состояние физического торца и производные признаки в обоих результатах."""
import copy
import json
from pathlib import Path
import unittest

from jsonschema import Draft202012Validator

SCHEMAS = Path(__file__).resolve().parents[1] / "Документация" / "Граничные контракты" / "schemas"


class EndStateSchemaTest(unittest.TestCase):
    def test_cut_has_exact_plane_and_source(self):
        schema = json.loads((SCHEMAS / "codes-result.v1.schema.json").read_text(encoding="utf-8"))
        validator = Draft202012Validator({"$defs": schema["$defs"], "$ref": "#/$defs/endState"})
        valid = {"state": "factory_cut", "cut": {"plane_centimm": 64000,
                 "sources": [{"kind": "opening", "id": "door"}]}}
        validator.validate(valid)
        validator.validate({"state": "factory_uncut"})
        for field, value in [("plane_centimm", 64000.5), ("sources", []),
                             ("sources", [{"kind": "opening"}]),
                             ("sources", [{"kind": "unknown", "id": "door"}])]:
            invalid = copy.deepcopy(valid)
            invalid["cut"][field] = value
            self.assertFalse(validator.is_valid(invalid), invalid)

    def test_projection_cannot_restore_spikes_or_change_end_origin(self):
        for filename in ["codes-result.v1.schema.json", "fb-layout-data-v1.schema.json"]:
            schema = json.loads((SCHEMAS / filename).read_text(encoding="utf-8"))
            Draft202012Validator.check_schema(schema)
            validator = Draft202012Validator({"$defs": schema["$defs"],
                                              "allOf": [rule for rule in schema["$defs"]["block"]["allOf"]
                                                        if "left_end" in rule.get("if", {}).get("required", [])]})
            for state, natural, hidden in [("factory_uncut", True, False),
                                            ("factory_cut", True, True), ("saw_cut", False, True)]:
                value = {"left_end": {"state": state}, "natural_end_left": natural}
                if state != "factory_uncut":
                    value["left_end"]["cut"] = {"plane_centimm": 0, "sources": [{"kind": "stock"}]}
                if filename.startswith("codes"):
                    value["hide_spikes_left"] = hidden
                else:
                    value["spikes_removed"] = {"left": hidden}
                validator.validate(value)
                invalid = copy.deepcopy(value)
                if filename.startswith("codes"):
                    invalid["hide_spikes_left"] = not hidden
                else:
                    invalid["spikes_removed"]["left"] = not hidden
                self.assertFalse(validator.is_valid(invalid), (filename, invalid))
                invalid = copy.deepcopy(value)
                invalid["natural_end_left"] = not natural
                self.assertFalse(validator.is_valid(invalid), (filename, invalid))


if __name__ == "__main__":
    unittest.main()

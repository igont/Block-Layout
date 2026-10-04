"""Virtual sections are replacements, not additive face adjustments."""
import copy
import json
from pathlib import Path
import unittest
from jsonschema import Draft202012Validator

SCHEMAS = Path(__file__).resolve().parents[1] / "Документация" / "Граничные контракты" / "schemas"

class BeamReplacementSchemaTest(unittest.TestCase):
    def test_replacement_requires_bottom_and_axis_and_rejects_additive_fields(self):
        valid = {"beam_id":"beam", "rule_id":"horizontal_beam_320_to_315",
                 "original_height_mm":320, "height_mm":315, "start_z_mm":315, "end_z_mm":315,
                 "bottom_start_z_mm":0, "bottom_end_z_mm":0, "height_direction":[0,0,-1]}
        for filename in ["codes-result.v1.schema.json", "fb-layout-data-v1.schema.json"]:
            schema = json.loads((SCHEMAS / filename).read_text(encoding="utf-8"))
            Draft202012Validator.check_schema(schema)
            validator = Draft202012Validator({"$defs":schema["$defs"], **schema["properties"]["beam_adjustments"]})
            validator.validate([valid])
            validator.validate([{**valid, "original_height_mm":319.999}])
            validator.validate([{**valid, "original_height_mm":319.995}])
            for height in [319.994,320.005,320.006]:
                self.assertFalse(validator.is_valid([{**valid, "original_height_mm":height}]))
            for field in ["bottom_start_z_mm", "start_z_mm", "height_direction"]:
                invalid = copy.deepcopy(valid)
                del invalid[field]
                self.assertFalse(validator.is_valid([invalid]))
            for field, value in [("height_mm",320), ("height_direction",[0,0,0.8]), ("side","height_positive")]:
                invalid = {**valid, field:value}
                self.assertFalse(validator.is_valid([invalid]))

if __name__ == "__main__":
    unittest.main()

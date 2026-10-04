"""Контракт происхождения доборов и совместимость прежнего обмена."""
import copy
import json
from pathlib import Path
import unittest

from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parents[1]
SCHEMAS = ROOT / "Документация" / "Граничные контракты" / "schemas"


class OffcutSchemaTest(unittest.TestCase):
    def setUp(self):
        self.schemas = [json.loads((SCHEMAS / name).read_text(encoding="utf-8"))
                        for name in ["codes-result.v1.schema.json", "fb-layout-data-v1.schema.json"]]
        self.plan = {"stock_code": "П640", "stock_length_mm": 640, "kerf_mm": 5,
                     "parts": [{"block_id": "parent", "interval_mm": [0, 315], "stock_end": "left"},
                               {"block_id": "child", "interval_mm": [325, 640], "stock_end": "right"}],
                     "saw_cuts_mm": [[315, 320], [320, 325]], "waste_intervals_mm": []}

    def test_both_contracts_share_the_same_stock_definitions(self):
        for name in ["stockInterval", "stockPart", "stockCutPlan", "doborGroup"]:
            self.assertEqual(self.schemas[0]["$defs"][name], self.schemas[1]["$defs"][name])
        for schema in self.schemas:
            Draft202012Validator.check_schema(schema)
            validator = Draft202012Validator({"$defs": schema["$defs"], "$ref": "#/$defs/stockCutPlan"})
            validator.validate(self.plan)
            for field, value in [("kerf_mm", 4), ("stock_length_mm", 650),
                                 ("parts", self.plan["parts"][:1]), ("unknown", True)]:
                plan = copy.deepcopy(self.plan)
                plan[field] = value
                self.assertFalse(validator.is_valid(plan), field)

    def test_child_link_and_parent_plan_are_consistent(self):
        # Здесь проверяется оболочка метаданных; геометрия и баланс материала
        # проверяются отдельными Rust- и Java-сценариями.
        for schema in self.schemas:
            constraints = schema["$defs"]["block"]["allOf"][:2]
            validator = Draft202012Validator({"$defs": schema["$defs"], "allOf": constraints})
            for valid in [{}, {"is_dobor": False, "cut_from_block_id": None},
                          {"is_dobor": True, "cut_from_block_id": "parent", "cut_zone_ids": ["window"]},
                          {"is_dobor": True, "cut_from_block_id": None, "cut_zone_ids": ["window"],
                           "dobor_group": {"source_id": "window", "side": "positive"}},
                          {"is_dobor": False, "cut_from_block_id": None,
                           "cut_zone_ids": ["window"], "stock_cut_plan": self.plan}]:
                validator.validate(valid)
            for invalid in [{"is_dobor": True}, {"is_dobor": True, "cut_from_block_id": None},
                            {"is_dobor": False, "cut_from_block_id": "parent"},
                            {"is_dobor": False, "dobor_group": {"source_id": "window", "side": "positive"}},
                            {"is_dobor": True, "cut_from_block_id": None, "cut_zone_ids": ["window"],
                             "dobor_group": {"source_id": "window", "side": "unknown"}},
                            {"is_dobor": True, "cut_from_block_id": None, "cut_zone_ids": ["window"],
                             "dobor_group": {"source_id": "window"}},
                            {"stock_cut_plan": self.plan},
                            {"is_dobor": True, "cut_from_block_id": "parent",
                             "cut_zone_ids": ["window"], "stock_cut_plan": self.plan}]:
                self.assertFalse(validator.is_valid(invalid), invalid)


if __name__ == "__main__":
    unittest.main()

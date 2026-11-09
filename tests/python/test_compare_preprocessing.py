import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/compare_preprocessing.py"
spec = importlib.util.spec_from_file_location("compare_preprocessing", SCRIPT)
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


def output(**changes):
    result = {key: 0.1 for key in comparison.METRICS}
    result.update(has_ground_truth=True, matched_points=100,
                  settings={"num_disparities": 160, "preprocessing": {"chain": []}})
    result.update(changes)
    return SimpleNamespace(returncode=0, stdout=json.dumps(result), stderr="")


class ComparePreprocessingTest(unittest.TestCase):
    def test_same_matcher_flags_and_baseline_has_no_config(self):
        with patch.object(comparison.subprocess, "run", side_effect=[
                output(mae_3d_m=0.002), output(mae_3d_m=0.001)]) as run:
            rows = comparison.compare(Path("bench"), [Path("kf")],
                                      [("clahe", Path("clahe.yml"))],
                                      ["--speckle-window=200"])
        calls = [call.args[0] for call in run.call_args_list]
        self.assertEqual(calls[0], ["bench", "--speckle-window=200", "kf"])
        self.assertIn("--preprocessing-config=clahe.yml", calls[1])
        self.assertEqual(rows[1]["delta_vs_none"]["mae_3d_m"], -0.001)

    def test_missing_gt_and_zero_scored_points_fail(self):
        for proc in [output(has_ground_truth=False), output(matched_points=0)]:
            with patch.object(comparison.subprocess, "run", return_value=proc):
                result = comparison.run_case("bench", "kf", None, [])
            self.assertEqual(result["status"], "failed")

    def test_changed_matcher_settings_invalidate_comparison(self):
        with patch.object(comparison.subprocess, "run", side_effect=[
                output(), output(settings={"num_disparities": 192})]):
            rows = comparison.compare("bench", ["kf"], [("variant", "x.yml")], [])
        self.assertEqual(rows[1]["status"], "failed")
        self.assertNotIn("delta_vs_none", rows[1])

    def test_benchmark_failure_is_not_a_zero_error(self):
        failed = SimpleNamespace(returncode=1, stderr="broken image", stdout="")
        with patch.object(comparison.subprocess, "run", side_effect=[failed, output()]):
            rows = comparison.compare("bench", ["kf"], [("variant", "x.yml")], [])
        self.assertEqual(rows[0]["error"], "broken image")
        self.assertEqual(rows[1]["status"], "failed")
        self.assertNotIn("delta_vs_none", rows[1])

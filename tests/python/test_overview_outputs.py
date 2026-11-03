"""Unit tests for the table writers of scripts/scared_overview.py.

The tests feed canned benchmark JSON to the writers and never run the
benchmark binary. XLSX tests are skipped when openpyxl is not importable.

Run:  python3 -m unittest discover -s tests/python -v
"""
import contextlib
import csv
import importlib.util
import io
import os
import sys
import tempfile
import unittest
from unittest import mock

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(REPO, "scripts", "scared_overview.py")


def load_module():
    spec = importlib.util.spec_from_file_location("scared_overview", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ov = load_module()

try:
    import openpyxl  # noqa: F401
    HAVE_OPENPYXL = True
except ImportError:
    HAVE_OPENPYXL = False


GT_JSON = {
    "keyframe_dir": "datasets/scared/dataset_1/keyframe_1",
    "algorithm": "StereoSGBM",
    "num_disparities": 160,
    "min_depth_m": 0.02,
    "max_depth_m": 0.3,
    "image_width": 1280,
    "image_height": 1024,
    "valid_points": 1103257,
    "valid_fraction": 0.84172,
    "z_min_m": 0.0402,
    "z_p05_m": 0.05213,
    "z_median_m": 0.05912,
    "z_p95_m": 0.07401,
    "z_max_m": 0.2999,
    "matching_ms": 328.4567,
    "reconstruction_ms": 12.3456,
    "has_ground_truth": True,
    "ground_truth_points": 900000,
    "matched_points": 769500,
    "coverage": 0.855,
    "mae_x_m": 0.0003,
    "mae_y_m": 0.0003,
    "mae_z_m": 0.00071234,
    "mae_3d_m": 0.00084321,
    "rmse_3d_m": 0.001432,
    "median_3d_m": 0.000591,
    "within_1mm": 0.6271,
    "within_2mm": 0.7993,
    "within_5mm": 0.8462,
    # Diagnostics (group 2 of improve-reconstruction-quality).
    "bias_z_median_m": 0.00016693,
    "fit_disparity_offset_px": 0.22153493,
    "fit_disparity_offset_residual_m": 0.00087932,
    "fit_depth_scale": 1.00282693,
    "fit_depth_scale_residual_m": 0.00093904,
    "tail_over_5mm": 0.01042796,
    "tail_over_20mm": 0.00043359,
    "tail_over_50mm": 1.7475e-05,
    "max_3d_m": 0.13629868,
    "coverage_loss": {
        "out_of_search_range": 0.0012345,
        "image_border": 0.06046776,
        "matcher_rejected": 0.08494662,
        "depth_range": 0.0,
        "outlier_filter": 0.0,
    },
    "unscored_fraction": 0.17010863,
    "depth_bias": [
        {"lo_m": 0.05, "hi_m": 0.06, "count": 300000,
         "bias_z_median_m": 0.0001, "abs_3d_median_m": 0.0005},
    ],
    "settings": {"disparity_offset_px": 0.0, "speckle_window_size": 100},
}

NOGT_JSON = {
    "keyframe_dir": "datasets/scared/test_dataset_8/keyframe_0",
    "algorithm": "StereoSGBM",
    "num_disparities": 160,
    "min_depth_m": 0.02,
    "max_depth_m": 0.3,
    "image_width": 1280,
    "image_height": 1024,
    "valid_points": 1094000,
    "valid_fraction": 0.8349,
    "z_min_m": 0.05,
    "z_p05_m": 0.1011,
    "z_median_m": 0.1153,
    "z_p95_m": 0.1352,
    "z_max_m": 0.29,
    "matching_ms": 314.2,
    "reconstruction_ms": 11.9,
    "has_ground_truth": False,
    "settings": {"disparity_offset_px": 0.0, "speckle_window_size": 100},
}

RESULTS = [
    ov.Result("dataset_1/keyframe_1", "ok", GT_JSON),
    ov.Result("test_dataset_8/keyframe_0", "ok", NOGT_JSON),
    ov.Result("dataset_9/keyframe_3", "failed", None, "Invalid option: boom"),
]

EXPECTED_HEADERS = [
    "keyframe", "status", "has_ground_truth", "valid_points_pct",
    "depth_p5_mm", "depth_median_mm", "depth_p95_mm",
    "matching_ms", "reconstruction_ms", "coverage_pct",
    "mae_3d_mm", "mae_z_mm", "rmse_3d_mm", "median_3d_mm",
    "within_1mm_pct", "within_2mm_pct", "within_5mm_pct",
    # Diagnostics columns, appended in the spec's order.
    "bias_z_median_mm", "fit_disparity_offset_px",
    "tail_over_5mm_pct", "tail_over_20mm_pct", "max_3d_mm",
    "lost_search_range_pct", "lost_matcher_pct",
]

# Accuracy, bias, tail and loss columns: empty without ground truth.
ACCURACY_HEADERS = EXPECTED_HEADERS[9:]
DIAGNOSTIC_HEADERS = EXPECTED_HEADERS[17:]
METRIC_HEADERS = EXPECTED_HEADERS[3:]

# Expected values of the diagnostics columns for GT_JSON (mm / px / %).
EXPECTED_DIAGNOSTICS = {
    "bias_z_median_mm": 0.1669,
    "fit_disparity_offset_px": 0.2215,
    "tail_over_5mm_pct": 1.043,
    "tail_over_20mm_pct": 0.043,
    "max_3d_mm": 136.2987,
    "lost_search_range_pct": 0.123,
    "lost_matcher_pct": 8.495,
}


class ColumnsTest(unittest.TestCase):
    def test_column_order(self):
        self.assertEqual(ov.HEADERS, EXPECTED_HEADERS)

    def test_row_units(self):
        row = dict(zip(ov.HEADERS, ov.table_row(RESULTS[0])))
        self.assertEqual(row["status"], "ok")
        self.assertIs(row["has_ground_truth"], True)
        self.assertAlmostEqual(row["depth_median_mm"], 59.12, places=6)
        self.assertAlmostEqual(row["mae_3d_mm"], 0.8432, places=6)
        self.assertAlmostEqual(row["mae_z_mm"], 0.7123, places=6)
        self.assertAlmostEqual(row["valid_points_pct"], 84.172, places=6)
        self.assertAlmostEqual(row["within_1mm_pct"], 62.71, places=6)
        self.assertAlmostEqual(row["matching_ms"], 328.457, places=6)
        for h, expected in EXPECTED_DIAGNOSTICS.items():
            self.assertAlmostEqual(row[h], expected, places=6, msg=h)

    def test_diagnostics_columns_are_last_in_spec_order(self):
        self.assertEqual(ov.HEADERS[-7:], DIAGNOSTIC_HEADERS)

    def test_no_gt_row_has_empty_diagnostics(self):
        row = dict(zip(ov.HEADERS, ov.table_row(RESULTS[1])))
        self.assertEqual(row["status"], "ok")
        self.assertIs(row["has_ground_truth"], False)
        for h in DIAGNOSTIC_HEADERS:
            self.assertIsNone(row[h], h)

    def test_partial_coverage_loss_is_empty_not_zero(self):
        """A missing coverage_loss key or sub-key gives None, never 0."""
        j = dict(GT_JSON)
        j["coverage_loss"] = {"image_border": 0.06}
        row = dict(zip(ov.HEADERS, ov.table_row(ov.Result("x/y", "ok", j))))
        self.assertIsNone(row["lost_search_range_pct"])
        self.assertIsNone(row["lost_matcher_pct"])
        del j["coverage_loss"]
        row = dict(zip(ov.HEADERS, ov.table_row(ov.Result("x/y", "ok", j))))
        self.assertIsNone(row["lost_matcher_pct"])

    def test_failed_row_is_empty(self):
        row = dict(zip(ov.HEADERS, ov.table_row(RESULTS[2])))
        self.assertEqual(row["keyframe"], "dataset_9/keyframe_3")
        self.assertEqual(row["status"], "failed")
        for h in ["has_ground_truth"] + METRIC_HEADERS:
            self.assertIsNone(row[h], h)

    def test_failed_result_carries_error(self):
        self.assertEqual(RESULTS[2].error, "Invalid option: boom")
        self.assertIsNone(RESULTS[0].error)
        # The three-field form still works for older callers.
        self.assertIsNone(ov.Result("a/b", "failed", None).error)


class MarkdownTest(unittest.TestCase):
    def test_markdown_format(self):
        out = io.StringIO()
        ov.write_markdown(RESULTS, out)
        lines = out.getvalue().splitlines()
        self.assertEqual(len(lines), 5)
        self.assertTrue(lines[0].startswith("| keyframe | valid % |"))
        self.assertEqual(lines[1], "|---|---|---|---|---|---|---|---|---|---|---|---|")
        self.assertEqual(
            lines[2],
            "| dataset_1/keyframe_1 | 84.2 | 52 / 59 / 74 | 328 | yes | 85.5 "
            "| 0.84 | 1.43 | 0.59 | 62.7 | 79.9 | 84.6 |")
        self.assertEqual(
            lines[3],
            "| test_dataset_8/keyframe_0 | 83.5 | 101 / 115 / 135 | 314 | no "
            "|  |  |  |  |  |  |  |")
        self.assertEqual(lines[4], "| dataset_9/keyframe_3 | failed | | | | | | | | | | |")


class CsvTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "m.csv")
        ov.write_csv(RESULTS, self.path)
        with open(self.path, newline="", encoding="utf-8") as f:
            self.rows = list(csv.reader(f))
        self.header = self.rows[0]
        self.by_name = [dict(zip(self.header, r)) for r in self.rows[1:]]

    def tearDown(self):
        self.tmp.cleanup()

    def test_header_and_row_count(self):
        self.assertEqual(self.header, EXPECTED_HEADERS)
        self.assertEqual(len(self.rows), 4)
        for r in self.rows:
            self.assertEqual(len(r), len(EXPECTED_HEADERS))

    def test_rfc4180_crlf_utf8(self):
        with open(self.path, "rb") as f:
            raw = f.read()
        raw.decode("utf-8")
        self.assertIn(b"\r\n", raw)
        self.assertNotIn(b"\n\n", raw)
        self.assertTrue(raw.endswith(b"\r\n"))

    def test_gt_row_units(self):
        row = self.by_name[0]
        self.assertEqual(row["keyframe"], "dataset_1/keyframe_1")
        self.assertEqual(row["status"], "ok")
        self.assertEqual(row["has_ground_truth"], "true")
        self.assertEqual(float(row["valid_points_pct"]), 84.172)
        self.assertEqual(float(row["depth_p5_mm"]), 52.13)
        self.assertEqual(float(row["depth_median_mm"]), 59.12)
        self.assertEqual(float(row["depth_p95_mm"]), 74.01)
        self.assertEqual(float(row["matching_ms"]), 328.457)
        self.assertEqual(float(row["reconstruction_ms"]), 12.346)
        self.assertEqual(float(row["coverage_pct"]), 85.5)
        self.assertEqual(float(row["mae_3d_mm"]), 0.8432)
        self.assertEqual(float(row["mae_z_mm"]), 0.7123)
        self.assertEqual(float(row["rmse_3d_mm"]), 1.432)
        self.assertEqual(float(row["median_3d_mm"]), 0.591)
        self.assertEqual(float(row["within_1mm_pct"]), 62.71)
        self.assertEqual(float(row["within_2mm_pct"]), 79.93)
        self.assertEqual(float(row["within_5mm_pct"]), 84.62)
        for h, expected in EXPECTED_DIAGNOSTICS.items():
            self.assertEqual(float(row[h]), expected, h)
        for h in METRIC_HEADERS:
            self.assertNotIn(",", row[h])

    def test_no_gt_row_has_empty_accuracy(self):
        row = self.by_name[1]
        self.assertEqual(row["status"], "ok")
        self.assertEqual(row["has_ground_truth"], "false")
        self.assertEqual(float(row["depth_median_mm"]), 115.3)
        self.assertEqual(float(row["matching_ms"]), 314.2)
        for h in ACCURACY_HEADERS:
            self.assertEqual(row[h], "", h)

    def test_failed_row_is_empty(self):
        row = self.by_name[2]
        self.assertEqual(row["keyframe"], "dataset_9/keyframe_3")
        self.assertEqual(row["status"], "failed")
        for h in ["has_ground_truth"] + METRIC_HEADERS:
            self.assertEqual(row[h], "", h)


def fake_args(**overrides):
    base = dict(bench="build-opencv/src/cloud_point/scared_dataset_benchmark",
                disparities=160, png_dir=None, json_dir=None,
                depth_range=[0.02, 0.3], csv=None, xlsx=None, bench_args=[],
                keyframes=["datasets/scared/dataset_1/keyframe_1",
                           "datasets/scared/test_dataset_8/keyframe_0"])
    base.update(overrides)
    return mock.Mock(**base)


@unittest.skipUnless(HAVE_OPENPYXL, "openpyxl not installed")
class XlsxTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "m.xlsx")
        self.info = ov.run_info(fake_args(), RESULTS)
        ov.write_xlsx(RESULTS, self.path, self.info)
        self.wb = openpyxl.load_workbook(self.path)

    def tearDown(self):
        self.wb.close()
        self.tmp.cleanup()

    def test_sheets(self):
        self.assertEqual(self.wb.sheetnames, ["metrics", "run"])

    def test_metrics_header_and_numbers(self):
        ws = self.wb["metrics"]
        header = [c.value for c in ws[1]]
        self.assertEqual(header, EXPECTED_HEADERS)
        self.assertEqual(ws.max_row, 4)
        gt = dict(zip(header, (c.value for c in ws[2])))
        self.assertEqual(gt["status"], "ok")
        self.assertIs(gt["has_ground_truth"], True)
        for h in METRIC_HEADERS:
            self.assertIsInstance(gt[h], (int, float), h)
            self.assertNotIsInstance(gt[h], bool, h)
        self.assertAlmostEqual(gt["mae_3d_mm"], 0.8432)
        self.assertAlmostEqual(gt["within_1mm_pct"], 62.71)
        for h, expected in EXPECTED_DIAGNOSTICS.items():
            self.assertAlmostEqual(gt[h], expected, msg=h)
        self.assertEqual(ws.cell(row=2, column=header.index("mae_3d_mm") + 1).number_format, "0.00")
        self.assertEqual(ws.cell(row=2, column=header.index("coverage_pct") + 1).number_format, "0.0")
        self.assertEqual(ws.cell(row=2, column=header.index("fit_disparity_offset_px") + 1).number_format, "0.0000")
        self.assertEqual(ws.cell(row=2, column=header.index("lost_matcher_pct") + 1).number_format, "0.00")

    def test_empty_cells(self):
        ws = self.wb["metrics"]
        header = [c.value for c in ws[1]]
        nogt = dict(zip(header, (c.value for c in ws[3])))
        self.assertIs(nogt["has_ground_truth"], False)
        for h in ACCURACY_HEADERS:
            self.assertIsNone(nogt[h], h)
        failed = dict(zip(header, (c.value for c in ws[4])))
        self.assertEqual(failed["status"], "failed")
        for h in ["has_ground_truth"] + METRIC_HEADERS:
            self.assertIsNone(failed[h], h)

    def test_frozen_header_and_autofilter(self):
        ws = self.wb["metrics"]
        self.assertEqual(ws.freeze_panes, "A2")
        self.assertEqual(ws.auto_filter.ref, "A1:X4")

    def test_run_sheet(self):
        ws = self.wb["run"]
        rows = [(r[0].value, r[1].value) for r in ws.iter_rows(min_row=2)]
        keys = [k for k, _ in rows]
        for k in ["num_disparities", "min_depth_m", "max_depth_m",
                  "timestamp_utc", "git_commit", "benchmark"]:
            self.assertIn(k, keys)
        d = dict(rows)
        self.assertEqual(d["num_disparities"], 160)
        self.assertEqual(d["min_depth_m"], 0.02)
        self.assertEqual(d["max_depth_m"], 0.3)
        self.assertEqual(d["benchmark"], "build-opencv/src/cloud_point/scared_dataset_benchmark")
        self.assertRegex(d["timestamp_utc"], r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\+00:00$")
        self.assertEqual([v for k, v in rows if k == "keyframe"],
                         ["datasets/scared/dataset_1/keyframe_1",
                          "datasets/scared/test_dataset_8/keyframe_0"])
        self.assertEqual(d["git_commit"], self.info["git_commit"])
        self.assertEqual([v for k, v in rows if k == "bench_arg"], [])

    def test_run_sheet_lists_bench_args_in_order(self):
        path = os.path.join(self.tmp.name, "b.xlsx")
        info = ov.run_info(fake_args(bench_args=["--speckle-window=800",
                                                 "--disparity-offset=0.1"]), RESULTS)
        ov.write_xlsx(RESULTS, path, info)
        wb = openpyxl.load_workbook(path)
        try:
            rows = [(r[0].value, r[1].value) for r in wb["run"].iter_rows(min_row=2)]
        finally:
            wb.close()
        self.assertEqual([v for k, v in rows if k == "bench_arg"],
                         ["--speckle-window=800", "--disparity-offset=0.1"])
        keys = [k for k, _ in rows]
        self.assertLess(keys.index("benchmark"), keys.index("bench_arg"))
        self.assertLess(keys.index("bench_arg"), keys.index("keyframe"))


class MissingOpenpyxlTest(unittest.TestCase):
    def test_xlsx_without_openpyxl_exits_2_before_benchmark(self):
        def no_benchmark(*a, **kw):
            raise AssertionError("benchmark must not run when openpyxl is missing")

        stderr = io.StringIO()
        with mock.patch.dict(sys.modules, {"openpyxl": None}), \
                mock.patch.object(ov.subprocess, "run", no_benchmark), \
                contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as cm:
                ov.main(["--xlsx", "/nonexistent/out.xlsx", "some/keyframe"])
        self.assertEqual(cm.exception.code, 2)
        self.assertIn("openpyxl", stderr.getvalue())
        self.assertIn("pip install openpyxl", stderr.getvalue())


class MainTest(unittest.TestCase):
    def test_main_collects_once_and_writes_all(self):
        """Each keyframe is benchmarked once; markdown and CSV agree."""
        import json as _json
        calls = []

        def fake_run(cmd, capture_output, text):
            calls.append(cmd)
            kf = cmd[1]
            m = mock.Mock()
            if kf.endswith("keyframe_1"):
                m.returncode, m.stdout, m.stderr = 0, _json.dumps(GT_JSON), ""
            elif kf.endswith("keyframe_0"):
                m.returncode, m.stdout, m.stderr = 0, _json.dumps(NOGT_JSON), ""
            else:
                m.returncode, m.stdout, m.stderr = 1, "", "boom"
            return m

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "m.csv")
            out, err = io.StringIO(), io.StringIO()
            with mock.patch.object(ov.subprocess, "run", fake_run), \
                    contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                ov.main(["--depth-range", "0.02", "0.30", "--csv", path,
                         "a/dataset_1/keyframe_1", "a/test_dataset_8/keyframe_0",
                         "a/dataset_9/keyframe_3"])
            self.assertEqual(len(calls), 3)
            self.assertEqual(calls[0][1:], ["a/dataset_1/keyframe_1", "160", "-", "0.02", "0.3"])
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
        self.assertEqual(len(rows), 4)
        self.assertEqual([r[1] for r in rows[1:]], ["ok", "ok", "failed"])
        md = out.getvalue().splitlines()
        self.assertEqual(len(md), 5)
        self.assertIn("| dataset_9/keyframe_3 | failed |", md[4])
        self.assertIn("dataset_9/keyframe_3: benchmark failed", err.getvalue())


if __name__ == "__main__":
    unittest.main()

"""Unit tests for --bench-arg forwarding and --fit-offset of
scripts/scared_overview.py.

They run a stub benchmark (a small Python script written to a temporary
directory, passed with --bench) instead of the real binary. The stub echoes
its argv in the JSON it prints, models a per-dataset "true" disparity offset
so that --disparity-offset changes the reported bias, and rejects any flag it
does not know with a usage message and exit status 1, like the real one.

Run:  python3 -m unittest discover -s tests/python -v
"""
import contextlib
import csv
import importlib.util
import io
import json
import os
import stat
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(REPO, "scripts", "scared_overview.py")


def load_module():
    spec = importlib.util.spec_from_file_location("scared_overview_fit", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ov = load_module()

try:
    import openpyxl  # noqa: F401
    HAVE_OPENPYXL = True
except ImportError:
    HAVE_OPENPYXL = False


# The stub's model. Per dataset: true offset (px), the number of matched
# points per keyframe (so the weighting is visible), and a base MAE (mm).
# Reported bias_z = (true - applied) * BIAS_PER_PX; MAE = base + |bias|.
STUB_BENCH = r'''#!/usr/bin/env python3
import json, sys

MODEL = {
    "dataset_1": {"offset": 0.20, "matched": [100000, 300000], "mae_mm": 0.8},
    "dataset_2": {"offset": 0.40, "matched": [200000], "mae_mm": 1.0},
    "dataset_3": {"offset": 0.10, "matched": [400000], "mae_mm": 0.9},
}
BIAS_PER_PX = 0.5  # mm of z bias per pixel of uncorrected offset
KNOWN = ("--disparity-offset", "--speckle-window", "--viewer-frame", "--viewer-stride")

flags = [a for a in sys.argv[1:] if a.startswith("--")]
pos = [a for a in sys.argv[1:] if not a.startswith("--")]
settings = {"disparity_offset_px": 0.0, "speckle_window_size": 100}
for f in flags:
    key, _, value = f.partition("=")
    if key not in KNOWN:
        sys.stderr.write("Invalid option: unknown option %s\nUsage: stub [flags] <keyframe_dir>\n" % f)
        sys.exit(1)
    if key == "--disparity-offset":
        settings["disparity_offset_px"] = float(value)
    if key == "--speckle-window":
        settings["speckle_window_size"] = int(value)

kf = pos[0]
parts = kf.rstrip("/").split("/")
dataset, keyframe = parts[-2], parts[-1]
out = {"keyframe_dir": kf, "argv": sys.argv[1:], "settings": settings,
       "num_disparities": int(pos[1]) if len(pos) > 1 else 160,
       "valid_fraction": 0.8, "z_p05_m": 0.05, "z_median_m": 0.06, "z_p95_m": 0.07,
       "matching_ms": 100.0, "reconstruction_ms": 10.0,
       "has_ground_truth": dataset in MODEL}
if dataset in MODEL:
    m = MODEL[dataset]
    idx = int(keyframe.split("_")[-1]) - 1
    residual_px = m["offset"] - settings["disparity_offset_px"]
    bias_mm = residual_px * BIAS_PER_PX
    out.update({
        "matched_points": m["matched"][idx % len(m["matched"])],
        "coverage": 0.85,
        "mae_3d_m": (m["mae_mm"] + abs(bias_mm)) / 1000.0,
        "mae_z_m": (m["mae_mm"] + abs(bias_mm)) / 1000.0,
        "rmse_3d_m": 0.0014, "median_3d_m": 0.0006,
        "within_1mm": 0.6, "within_2mm": 0.8, "within_5mm": 0.85,
        "bias_z_median_m": bias_mm / 1000.0,
        "fit_disparity_offset_px": residual_px,
        "fit_disparity_offset_residual_m": 0.0001,
        "fit_depth_scale": 1.0, "fit_depth_scale_residual_m": 0.0002,
        "tail_over_5mm": 0.01, "tail_over_20mm": 0.001, "tail_over_50mm": 0.0,
        "max_3d_m": 0.1,
        "coverage_loss": {"out_of_search_range": 0.01, "image_border": 0.06,
                          "matcher_rejected": 0.08, "depth_range": 0.0,
                          "outlier_filter": 0.0},
        "unscored_fraction": 0.17,
        "depth_bias": [],
    })
json.dump(out, sys.stdout)
'''

KF = {
    "d1k1": "data/dataset_1/keyframe_1",
    "d1k2": "data/dataset_1/keyframe_2",
    "d2k1": "data/dataset_2/keyframe_1",
    "d3k1": "data/dataset_3/keyframe_1",
    "t8k0": "data/test_dataset_8/keyframe_0",
}
ALL_GT = [KF["d1k1"], KF["d1k2"], KF["d2k1"], KF["d3k1"]]

# Point-weighted expectations from the stub's MODEL (applied offset 0):
#   dataset_1: 0.20 px x (100000 + 300000), dataset_2: 0.40 x 200000,
#   dataset_3: 0.10 x 400000  ->  (80000 + 80000 + 40000) / 1000000
EXPECTED_ALL = 0.20
EXPECTED_WITHOUT = {  # fit on the other two datasets
    "dataset_1": (0.40 * 200000 + 0.10 * 400000) / 600000,   # 0.2
    "dataset_2": (0.20 * 400000 + 0.10 * 400000) / 800000,   # 0.15
    "dataset_3": (0.20 * 400000 + 0.40 * 200000) / 600000,   # 0.2666..
}


class StubBenchMixin:
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.bench = os.path.join(self.tmp.name, "stub_bench.py")
        with open(self.bench, "w") as f:
            f.write(STUB_BENCH)
        os.chmod(self.bench, os.stat(self.bench).st_mode | stat.S_IXUSR)
        self.snapshot = self._tree()

    def tearDown(self):
        self.tmp.cleanup()

    def _tree(self):
        """Files under the temp dir, to assert that nothing was written."""
        found = set()
        for root, _dirs, files in os.walk(self.tmp.name):
            for name in files:
                found.add(os.path.relpath(os.path.join(root, name), self.tmp.name))
        return found

    def run_main(self, argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                code = ov.main(["--bench", self.bench] + argv)
            except SystemExit as e:
                code = e.code
        return code, out.getvalue(), err.getvalue()


class BenchArgTest(StubBenchMixin, unittest.TestCase):
    def test_flags_forwarded_in_order_to_every_call(self):
        json_dir = os.path.join(self.tmp.name, "json")
        code, out, err = self.run_main([
            "--bench-arg=--speckle-window=800", "--bench-arg=--disparity-offset=0.1",
            "--depth-range", "0.02", "0.3", "--json-dir", json_dir,
            KF["d1k1"], KF["d2k1"], KF["t8k0"]])
        self.assertEqual(code, 0, err)
        names = sorted(os.listdir(json_dir))
        self.assertEqual(len(names), 3)
        for name in names:
            with open(os.path.join(json_dir, name)) as f:
                j = json.load(f)
            self.assertEqual(j["argv"][:2], ["--speckle-window=800", "--disparity-offset=0.1"])
            self.assertEqual(j["argv"][3:], ["160", "-", "0.02", "0.3"])
            self.assertEqual(j["settings"]["disparity_offset_px"], 0.1)
            self.assertEqual(j["settings"]["speckle_window_size"], 800)

    def test_two_word_form_is_joined(self):
        args = ov.parse_args(["--bench-arg", "--speckle-window=800",
                              "--bench-arg=--wls", "x/dataset_1/keyframe_1"])
        self.assertEqual(args.bench_args, ["--speckle-window=800", "--wls"])
        self.assertEqual(args.keyframes, ["x/dataset_1/keyframe_1"])

    def test_rejected_flag_gives_failed_row_with_message(self):
        path = os.path.join(self.tmp.name, "m.csv")
        code, out, err = self.run_main([
            "--bench-arg=--no-such-flag=1", "--csv", path, KF["d1k1"], KF["d2k1"]])
        self.assertEqual(code, 0)
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        self.assertEqual([r[1] for r in rows[1:]], ["failed", "failed"])
        self.assertIn("unknown option --no-such-flag=1", err)
        self.assertIn("dataset_1/keyframe_1: benchmark failed", err)
        self.assertIn("| dataset_2/keyframe_1 | failed |", out)

    def test_rejected_flag_result_carries_error(self):
        r = ov.run_benchmark(self.bench, KF["d1k1"], 160, None, None, None,
                             ["--bogus"])
        self.assertEqual(r.status, "failed")
        self.assertIsNone(r.json)
        self.assertIn("Invalid option: unknown option --bogus", r.error)
        self.assertIn("Usage:", r.error)

    @unittest.skipUnless(HAVE_OPENPYXL, "openpyxl not installed")
    def test_run_sheet_lists_bench_args(self):
        path = os.path.join(self.tmp.name, "m.xlsx")
        code, out, err = self.run_main([
            "--bench-arg=--speckle-window=800", "--xlsx", path, KF["d1k1"]])
        self.assertEqual(code, 0, err)
        wb = openpyxl.load_workbook(path)
        try:
            rows = [(r[0].value, r[1].value) for r in wb["run"].iter_rows(min_row=2)]
            header = [c.value for c in wb["metrics"][1]]
            gt = dict(zip(header, (c.value for c in wb["metrics"][2])))
        finally:
            wb.close()
        self.assertEqual([v for k, v in rows if k == "bench_arg"], ["--speckle-window=800"])
        self.assertEqual(dict(rows)["benchmark"], self.bench)
        self.assertAlmostEqual(gt["fit_disparity_offset_px"], 0.2)


class DatasetIdTest(unittest.TestCase):
    def test_dataset_component(self):
        self.assertEqual(ov.dataset_id("datasets/scared/dataset_1/keyframe_3"), "dataset_1")
        self.assertEqual(ov.dataset_id("datasets/scared/dataset_12/keyframe_3/"), "dataset_12")
        self.assertEqual(ov.dataset_id("/abs/test_dataset_8/keyframe_0"), "test_dataset_8")
        self.assertEqual(ov.dataset_id("dataset_2/keyframe_1"), "dataset_2")

    def test_fallback_to_parent_dir(self):
        self.assertEqual(ov.dataset_id("some/other/layout/kf"), "layout")
        self.assertEqual(ov.dataset_id("kf"), "kf")


class CombinedOffsetTest(unittest.TestCase):
    def result(self, label, delta, matched, current=0.0, gt=True, status="ok"):
        j = None if status != "ok" else {
            "has_ground_truth": gt, "matched_points": matched,
            "fit_disparity_offset_px": delta,
            "settings": {"disparity_offset_px": current}}
        return ov.Result(label, status, j)

    def test_point_weighted_mean(self):
        results = [self.result("a/k1", 0.2, 100000), self.result("a/k2", 0.2, 300000),
                   self.result("b/k1", 0.4, 200000), self.result("c/k1", 0.1, 400000)]
        offset, weight = ov.combined_offset(results)
        self.assertAlmostEqual(offset, 0.2)
        self.assertEqual(weight, 1000000)

    def test_current_offset_is_added(self):
        """A keyframe run with --disparity-offset=0.1 and delta 0.05 recommends 0.15."""
        offset, _ = ov.combined_offset([self.result("a/k1", 0.05, 10, current=0.1)])
        self.assertAlmostEqual(offset, 0.15)

    def test_skips_failed_no_gt_and_unweighted(self):
        results = [self.result("a/k1", 0.3, 100),
                   self.result("b/k1", 9.0, 100, gt=False),
                   self.result("c/k1", 9.0, 0),
                   self.result("d/k1", 9.0, 100, status="failed")]
        offset, weight = ov.combined_offset(results)
        self.assertAlmostEqual(offset, 0.3)
        self.assertEqual(weight, 100)
        self.assertEqual(ov.combined_offset([]), (None, 0))


def parse_table(text):
    """Cells of every markdown data row (header included) in ``text``."""
    rows = []
    for line in text.splitlines():
        if line.startswith("| ") and not line.startswith("|---"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            rows.append(cells)
    return rows


class FitOffsetTest(StubBenchMixin, unittest.TestCase):
    def test_aggregation_and_holdout(self):
        code, out, err = self.run_main(["--fit-offset", "--depth-range", "0.02", "0.3",
                                        "--bench-arg=--speckle-window=800"]
                                       + ALL_GT + [KF["t8k0"]])
        self.assertEqual(code, 0, err)
        self.assertIn("recommended disparity offset: +0.2000 px", out)
        self.assertIn("4 keyframes from 3 datasets, 1000000 matched points", out)
        self.assertIn("test_dataset_8/keyframe_0: no ground truth, not used", err)

        # Holdout table: one row per dataset, fit from the others, bias
        # before/after and whether |bias| decreased.
        rows = [r for r in parse_table(out) if r[0] in EXPECTED_WITHOUT]
        self.assertEqual([r[0] for r in rows], ["dataset_1", "dataset_2", "dataset_3"])
        by_ds = {r[0]: r for r in rows}
        self.assertEqual(by_ds["dataset_1"][1], "2")
        for ds, fit in EXPECTED_WITHOUT.items():
            self.assertAlmostEqual(float(by_ds[ds][2]), fit, places=4, msg=ds)
        # dataset_1: true 0.20, fit from others 0.20 -> bias 0.100 mm -> 0.000
        self.assertAlmostEqual(float(by_ds["dataset_1"][3]), 0.9, places=3)   # MAE before
        self.assertAlmostEqual(float(by_ds["dataset_1"][4]), 0.8, places=3)   # MAE after
        self.assertAlmostEqual(float(by_ds["dataset_1"][5]), 0.1, places=3)   # bias before
        self.assertAlmostEqual(float(by_ds["dataset_1"][6]), 0.0, places=3)   # bias after
        self.assertEqual(by_ds["dataset_1"][7], "yes")
        # dataset_2: true 0.40, fit 0.15 -> bias 0.200 -> 0.125 mm
        self.assertAlmostEqual(float(by_ds["dataset_2"][5]), 0.2, places=3)
        self.assertAlmostEqual(float(by_ds["dataset_2"][6]), 0.125, places=3)
        self.assertEqual(by_ds["dataset_2"][7], "yes")
        # dataset_3: true 0.10, fit 0.2667 -> bias 0.050 -> -0.0833 mm (worse)
        self.assertAlmostEqual(float(by_ds["dataset_3"][5]), 0.05, places=3)
        self.assertAlmostEqual(float(by_ds["dataset_3"][6]), -0.0833, places=3)
        self.assertEqual(by_ds["dataset_3"][7], "no")
        self.assertNotIn("holdout check skipped", out)
        # Nothing was written next to the stub.
        self.assertEqual(self._tree(), self.snapshot)

    def test_holdout_reruns_with_offset_after_bench_args(self):
        calls = []
        real_run = ov.run_benchmark

        def spy(bench, kf, *a, **kw):
            calls.append((kf, list(a[-1]) if a else list(kw.get("extra_args", ()))))
            return real_run(bench, kf, *a, **kw)

        ov.run_benchmark = spy
        try:
            code, out, err = self.run_main(["--fit-offset", "--bench-arg=--speckle-window=800",
                                            KF["d1k1"], KF["d2k1"]])
        finally:
            ov.run_benchmark = real_run
        self.assertEqual(code, 0, err)
        first = [c for c in calls[:2]]
        self.assertEqual(first, [(KF["d1k1"], ["--speckle-window=800"]),
                                 (KF["d2k1"], ["--speckle-window=800"])])
        reruns = calls[2:]
        self.assertEqual([kf for kf, _ in reruns], [KF["d1k1"], KF["d2k1"]])
        # dataset_1 held out: fit from dataset_2 = 0.40; dataset_2: from dataset_1 = 0.20
        self.assertEqual(reruns[0][1], ["--speckle-window=800", "--disparity-offset=0.400000"])
        self.assertEqual(reruns[1][1], ["--speckle-window=800", "--disparity-offset=0.200000"])

    def test_single_dataset_prints_offset_and_message(self):
        code, out, err = self.run_main(["--fit-offset", KF["d1k1"], KF["d1k2"]])
        self.assertEqual(code, 0, err)
        self.assertIn("recommended disparity offset: +0.2000 px", out)
        self.assertIn("needs keyframes from at least two datasets (got 1: dataset_1)", out)
        self.assertNotIn("held-out dataset", out)
        self.assertEqual(self._tree(), self.snapshot)

    def test_no_ground_truth_keyframe_fails(self):
        code, out, err = self.run_main(["--fit-offset", KF["t8k0"]])
        self.assertEqual(code, 1)
        self.assertIn("at least one successful keyframe with ground truth", err)

    def test_file_outputs_are_rejected(self):
        for flag in (["--csv", "x.csv"], ["--xlsx", "x.xlsx"], ["--png-dir", "p"],
                     ["--json-dir", "j"], ["--view"], ["--view-out", "v"]):
            code, out, err = self.run_main(["--fit-offset"] + flag + [KF["d1k1"]])
            self.assertEqual(code, 2, flag)
            self.assertIn("--fit-offset writes no files", err)
        self.assertEqual(self._tree(), self.snapshot)


if __name__ == "__main__":
    unittest.main()

"""Unit tests for scripts/cloud_viewer/build_site.py and the --view wiring of
scripts/scared_overview.py.

The benchmark is never run: subprocess.run is faked and writes a stub frame
file where the --viewer-frame flag points. webbrowser.open is patched so no
browser is launched.

Run:  python3 -m unittest discover -s tests/python -v
"""
import contextlib
import csv
import importlib.util
import io
import json
import os
import socket
import sys
import tempfile
import unittest
from unittest import mock

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from cloud_viewer import build_site  # noqa: E402


def load_overview():
    spec = importlib.util.spec_from_file_location(
        "scared_overview_view", os.path.join(SCRIPTS, "scared_overview.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ov = load_overview()

GT_JSON = {
    "keyframe_dir": "datasets/scared/dataset_1/keyframe_1",
    "valid_fraction": 0.84172, "z_p05_m": 0.05213, "z_median_m": 0.05912,
    "z_p95_m": 0.07401, "z_min_m": 0.04, "z_max_m": 0.3,
    "matching_ms": 328.4567, "reconstruction_ms": 12.3456,
    "has_ground_truth": True, "ground_truth_points": 900000,
    "coverage": 0.855, "mae_x_m": 0.0003, "mae_z_m": 0.00071234,
    "mae_3d_m": 0.00084321, "rmse_3d_m": 0.001432, "median_3d_m": 0.000591,
    "within_1mm": 0.6271, "within_2mm": 0.7993, "within_5mm": 0.8462,
    "image_width": 1280, "image_height": 1024,
}

NOGT_JSON = {
    "keyframe_dir": "datasets/scared/test_dataset_8/keyframe_0",
    "valid_fraction": 0.8349, "z_p05_m": 0.1011, "z_median_m": 0.1153,
    "z_p95_m": 0.1352, "matching_ms": 314.2, "reconstruction_ms": 11.9,
    "has_ground_truth": False, "coverage": None,
}

TEMPLATE_FILES = ["index.html", "viewer.js", "viewer.css",
                  os.path.join("vendor", "three.min.js"),
                  os.path.join("vendor", "OrbitControls.js"),
                  os.path.join("vendor", "LICENSE-three.txt")]


def stub_frame(site_dir, fid):
    path = build_site.frame_path(site_dir, fid)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write('CloudViewer.addFrame(%s, "");\n' % json.dumps(fid))
    return path


class FrameIdTest(unittest.TestCase):
    def test_slash_becomes_underscore(self):
        self.assertEqual(build_site.frame_id("dataset_1/keyframe_1"), "dataset_1_keyframe_1")
        self.assertEqual(build_site.frame_id("test_dataset_8/keyframe_0"),
                         "test_dataset_8_keyframe_0")

    def test_sanitised(self):
        for label in ["a b/c:d", "x\\y", "/lead/", "ünïcode/kf", "..", ""]:
            fid = build_site.frame_id(label)
            self.assertRegex(fid, r"^[A-Za-z0-9_.-]+$", label)
        self.assertEqual(build_site.frame_id("a b/c:d"), "a-b_c-d")
        self.assertEqual(build_site.frame_id("v1.2/kf-3"), "v1.2_kf-3")
        self.assertEqual(build_site.frame_id(""), "frame")


class MetricsTest(unittest.TestCase):
    def test_filter_metrics_keeps_only_present_contract_keys(self):
        m = build_site.filter_metrics(GT_JSON)
        self.assertEqual(set(m), set(build_site.METRIC_KEYS))
        self.assertNotIn("mae_x_m", m)
        self.assertNotIn("image_width", m)
        self.assertNotIn("keyframe_dir", m)
        self.assertEqual(m["mae_3d_m"], 0.00084321)
        self.assertEqual(m["matching_ms"], 328.4567)

    def test_filter_metrics_drops_none_and_absent(self):
        m = build_site.filter_metrics(NOGT_JSON)
        self.assertNotIn("coverage", m)
        self.assertNotIn("mae_3d_m", m)
        self.assertEqual(set(m), {"valid_fraction", "z_p05_m", "z_median_m",
                                  "z_p95_m", "matching_ms", "reconstruction_ms"})
        self.assertEqual(build_site.filter_metrics(None), {})
        self.assertEqual(build_site.filter_metrics({}), {})

    def test_frame_entry(self):
        e = build_site.frame_entry("dataset_1_keyframe_1", "dataset_1/keyframe_1", GT_JSON)
        self.assertEqual(e["id"], "dataset_1_keyframe_1")
        self.assertEqual(e["label"], "dataset_1/keyframe_1")
        self.assertIs(e["has_gt"], True)
        e = build_site.frame_entry("x", "x", NOGT_JSON)
        self.assertIs(e["has_gt"], False)
        e = build_site.frame_entry("x", "x", {})
        self.assertIs(e["has_gt"], False)
        self.assertEqual(e["metrics"], {})


class BuildSiteTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.site = os.path.join(self.tmp.name, "out", "viewer")

    def tearDown(self):
        self.tmp.cleanup()

    def test_prepare_copies_template_and_creates_frames(self):
        site = build_site.prepare_site(self.site)
        self.assertEqual(site, os.path.abspath(self.site))
        for rel in TEMPLATE_FILES:
            self.assertTrue(os.path.isfile(os.path.join(site, rel)), rel)
        self.assertTrue(os.path.isdir(os.path.join(site, "frames")))
        with open(os.path.join(site, "index.html")) as f, \
                open(os.path.join(build_site.TEMPLATE_DIR, "index.html")) as g:
            self.assertEqual(f.read(), g.read())
        # idempotent and keeps existing frames
        stub_frame(site, "keep")
        build_site.prepare_site(self.site)
        self.assertTrue(os.path.isfile(build_site.frame_path(site, "keep")))

    def test_manifest_shape_and_roundtrip(self):
        frames = [build_site.frame_entry("dataset_1_keyframe_1", "dataset_1/keyframe_1", GT_JSON),
                  build_site.frame_entry("test_dataset_8_keyframe_0",
                                         "test_dataset_8/keyframe_0", NOGT_JSON)]
        index, size = build_site.build_site(self.site, frames, generated="2026-01-02T03:04:05+00:00")
        self.assertEqual(index, os.path.join(os.path.abspath(self.site), "index.html"))
        self.assertGreater(size, 500_000)  # three.js is in there
        path = os.path.join(self.site, "frames", "manifest.js")
        with open(path, encoding="utf-8") as f:
            text = f.read()
        self.assertTrue(text.startswith("CloudViewer.setManifest({"))
        self.assertTrue(text.rstrip().endswith("});"))
        m = build_site.read_manifest(self.site)
        self.assertEqual(sorted(m), ["frames", "generated", "title"])
        self.assertEqual(m["title"], "SCARED reconstruction viewer")
        self.assertEqual(m["generated"], "2026-01-02T03:04:05+00:00")
        self.assertEqual(len(m["frames"]), 2)
        f0, f1 = m["frames"]
        self.assertEqual(sorted(f0), ["has_gt", "id", "label", "metrics"])
        self.assertEqual(f0["id"], "dataset_1_keyframe_1")
        self.assertEqual(f0["label"], "dataset_1/keyframe_1")
        self.assertIs(f0["has_gt"], True)
        self.assertEqual(f0["metrics"]["mae_3d_m"], 0.00084321)
        self.assertEqual(f0["metrics"]["within_5mm"], 0.8462)
        self.assertIs(f1["has_gt"], False)
        self.assertNotIn("coverage", f1["metrics"])
        self.assertNotIn("mae_3d_m", f1["metrics"])
        self.assertNotIn("null", text)

    def test_generated_is_utc_iso(self):
        build_site.build_site(self.site, [])
        m = build_site.read_manifest(self.site)
        self.assertRegex(m["generated"], r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\+00:00$")
        self.assertEqual(m["frames"], [])

    def test_write_manifest_rejects_bad_id(self):
        build_site.prepare_site(self.site)
        with self.assertRaises(ValueError):
            build_site.write_manifest(self.site, build_site.make_manifest(
                [{"id": "a/b", "label": "x", "has_gt": False, "metrics": {}}]))

    def test_describe(self):
        self.assertEqual(build_site.describe("/x/index.html", 3_250_000, 2),
                         "viewer: /x/index.html (3.2 MB, 2 frames)")
        self.assertEqual(build_site.describe("/x/index.html", 900_000, 1),
                         "viewer: /x/index.html (0.9 MB, 1 frame)")

    def test_viewer_frames_excludes_failed_and_missing(self):
        site = build_site.prepare_site(self.site)
        stub_frame(site, "dataset_1_keyframe_1")
        stub_frame(site, "test_dataset_8_keyframe_0")
        gt = dict(GT_JSON, viewer_frame=build_site.frame_path(site, "dataset_1_keyframe_1"))
        results = [ov.Result("dataset_1/keyframe_1", "ok", gt),
                   ov.Result("dataset_9/keyframe_3", "failed", None),
                   ov.Result("test_dataset_8/keyframe_0", "ok", NOGT_JSON),
                   ov.Result("dataset_2/keyframe_2", "ok", dict(NOGT_JSON))]  # no frame file
        err = io.StringIO()
        frames = ov.viewer_frames(results, site, err)
        self.assertEqual([f["id"] for f in frames],
                         ["dataset_1_keyframe_1", "test_dataset_8_keyframe_0"])
        self.assertIn("dataset_2/keyframe_2: no viewer frame written", err.getvalue())
        self.assertNotIn("dataset_9", err.getvalue())


class OpenInBrowserTest(unittest.TestCase):
    def test_returns_false_and_notes_path_when_no_browser(self):
        err = io.StringIO()
        with mock.patch.object(build_site.webbrowser, "open", return_value=False) as op:
            ok = build_site.open_in_browser("/some/site/index.html", err)
        self.assertFalse(ok)
        op.assert_called_once_with("file:///some/site/index.html")
        self.assertIn("could not open a browser", err.getvalue())
        self.assertIn("file:///some/site/index.html", err.getvalue())

    def test_swallows_webbrowser_error(self):
        import webbrowser
        err = io.StringIO()
        with mock.patch.object(build_site.webbrowser, "open",
                               side_effect=webbrowser.Error("no runnable browser")):
            self.assertFalse(build_site.open_in_browser("/x/index.html", err))
        self.assertIn("/x/index.html", err.getvalue())

    def test_http_url_passed_through(self):
        with mock.patch.object(build_site.webbrowser, "open", return_value=True) as op:
            self.assertTrue(build_site.open_in_browser("http://127.0.0.1:8765/index.html"))
        op.assert_called_once_with("http://127.0.0.1:8765/index.html")


class ServeTest(unittest.TestCase):
    def test_port_in_use_returns_2_and_names_port(self):
        blocker = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        blocker.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        blocker.bind(("127.0.0.1", 0))
        blocker.listen(1)
        port = blocker.getsockname()[1]
        try:
            err = io.StringIO()
            with tempfile.TemporaryDirectory() as tmp, \
                    mock.patch.object(build_site.webbrowser, "open") as op:
                rc = build_site.serve(tmp, port, "127.0.0.1", open_browser=True, out=err)
        finally:
            blocker.close()
        self.assertEqual(rc, 2)
        self.assertEqual(err.getvalue().strip(),
                         f"error: port {port} on 127.0.0.1 is already in use")
        op.assert_not_called()

    def test_serve_prints_url_and_stops_on_keyboard_interrupt(self):
        with tempfile.TemporaryDirectory() as tmp:
            server = build_site.make_server(tmp, 0, "127.0.0.1")
            port = server.server_address[1]
            server.serve_forever = mock.Mock(side_effect=KeyboardInterrupt)
            err = io.StringIO()
            with mock.patch.object(build_site.webbrowser, "open", return_value=True) as op:
                rc = build_site.serve(tmp, port, "127.0.0.1", open_browser=True,
                                      out=err, server=server)
        self.assertEqual(rc, 0)
        self.assertIn(f"http://127.0.0.1:{port}/index.html", err.getvalue())
        op.assert_called_once_with(f"http://127.0.0.1:{port}/index.html")

    def test_main_serve_port_in_use_exits_2(self):
        blocker = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        blocker.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        blocker.bind(("127.0.0.1", 0))
        blocker.listen(1)
        port = blocker.getsockname()[1]
        try:
            with tempfile.TemporaryDirectory() as tmp:
                site = os.path.join(tmp, "site")
                out, err = io.StringIO(), io.StringIO()
                with mock.patch.object(ov.subprocess, "run", make_fake_run([])), \
                        mock.patch.object(build_site.webbrowser, "open") as op, \
                        contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                    rc = ov.main(["--serve", str(port), "--view-out", site,
                                  "a/dataset_1/keyframe_1"])
                # the site is still written before the port error
                self.assertTrue(os.path.isfile(os.path.join(site, "frames", "manifest.js")))
        finally:
            blocker.close()
        self.assertEqual(rc, 2)
        self.assertIn(f"error: port {port} on 127.0.0.1 is already in use", err.getvalue())
        self.assertEqual(out.getvalue().count("\n"), 3)  # markdown table only
        op.assert_not_called()


def make_fake_run(calls):
    """A subprocess.run stand-in: records cmd, writes the stub frame file
    named by --viewer-frame=, and answers with canned JSON per keyframe."""
    def fake_run(cmd, capture_output, text):
        calls.append(list(cmd))
        flags = [a for a in cmd[1:] if a.startswith("--")]
        positionals = [a for a in cmd[1:] if not a.startswith("--")]
        kf = positionals[0]
        m = mock.Mock()
        if kf.endswith("keyframe_1"):
            payload = dict(GT_JSON)
        elif kf.endswith("keyframe_0"):
            payload = dict(NOGT_JSON)
        else:
            m.returncode, m.stdout, m.stderr = 1, "", "boom"
            return m
        for a in flags:
            if a.startswith("--viewer-frame="):
                path = a[len("--viewer-frame="):]
                os.makedirs(os.path.dirname(path), exist_ok=True)
                fid = os.path.splitext(os.path.basename(path))[0]
                with open(path, "w") as f:
                    f.write('CloudViewer.addFrame(%s, "");\n' % json.dumps(fid))
                payload["viewer_frame"] = path
        m.returncode, m.stdout, m.stderr = 0, json.dumps(payload), ""
        return m
    return fake_run


class ViewWiringTest(unittest.TestCase):
    KEYFRAMES = ["a/dataset_1/keyframe_1", "a/test_dataset_8/keyframe_0",
                 "a/dataset_9/keyframe_3"]

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.site = os.path.join(self.tmp.name, "viewer")
        self.calls = []

    def tearDown(self):
        self.tmp.cleanup()

    def run_main(self, argv):
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(ov.subprocess, "run", make_fake_run(self.calls)), \
                mock.patch.object(build_site.webbrowser, "open", return_value=True) as op, \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = ov.main(argv)
        return rc, out.getvalue(), err.getvalue(), op

    def test_view_passes_viewer_flags_once_per_keyframe(self):
        rc, out, err, op = self.run_main(
            ["--view", "--view-out", self.site, "--no-open", "--viewer-stride", "3",
             "--depth-range", "0.02", "0.30"] + self.KEYFRAMES)
        self.assertEqual(rc, 0)
        self.assertEqual(len(self.calls), 3)
        site = os.path.abspath(self.site)
        for cmd, kf, fid in zip(self.calls, self.KEYFRAMES,
                                ["dataset_1_keyframe_1", "test_dataset_8_keyframe_0",
                                 "dataset_9_keyframe_3"]):
            self.assertEqual(cmd[0], ov.DEFAULT_BENCH)
            self.assertEqual(cmd[1], f"--viewer-frame={site}/frames/{fid}.js")
            self.assertEqual(cmd[2], "--viewer-stride=3")
            self.assertEqual(cmd[3:], [kf, "160", "-", "0.02", "0.3"])
            self.assertEqual(sum(a.startswith("--viewer-frame=") for a in cmd), 1)
            self.assertEqual(sum(a.startswith("--viewer-stride=") for a in cmd), 1)
        # stdout is exactly the Markdown table
        lines = out.splitlines()
        self.assertEqual(len(lines), 5)
        self.assertTrue(all(l.startswith("|") for l in lines))
        self.assertIn("| dataset_9/keyframe_3 | failed |", out)
        # site: template + manifest with the two successful frames
        m = build_site.read_manifest(self.site)
        self.assertEqual([f["id"] for f in m["frames"]],
                         ["dataset_1_keyframe_1", "test_dataset_8_keyframe_0"])
        self.assertEqual([f["label"] for f in m["frames"]],
                         ["dataset_1/keyframe_1", "test_dataset_8/keyframe_0"])
        self.assertEqual([f["has_gt"] for f in m["frames"]], [True, False])
        self.assertEqual(m["frames"][0]["metrics"]["mae_3d_m"], GT_JSON["mae_3d_m"])
        for rel in TEMPLATE_FILES:
            self.assertTrue(os.path.isfile(os.path.join(self.site, rel)), rel)
        self.assertTrue(os.path.isfile(build_site.frame_path(self.site, "dataset_1_keyframe_1")))
        self.assertFalse(os.path.isfile(build_site.frame_path(self.site, "dataset_9_keyframe_3")))
        self.assertRegex(err, r"viewer: .*/viewer/index\.html \(\d+\.\d MB, 2 frames\)")
        self.assertIn("dataset_9/keyframe_3: benchmark failed", err)
        op.assert_not_called()

    def test_csv_and_view_run_benchmark_once_and_agree(self):
        path = os.path.join(self.tmp.name, "m.csv")
        rc, out, err, op = self.run_main(
            ["--csv", path, "--view-out", self.site, "--no-open"] + self.KEYFRAMES[:2])
        self.assertEqual(rc, 0)
        self.assertEqual(len(self.calls), 2)
        self.assertTrue(all(any(a.startswith("--viewer-frame=") for a in c)
                            for c in self.calls))
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        m = build_site.read_manifest(self.site)
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(m["frames"]), 2)
        self.assertAlmostEqual(float(rows[0]["mae_3d_mm"]),
                               m["frames"][0]["metrics"]["mae_3d_m"] * 1000, places=4)
        self.assertAlmostEqual(float(rows[1]["depth_median_mm"]),
                               m["frames"][1]["metrics"]["z_median_m"] * 1000, places=4)
        self.assertIn(f"wrote {path}", err)

    def test_no_view_passes_no_viewer_flags(self):
        rc, out, err, op = self.run_main(self.KEYFRAMES[:1])
        self.assertEqual(rc, 0)
        self.assertEqual(self.calls[0][1], self.KEYFRAMES[0])
        self.assertFalse(any(a.startswith("--viewer") for a in self.calls[0]))
        self.assertFalse(os.path.exists(self.site))
        op.assert_not_called()

    def test_view_opens_browser_unless_no_open(self):
        rc, out, err, op = self.run_main(["--view-out", self.site] + self.KEYFRAMES[:1])
        self.assertEqual(rc, 0)
        index = os.path.join(os.path.abspath(self.site), "index.html")
        op.assert_called_once_with("file://" + index)
        self.calls.clear()
        rc, out, err, op = self.run_main(["--view", "--view-out", self.site, "--no-open"]
                                         + self.KEYFRAMES[:1])
        op.assert_not_called()

    def test_all_failed_writes_empty_site(self):
        rc, out, err, op = self.run_main(["--view-out", self.site, "--no-open",
                                          self.KEYFRAMES[2]])
        self.assertEqual(rc, 0)
        m = build_site.read_manifest(self.site)
        self.assertEqual(m["frames"], [])
        self.assertIn("0 frames", err)

    def test_default_view_out_and_stride(self):
        args = ov.parse_args(["--view", "kf"])
        self.assertEqual(args.view_out, ov.DEFAULT_VIEW_OUT)
        self.assertEqual(args.viewer_stride, 4)
        self.assertIsNone(args.serve)
        self.assertEqual(args.bind, "127.0.0.1")
        args = ov.parse_args(["kf", "--serve"])  # bare --serve after the keyframes
        self.assertTrue(args.view)
        self.assertEqual(args.serve, ov.DEFAULT_SERVE_PORT)
        args = ov.parse_args(["--serve", "9000", "--bind", "0.0.0.0", "kf"])
        self.assertEqual((args.serve, args.bind), (9000, "0.0.0.0"))
        args = ov.parse_args(["kf"])
        self.assertFalse(args.view)


if __name__ == "__main__":
    unittest.main()

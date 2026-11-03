# Tasks

## 1. Viewer frame export (C++)

- [x] 1.1 Add `ViewerFrameOptions` and `write_viewer_frame()` (CPV1 payload, base64, `frames/<id>.js` wrapper) to `include/cloud_point/cloud_point_client.hpp` / `src/cloud_point/cloud_point_client.cpp`, reusing the `base64` subproject; verify it builds in `build-opencv/`
- [x] 1.2 Add `tests/test_viewer_export.cpp` (header fields, `INT16_MIN` invalid marker, quantisation ≤ 0.01 mm, stride sampling, GT/RGB flags, and a round-trip MAE equal to the directly computed MAE) and register it in `tests/meson.build`; verify with `meson test -C build-opencv test_viewer_export`
- [x] 1.3 Add `CloudPointClient::compute_frame()` returning the cloud plus the rectified left BGR image, and make `compute_cloud()` delegate to it; verify with `meson test -C build-opencv` (existing client tests unchanged and passing) plus a new case that checks the image size equals the cloud size
- [x] 1.4 Add `--viewer-frame=PATH` / `--viewer-stride=N` flags to `scared_dataset_benchmark` (parsed before the positional arguments, which stay compatible) and add `viewer_frame` to the JSON; verify that `scared_dataset_benchmark datasets/scared/dataset_1/keyframe_1 160 - 0.02 0.30` still prints identical metrics, and that adding `--viewer-frame=/tmp/f.js` writes the file

## 2. Viewer site template

- [x] 2.1 Create `scripts/cloud_viewer/site/` (`index.html`, `viewer.css`, `viewer.js`) from the exploration prototype: lazy `<script>` frame loading via `CloudViewer.addFrame`, a manifest-driven frame list and metrics, colour modes, GT overlay, histogram and depth-bias charts, theme tokens, and GT-only controls disabled for no-GT frames; verify by opening a hand-built one-frame site via `file://` in Chromium and Firefox
- [x] 2.2 Vendor three.js r128 `three.min.js` + `examples/js/controls/OrbitControls.js` and `LICENSE` into `scripts/cloud_viewer/site/vendor/`; verify with a grep over the site that the page references no `http(s)://` URLs

## 3. Dataset tooling: overview refactor, tables, viewer launch

- [x] 3.1 Refactor `scripts/scared_overview.py` into collect-then-write (`Result` list, shared `COLUMNS` definition); verify that the Markdown output on `datasets/scared/dataset_1/keyframe_*` is byte-identical to the pre-change output
- [x] 3.2 Add `--csv PATH` (stdlib `csv`, mm/percent units, empty cells for N/A, `status` column); verify with a unit test on canned JSON (GT, no-GT, failed rows)
- [x] 3.3 Add `--xlsx PATH` with openpyxl (`metrics` sheet with frozen header, number formats and autofilter; `run` sheet with disparities, depth range, timestamp, git commit and bench path) and an upfront missing-package check that exits non-zero before benchmarking; verify with unit tests (skipped without openpyxl) and by running with openpyxl unavailable
- [x] 3.4 Add `scripts/cloud_viewer/build_site.py` (copy the template, write the manifest with metrics, place the frame files, print the site size) and the `--view`, `--view-out DIR`, `--no-open`, `--viewer-stride N`, `--serve [PORT]` and `--bind ADDR` flags in `scared_overview.py`, with the benchmark invoked with `--viewer-frame`; verify that a run on 2 keyframes with `--no-open` produces a site that opens via `file://` with both frames, and that `--serve 8080` on this machine (port taken) exits with status 2 and names the port
- [x] 3.5 Add `tests/python/test_overview_outputs.py` and a meson `test()` that runs it when `python3` is found; verify with `meson test -C build-opencv`

## 4. CLI launch

- [x] 4.1 Add a `viewer` config section (`output_dir`, `template_dir`, `open_command`, `stride`) to `include/cloud_point_rpc/config.hpp`, `CliStereoOptions`, `src/main.cpp` and `config.yaml.example`, plus the `CPRPC_VIEWER_TEMPLATE_DIR` compile-time default in meson; verify with a `tests/test_config.cpp` case for defaults and overrides
- [x] 4.2 Add menu option 6 "compute-cloud + open viewer" in `src/cli.cpp`: `compute_frame()`, the cloud summary, the site build (template copy on first use, timestamped frame id, manifest rewrite), a non-blocking `posix_spawnp` of the open command with failure reported as a note, and the same "OpenCV support not built" message in non-OpenCV builds; verify with new `tests/test_cli.cpp` cases (open command `true` → site files exist; `false` → note printed and menu continues; compute error → no site written)
- [x] 4.3 Manual end-to-end check: run `scared_dataset_server` on port 8090 with `dataset_1/keyframe_1`, run the CLI with `config.scared.yml` pointed at 8090, select option 6 twice, and confirm the browser opens a site with two frames in image/depth modes

## 5. Environment and documentation

- [x] 5.1 Add `python3-openpyxl` to the `Dockerfile`; verify that `docker build` succeeds and `python3 -c "import openpyxl"` works in the container
- [x] 5.2 Update `README.md` (the viewer via `scared_overview.py --view` and CLI option 6, CSV/XLSX flags, file:// vs `--serve`, the openpyxl note) and `config.yaml.example`, and update `API.md` for `compute_frame()` / `write_viewer_frame()`; verify every documented command by running it once
- [x] 5.3 Leave generated OpenWiki pages to the scheduled regeneration (do not hand-edit); verify that `git diff --stat openwiki/` is empty

## 6. Integration check

- [x] 6.1 Run `scripts/scared_overview.py --depth-range 0.02 0.30 --csv out/m.csv --xlsx out/m.xlsx --view --no-open datasets/scared/dataset_*/keyframe_* datasets/scared/test_dataset_8/keyframe_*` and verify that each keyframe is benchmarked once, that the CSV, XLSX and viewer metrics agree, that the per-frame MAE recomputed in the viewer is within 0.05 mm of the benchmark, and that the numbers match the README reference table

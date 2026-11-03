# Design

## Context

- `scripts/scared_overview.py` runs `scared_dataset_benchmark` once per keyframe (a subprocess that prints JSON), then prints a Markdown table. It already supports `--png-dir` and `--json-dir`, but its results only live in the table printing loop.
- `scared_dataset_benchmark` takes positional arguments only: `KEYFRAME [NUM_DISP] [depth.png|-] [MIN MAX]`. It holds the cloud, the rectified ground-truth point map and the left image, but never writes the cloud.
- `CloudPointClient::compute_cloud()` returns only a `PointCloud`. The rectified left image stays internal, so the CLI has no colour for the viewer.
- The CLI (`src/cli.cpp`) is a numbered menu. Options 4 and 5 share one reconstruction branch, and option 5 adds `write_ply`. `CliStereoOptions` is the OpenCV-free settings struct filled from `config.hpp`.
- An exploration prototype (a scratch exporter plus a three.js page) proved the approach. Int16 samples at 0.02 mm and stride 4 give ~1.2 MB per frame, and the MAE recomputed from the samples matched the benchmark within 0.04 mm.
- Constraints from local setup: port 8080 is often taken, and the Docker dev container has no browser. Viewers must work without a network.

## Goals / Non-Goals

**Goals:**
- One viewer page and one frame format shared by the Python tooling and the C++ CLI.
- A generated site opens by double-click (`file://`), so no server is needed in the common case.
- Existing behaviour stays the same when no new flags or menu options are used.

**Non-Goals:**
- Mesh rendering, streaming or live-updating views (the site is a static snapshot).
- Serving the viewer from `rpc_server` or Unity.
- Changing matcher settings or accuracy (a follow-up change will cover that).

## Decisions

### 1. Frame data ships as JavaScript files, not fetched binaries
Each frame is written as `frames/<id>.js` containing `CloudViewer.addFrame({id, label, meta}, "<base64 payload>")`, and `frames/manifest.js` lists the frames and their metrics. The page adds a `<script>` tag when a frame is first selected.
- *Why:* `fetch()` of local files is blocked under `file://` in Chromium, but `<script src>` is allowed. That makes the site open without a server, which the Docker/headless scenario and "copy the folder to a laptop" both need.
- *Alternatives:* fetch `.bin` files over a local HTTP server (needs a server for every view, and ports get in the way); inline every frame into `index.html` (12 frames make a ~16 MB page that is slow to open and all-or-nothing).
- *Cost:* base64 adds 33 % (~1.6 MB per frame at stride 4). That is acceptable.

### 2. Frame payload format ("CPV1")
Little-endian: magic `CPV1`, then `u16 width, u16 height, u16 stride, u8 flags (bit0 = has GT, bit1 = has RGB), u8 reserved`. The arrays follow as `int16 xyz[w*h*3]`, then `int16 gt_xyz[w*h*3]` if flag bit0 is set, then `u8 rgb[w*h*3]` if flag bit1 is set. XYZ values are rectified-left camera coordinates in units of 0.02 mm (±655 mm range); `INT16_MIN` means invalid. Samples are taken at `(r*stride, c*stride)` without averaging, so the error statistics stay honest.
- *Alternatives:* float32 (2× larger for no visible gain, since quantisation is 0.01 mm RMS, far below the ~0.6 mm median error); PLY (no GT channel, and the viewer would have to parse text).

### 3. One C++ writer, used by both entry points
Add `write_viewer_frame(const PointCloud&, const cv::Mat* gt_rectified, const cv::Mat* rgb_rectified, const ViewerFrameOptions&)` in `cloud_point_client` next to `write_ply`. It returns the base64 payload, and a small helper writes the `frames/<id>.js` wrapper. It reuses the existing `base64` subproject.
- The benchmark gains optional flags, parsed before the positional arguments to stay backward compatible: `--viewer-frame=PATH` and `--viewer-stride=N`. It writes the frame file and adds `"viewer_frame": PATH` to its JSON.
- *Alternative:* Python encodes the frames from a raw dump. Rejected because the CLI must build a site without Python.

### 4. Colour for CLI frames: additive `compute_frame()`
Add `CloudPointClient::compute_frame()`, which returns `{PointCloud cloud; cv::Mat rectified_left_bgr;}`. `compute_cloud()` becomes a thin wrapper around it, so the API stays compatible. The rectifier already accepts any `cv::remap` type, so this means rectifying the colour image in addition to the gray one. That is ~5 ms, and only `compute_frame()` pays it.

### 5. Site template lives in `scripts/cloud_viewer/site/`
`index.html`, `viewer.js`, `viewer.css` and `vendor/three.min.js` + `vendor/OrbitControls.js` (three r128, MIT; licence file kept next to them). Both builders copy this template and add `frames/`.
- The Python builder (`scripts/cloud_viewer/build_site.py`) is imported by `scared_overview.py`.
- The C++ CLI finds the template through `viewer.template_dir` in the config. When that is unset, it falls back to a compile-time `CPRPC_VIEWER_TEMPLATE_DIR` define (the source path, which is right for both the host and the bind-mounted Docker build). A missing template gives a clear error that names the key.
- *Alternative:* embed the template into the binary via a meson `custom_target`. Rejected for now: it adds ~700 KB of vendored JS to the binary and a generator step, and there are no installed deployments yet.

### 6. Opening and serving
- Python: `webbrowser.open(file_uri)`. `--no-open` skips it. `--serve [PORT]` (default 8765) starts `ThreadingHTTPServer` on 127.0.0.1 and blocks until Ctrl-C. `--bind` overrides the address for Docker `--network=host` use. A port in use exits with status 2 and a message.
- CLI: launches `viewer.open_command` (default `xdg-open`, `open` on macOS) with `posix_spawnp` and does not wait for it. A spawn failure or non-zero exit is only reported as a note.
- Output directory: Python uses `--view-out DIR` (default `out/viewer`). The CLI uses `viewer.output_dir` (default `out/viewer-cli`) and writes a timestamped `frames/` id so that repeated captures add up in one site. The manifest is rewritten each time and lists all frames present.

### 7. Refactor `scared_overview.py` into collect-then-write
First collect a list of `Result(label, status, json)` by running the benchmark once per keyframe, then pass that list to the writers: Markdown (stdout, unchanged format), `--csv`, `--xlsx`, `--json-dir` and `--view`. Column definitions live in one table (`COLUMNS = [(header, key, transform, number_format)]`) shared by CSV and XLSX so the two cannot drift apart.
- XLSX: `openpyxl` is imported lazily, but its availability is checked right after argument parsing so a missing package fails before any benchmark runs. Sheets: `metrics` (frozen first row, `0.00` / `0.0%` formats, autofilter) and `run` (parameters, `git rev-parse HEAD` when available, and the benchmark path).
- *Alternative:* a stdlib-only xlsx writer. Rejected by the user in favour of optional openpyxl.

### 8. Tests
- GTest `tests/test_viewer_export.cpp`: header layout, invalid marker, quantisation bound (≤ 0.01 mm) and stride sampling on a synthetic cloud. It also checks that a round-trip error computation matches a direct computation.
- `tests/test_cli.cpp`: option 6 against the existing fake server writes a site with `index.html`, the manifest and one frame, with the open command set to `true`. With the command set to `false`, it prints the note and continues.
- Python `tests/python/test_overview_outputs.py` (unittest; run by meson `test()` when `python3` is found): CSV/XLSX columns, empty cells for missing GT and failed rows. It feeds canned benchmark JSON and never runs the binary. XLSX tests skip when openpyxl is absent.
- A manual check on real SCARED data is listed in tasks (the dataset is not in CI).

## Risks / Trade-offs

- [Vendored three.js grows the repo by ~700 KB] → pinned r128 with its licence file, stored in one place.
- [`file://` script loading differs between browsers] → only classic `<script>` tags are used (no modules, no fetch). The task list includes a check in Firefox and Chromium.
- [Sites can get large with many frames (~1.6 MB each)] → frames load lazily. `--viewer-stride` / `viewer.stride` trade detail for size, and the tool prints the site size.
- [The CLI depends on a template path in the source tree] → config override plus a clear error. Embedding can come later if installs appear.
- [The CPV1 format may need more channels (e.g. disparity, confidence)] → a version in the magic and a reserved flags byte.

## Migration Plan

Additive only. Existing flags, benchmark positional arguments, the Markdown output format and CLI options 0–5 are unchanged. Rollback means reverting the change. No data migrations.

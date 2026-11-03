# JSON-RPC API Documentation

The Cloud Point RPC server implements the **JSON-RPC 2.0** protocol over TCP.

## General Format

All requests and responses are JSON objects.

### Request
```json
{
  "jsonrpc": "2.0",
  "method": "<method_name>",
  "params": {},
  "id": <integer|string>
}
```
*`params` is currently ignored by all handlers but is valid per JSON-RPC 2.0.*

### Response (Success)
```json
{
  "jsonrpc": "2.0",
  "result": <method_specific_result>,
  "id": <matching_request_id>
}
```

### Response (Error)
```json
{
  "jsonrpc": "2.0",
  "error": {
    "code": <integer>,
    "message": "<string>"
  },
  "id": <matching_request_id>
}
```

---

## Conventions

- **Matrices:** Row-major storage. A 3×3 matrix `M` with rows `[r0, r1, r2]` serialises as a flat 9-element JSON array `[r0[0], r0[1], r0[2], r1[0], ...]`.
- **Units:** Translation in **metres**. No pixel units unless stated.
- **Extrinsics convention (OpenCV):** `x_right = R · x_left + T`. For a parallel rig with baseline `b`, `R = I` and `T = [-b, 0, 0]` (e.g. `[-0.06, 0, 0]` for a 6 cm baseline).
- **Image layout:** Top-left origin, row-major, packed channels (BGR order unless otherwise noted). Unity must flip GPU readback vertically before encoding.
- **Image data encoding:** Raw pixel bytes encoded as **Base64** (standard alphabet, no line breaks). The `type` field indicates channel layout.
- **Calibration arrays:** Plain JSON double arrays — **not** base64. Only image pixel data uses base64.

---

## Methods Served by the Unity Side

These methods are implemented server-side (in Unity, or in the C++ test mock in `src/server_main.cpp`).

---

### `get-available-methods`

Returns the list of method names registered on this server instance.

**Request:**
```json
{ "jsonrpc": "2.0", "method": "get-available-methods", "id": 0 }
```

**Response `result`:** `["method-name-1", "method-name-2", ...]`

---

### `get-stereo-calibration`

Returns full stereo rig calibration: intrinsics for both cameras, stereo rotation and translation.

**Request:**
```json
{ "jsonrpc": "2.0", "method": "get-stereo-calibration", "id": 1 }
```

**Response `result`:**
```json
{
  "left": {
    "camera_matrix": [fx, 0, cx,  0, fy, cy,  0, 0, 1],
    "dist_coeffs":   [k1, k2, p1, p2, k3]
  },
  "right": {
    "camera_matrix": [fx, 0, cx,  0, fy, cy,  0, 0, 1],
    "dist_coeffs":   [k1, k2, p1, p2, k3]
  },
  "rotation":    [r00, r01, r02,  r10, r11, r12,  r20, r21, r22],
  "translation": [tx, ty, tz],
  "image_size":  { "width": 640, "height": 480 }
}
```

Field details:

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `camera_matrix` | `double[]` | 9 | Row-major 3×3 intrinsic matrix: `[fx, 0, cx, 0, fy, cy, 0, 0, 1]` |
| `dist_coeffs` | `double[]` | 5 | Radial/tangential coefficients `[k1, k2, p1, p2, k3]` |
| `rotation` | `double[]` | 9 | Row-major rotation matrix R (left-to-right frame, OpenCV convention) |
| `translation` | `double[]` | 3 | Translation vector in metres |
| `image_size` | object | — | Sensor resolution before any rectification |

**Test mock defaults:** fx=fy=800, cx=320, cy=240, zero distortion, R=identity, T=[-0.06, 0, 0], 640×480.

---

### `get-image-pair`

Returns a synchronised stereo frame as two base64-encoded images.

**Request:**
```json
{ "jsonrpc": "2.0", "method": "get-image-pair", "id": 2 }
```

**Response `result`:**
```json
{
  "frame": 42,
  "left":  { "width": 640, "height": 480, "type": "BGR", "data": "<base64>" },
  "right": { "width": 640, "height": 480, "type": "BGR", "data": "<base64>" }
}
```

Field details:

| Field | Description |
|-------|-------------|
| `frame` | Monotonically increasing counter per server instance; wraps at `uint64_t` max. |
| `type` | Channel layout: `"BGR"` (3 ch), `"RGBA"` (4 ch), or `"DEPTH"` (1 ch float32). |
| `data` | Base64-encoded raw pixel bytes. Size = `width × height × channels`. |

Unity must supply images in top-left-origin row-major order; flip GPU readback vertically before encoding.

---

### `get-intrinsic-params` *(legacy)*

Returns left-camera intrinsic matrix as a flat 9-element double array.

**Request:**
```json
{ "jsonrpc": "2.0", "method": "get-intrinsic-params", "id": 3 }
```

**Response `result`:** `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`

9 plain JSON doubles, row-major 3×3. **Not base64.**

---

### `get-extrinsic-params` *(legacy)*

Returns a flat 16-element double array (4×4 row-major extrinsic matrix).

**Request:**
```json
{ "jsonrpc": "2.0", "method": "get-extrinsic-params", "id": 4 }
```

**Response `result`:** `[r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz, 0, 0, 0, 1]`

16 plain JSON doubles. **Not base64.**

---

## Client-Side Outputs

These are computed locally by the C++ client (`RpcClient`) from data received via the methods above. They are **not** JSON-RPC methods callable on the server.

---

### `get-cloud-point` *(computed by client — spec in progress)*

Reconstructs a dense 3-D point cloud from the rectified stereo pair. The `RpcClient::get_cloud_point()` method will call `get-stereo-calibration` and `get-image-pair`, run stereo rectification and disparity computation locally (OpenCV SGBM or CUDA stereo), and reproject to 3-D.

**Expected future result shape** (subject to change in Phase 2):
```json
{
  "width":  <int>,
  "height": <int>,
  "data":   "<base64-encoded float32 XYZ triplets, row-major>"
}
```

`data` will encode `width × height × 3` little-endian `float32` values (x, y, z in metres per pixel, `NaN` for invalid/occluded depth). Full specification and encoding details are deferred to Phase 2.

---

## C++ Client API (OpenCV build)

Declared in `include/cloud_point/cloud_point_client.hpp`, namespace `score`. Available only when the project is built with OpenCV (`HAVE_CLOUD_POINT_COMPUTE`).

### `CloudPointClient::compute_cloud()` / `compute_frame()`

```cpp
struct CloudPointClient::Frame {
    PointCloud cloud;
    cv::Mat    rectified_left_bgr;   // CV_8UC3, same width/height as cloud
};

[[nodiscard]] std::expected<PointCloud, Error> compute_cloud();
[[nodiscard]] std::expected<Frame, Error>      compute_frame();
```

Both fetch one image pair over RPC (`get-image-pair`), convert to gray, rectify, match and reproject with the settings given at construction. `compute_frame()` additionally rectifies the left colour image onto the same pixel grid (~5 ms), so `rectified_left_bgr` pixel `(r, c)` is the colour of cloud point `(r, c)`; `compute_cloud()` is the same pipeline without that step. Recoverable per-frame failures (RPC error, decode error) are returned as `Error`; calling either before `connect()` returns `Error` immediately. The CLI's option 6 uses `compute_frame()` to colour the viewer frame.

The reprojection step is a `PointCloudBuilder` constructed from the client's `cloud_point` settings (below), so the client, the CLI and `scared_dataset_benchmark` produce identical clouds from the same pair and settings.

### `PointCloudBuilder`

Declared in `include/cloud_point/point_cloud_builder.hpp`. `build()` runs, in this order: disparity offset correction, `cv::reprojectImageTo3D` with Q, the depth-range filter, then the optional outlier filter.

```cpp
class PointCloudBuilder {
  public:
    enum class Reason : std::uint8_t {
        valid            = 0,
        matcher_rejected = 1,
        depth_range      = 2,
        outlier_filter   = 3,
    };
    static const char *reason_name(Reason r) noexcept;   // "valid", "matcher_rejected", ...

    struct Options {
        struct OutlierFilter {
            bool   enabled;          // false
            int    window;           // 5   (odd, >= 3)
            double abs_threshold_m;  // 0.002 (>= 0)
            double rel_threshold;    // 0.03  (>= 0, times the median depth)
            int    min_neighbours;   // 6   (0 <= min_neighbours < window^2)
        };
        float         min_depth_m;          // 0.01
        float         max_depth_m;          // 10.0
        double        disparity_offset_px;  // 0.0 (finite)
        OutlierFilter outlier_filter;
        Options() noexcept;
        Options(float min_m, float max_m) noexcept;
    };

    explicit PointCloudBuilder(cv::Mat q, Options opts = Options{});
    [[nodiscard]] PointCloud build(const cv::Mat &disparity, cv::Mat *reason = nullptr) const;
    [[nodiscard]] const Options &options() const noexcept;
};
```

| Field | Meaning |
|-------|---------|
| `min_depth_m`, `max_depth_m` | Points with z outside this range are invalid (`Reason::depth_range`). |
| `disparity_offset_px` | Added to every raw disparity > 0 before reprojection (`corrected = raw + offset`); a corrected value ≤ 0 makes the pixel invalid (`Reason::matcher_rejected`). 0 leaves the disparity map untouched, so the output is byte-identical to a build without the option. |
| `outlier_filter.enabled` | Runs the neighbourhood-consistency filter after the depth-range filter. Off: the cloud is identical to the unfiltered one. |
| `outlier_filter.window` | Side of the square neighbourhood (the pixel itself is excluded). |
| `outlier_filter.abs_threshold_m`, `rel_threshold` | A point is invalid (`Reason::outlier_filter`) when \|z − median z of the valid neighbours\| > `max(abs_threshold_m, rel_threshold · median)`. |
| `outlier_filter.min_neighbours` | A point with fewer valid neighbours than this is invalid. |

Decisions are made against the unfiltered input, so the result does not depend on scan order; rows run under `cv::parallel_for_`. The filter only invalidates points, it never moves or adds one.

`build()` accepts `CV_16S` (SGBM fixed point, disparity × 16) or `CV_32F` (pixels) and stores invalid points as quiet NaN on all three coordinates. When `reason` is non-null it receives a `CV_8U` map of the disparity's size with one `Reason` per pixel: `matcher_rejected` for a raw or offset-corrected disparity ≤ 0 or a non-finite / sentinel reprojection, `depth_range`, `outlier_filter`, or `valid`. The constructor throws `std::invalid_argument` if `q` is not 4×4 `CV_64F`, `disparity_offset_px` is not finite, or the outlier filter is enabled with an even or < 3 window, a negative threshold or `min_neighbours` outside `[0, window²)`; `build()` throws on an unsupported disparity type.

### `diagnose_reconstruction()`

Declared in `include/cloud_point/reconstruction_diagnostics.hpp`. Computes the benchmark's diagnostics (see [SCARED benchmark](#scared-benchmark-scared_dataset_benchmark)) from a predicted cloud, a `CV_32FC3` rectified ground-truth map of the same size, and `ReconstructionDiagnosticsInputs { cv::Mat q; cv::Mat reason; int min_disparity; int num_disparities; double disparity_offset_px; float min_depth_m; float max_depth_m; }`. The returned `ReconstructionDiagnostics` holds `depth_bias` (vector of `DepthBin { lo_m, hi_m, count, bias_z_median_m, abs_3d_median_m }`), `bias_z_median_m`, `fit_disparity_offset_px` / `fit_disparity_offset_residual_m`, `fit_depth_scale` / `fit_depth_scale_residual_m`, `tail_over_5mm` / `tail_over_20mm` / `tail_over_50mm`, `max_3d_m`, `coverage_loss { out_of_search_range, image_border, matcher_rejected, depth_range, outlier_filter }` and `unscored_fraction`, in the same units as the JSON keys of the same name. `ReprojectionGeometry::from_q(q)` exposes the depth ↔ disparity relation encoded in Q (`f`, `q32`, `q33`, `fB`; `depth(d)`, `disparity(z)`).

### `ViewerFrameOptions`

```cpp
struct ViewerFrameOptions {
    int stride;                                   // >= 1, default 4
    ViewerFrameOptions() noexcept;                // stride = 4
    explicit ViewerFrameOptions(int step) noexcept;
};
```

Sampling stride for the viewer export: every `stride`-th pixel in each direction is kept. Samples are raw pixel values, never averaged, so error statistics recomputed from the export match the full-resolution ones. Stride 4 on a 1280×1024 cloud gives ~1.2 MB binary (~1.6 MB base64) per frame with ground truth.

### `encode_viewer_frame()`

```cpp
std::string encode_viewer_frame(const PointCloud &cloud,
                                const cv::Mat *gt_rectified,
                                const cv::Mat *rgb_rectified_bgr,
                                const ViewerFrameOptions &opts = ViewerFrameOptions{});
```

Encodes a cloud (rectified-left frame, metres) plus optional ground truth and colour as a base64 `CPV1` payload (format below).

| Parameter | Description |
|-----------|-------------|
| `cloud` | Source cloud; `width × height` grid in the rectified-left frame, metres. |
| `gt_rectified` | Optional `CV_32FC3` rectified ground-truth point map of the cloud's size, metres (as produced by `StereoRectifier::rectify_left_point_map`). `nullptr` or an empty `Mat` = no ground truth. |
| `rgb_rectified_bgr` | Optional `CV_8UC3` **BGR** rectified left image of the cloud's size (e.g. `Frame::rectified_left_bgr`). `nullptr` or an empty `Mat` = no colour. Stored as R,G,B in the payload. |
| `opts` | Sampling stride. |

Returns standard base64 (padded, no line breaks). Throws `std::invalid_argument` if `opts.stride < 1`, the cloud is empty or malformed, the sampled grid is empty or exceeds 65535 in either direction, or an optional input has the wrong type or size.

### `write_viewer_frame_js()`

```cpp
void write_viewer_frame_js(const std::string &path, const std::string &id,
                           const std::string &base64);
```

Writes the frame script `CloudViewer.addFrame("<id>", "<base64>");` to `path`. The viewer site loads such files as `frames/<id>.js` through a `<script>` tag (which works from `file://`, unlike `fetch()`). `id` must match `[A-Za-z0-9_.-]+` and is normally the file stem. Throws `std::invalid_argument` for an empty or invalid `id` and `std::runtime_error` when the file cannot be written.

Callers: `scared_dataset_benchmark --viewer-frame=PATH [--viewer-stride=N]` (with ground truth when `point_cloud.obj` exists) and CLI option 6 (colour only, timestamped id).

---

## CLI configuration: `cloud_point` section

Read by `load_config()` (`include/cloud_point_rpc/config.hpp`) into `CloudPointConfig` and used by CLI options 4–6 through `CloudPointClient`. Every key is optional; an omitted key takes its default. Validation failures throw `std::runtime_error` with a message naming the key.

| Key | Type | Default | Validation |
|-----|------|---------|------------|
| `algorithm` | string | `"gpu"` | `"gpu"` (falls back to CPU without CUDA) or `"cpu"` |
| `num_disparities` | int | `128` | positive multiple of 16 |
| `min_depth_m`, `max_depth_m` | double | `0.01`, `10.0` | `0 < min_depth_m < max_depth_m` |
| `ply_stride` | int | `1` | ≥ 1 |
| `wls_filter` | bool | `false` | — |
| `disparity_offset_px` | double | `0.0` | finite; `cloud_point.disparity_offset_px must be finite` |
| `outlier_filter.enabled` | bool | `false` | — |
| `outlier_filter.window` | int | `5` | odd and ≥ 3; `cloud_point.outlier_filter.window must be odd and >= 3` |
| `outlier_filter.abs_threshold_m` | double | `0.002` | ≥ 0; `cloud_point.outlier_filter.abs_threshold_m must be >= 0` |
| `outlier_filter.rel_threshold` | double | `0.03` | ≥ 0; `cloud_point.outlier_filter.rel_threshold must be >= 0` |
| `outlier_filter.min_neighbours` | int | `6` | `0 <= min_neighbours < window²`; `cloud_point.outlier_filter.min_neighbours must be in [0, window^2)` |

The outlier-filter keys are validated even when `enabled` is false, so a typo does not surface only when the filter is switched on. `disparity_offset_px` and `outlier_filter.*` map one-to-one onto `PointCloudBuilder::Options` (above). The SGBM matcher parameters (`CpuStereoMatcher::Params`: block size, uniqueness, speckle window and range, LR tolerance, pre-filter cap, median kernel, WLS lambda/sigma) have no YAML keys yet; they are benchmark flags only.

```yaml
cloud_point:
  algorithm: cpu
  num_disparities: 160
  min_depth_m: 0.02
  max_depth_m: 0.30
  disparity_offset_px: 0.0
  outlier_filter:
    enabled: false
    window: 5
    abs_threshold_m: 0.002
    rel_threshold: 0.03
    min_neighbours: 6
```

---

## SCARED benchmark (`scared_dataset_benchmark`)

```
scared_dataset_benchmark [flags] <keyframe_dir> [num_disparities] [depth_png|-] [min_depth_m max_depth_m]
```

Positional arguments: the keyframe directory, the number of disparity levels (default 160, positive multiple of 16), an optional colour-mapped depth PNG path (`-` to skip) and an optional depth range (both or neither; without it the builder defaults 0.01–10 m apply). `--key=value` flags may appear anywhere. An unknown or malformed flag prints `Invalid option: ...` plus the usage text and exits with status 1 before the dataset is loaded; the settings are validated with the same constructors the pipeline uses. Without flags the settings equal the CLI defaults (`CpuStereoMatcher::Params()` / `PointCloudBuilder::Options()`).

| Flag | Sets |
|------|------|
| `--viewer-frame=PATH`, `--viewer-stride=N` | viewer frame export (see `write_viewer_frame_js`); `N` ≥ 1, default 4 |
| `--block-size=N`, `--uniqueness=N`, `--speckle-window=N`, `--speckle-range=N`, `--disp12-max-diff=N`, `--pre-filter-cap=N`, `--median-kernel=N` | `CpuStereoMatcher::Params` `block_size`, `uniqueness_ratio`, `speckle_window_size`, `speckle_range`, `disp12_max_diff`, `pre_filter_cap`, `median_kernel` (integers; uniqueness, speckle window and range ≥ 0, pre-filter cap ≥ 1) |
| `--wls` / `--no-wls` | `wls_filter` (no value) |
| `--disparity-offset=PX` | `PointCloudBuilder::Options::disparity_offset_px` (finite number) |
| `--outlier-filter` / `--no-outlier-filter` | `outlier_filter.enabled` (no value) |
| `--outlier-window=N`, `--outlier-abs-m=M`, `--outlier-rel=R`, `--outlier-min-neighbours=N` | `outlier_filter.window`, `abs_threshold_m`, `rel_threshold`, `min_neighbours` |

### JSON output

One JSON object on stdout. Always present: `keyframe_dir`, `algorithm` (`"StereoSGBM"`), `num_disparities`, `min_depth_m`, `max_depth_m`, `image_width`, `image_height`, `valid_points`, `valid_fraction`, `z_min_m`, `z_p05_m`, `z_median_m`, `z_p95_m`, `z_max_m`, `matching_ms`, `reconstruction_ms`, `has_ground_truth`, `settings`; `depth_png` and `viewer_frame` echo the paths when written.

`settings` holds every effective reconstruction setting: `min_disparity`, `num_disparities`, `block_size`, `uniqueness_ratio`, `speckle_window_size`, `speckle_range`, `disp12_max_diff`, `pre_filter_cap`, `median_kernel`, `wls_filter`, `wls_lambda`, `wls_sigma`, `min_depth_m`, `max_depth_m`, `disparity_offset_px` and `outlier_filter { enabled, window, abs_threshold_m, rel_threshold, min_neighbours }`.

With `point_cloud.obj` in the keyframe the evaluator's metrics are added (`ground_truth_points`, `matched_points`, `coverage`, `mae_x_m`, `mae_y_m`, `mae_z_m`, `mae_3d_m`, `rmse_3d_m`, `median_3d_m`, `within_1mm`, `within_2mm`, `within_5mm`) together with the diagnostics below. Units are SI (metres, fractions, milliseconds).

| Key | Meaning |
|-----|---------|
| `depth_bias` | Array of 10 mm bins keyed by ground-truth depth, covering the depth range; bins with fewer than 200 matched points are omitted. Each: `lo_m`, `hi_m`, `count`, `bias_z_median_m` (median `z_rec − z_gt`), `abs_3d_median_m`. |
| `bias_z_median_m` | Median signed depth error over all matched points. |
| `fit_disparity_offset_px`, `fit_disparity_offset_residual_m` | Offset model: δ = median(d_gt − d_rec) over matched points with 3-D error < 20 mm (d from Q); residual = RMS over the bins of the bin median signed error left after adding δ. |
| `fit_depth_scale`, `fit_depth_scale_residual_m` | Scale model: s = median(z_rec / z_gt) on the same points; residual as above after dividing z by s. |
| `tail_over_5mm`, `tail_over_20mm`, `tail_over_50mm` | Fractions of matched points whose 3-D error exceeds 5 / 20 / 50 mm. |
| `max_3d_m` | Largest 3-D error over matched points. |
| `coverage_loss` | Object with `out_of_search_range`, `image_border`, `matcher_rejected`, `depth_range`, `outlier_filter`: fractions of valid ground-truth pixels, each unmatched pixel assigned to the first cause that applies in that order (`out_of_search_range`: ground-truth disparity minus the applied offset outside `[min_disparity, min_disparity + num_disparities)`; `image_border`: column `x < min_disparity + num_disparities`, SGBM's invalid left band; the rest from the builder's `Reason` map). `coverage` + the five fractions = 1. |
| `unscored_fraction` | Fraction of valid reconstructed points whose ground-truth pixel is non-finite. |

`scripts/scared_overview.py` maps these onto its CSV/XLSX columns `bias_z_median_mm`, `fit_disparity_offset_px`, `tail_over_5mm_pct`, `tail_over_20mm_pct`, `max_3d_mm`, `lost_search_range_pct` (`coverage_loss.out_of_search_range`) and `lost_matcher_pct` (`coverage_loss.matcher_rejected`), forwards `--bench-arg=FLAG` to every benchmark call, and uses `fit_disparity_offset_px`, `matched_points`, `mae_3d_m`, `bias_z_median_m` and `settings.disparity_offset_px` for `--fit-offset`.

---

## Viewer frame format (CPV1)

Shared by the C++ writer (`encode_viewer_frame`), the Python site builder (`scripts/cloud_viewer/build_site.py`) and the viewer page (`scripts/cloud_viewer/site/viewer.js`).

### Payload

Binary, little-endian, then base64-encoded (standard alphabet, padded, no newlines):

| Offset | Type | Field |
|--------|------|-------|
| 0 | `char[4]` | Magic `"CPV1"` |
| 4 | `u16` | `width` — sampled grid columns = `floor(cloud.width / stride)` |
| 6 | `u16` | `height` — sampled grid rows = `floor(cloud.height / stride)` |
| 8 | `u16` | `stride` |
| 10 | `u8` | `flags`: bit0 = has ground truth, bit1 = has RGB |
| 11 | `u8` | reserved (0) |
| 12 | `int16[width*height*3]` | `xyz` — reconstructed X,Y,Z, row-major; sample `(r, c)` = cloud pixel `(r*stride, c*stride)`, no averaging |
| … | `int16[width*height*3]` | `gt_xyz` — only if bit0; rectified-left ground truth at the same pixel |
| … | `u8[width*height*3]` | `rgb` — only if bit1; rectified left image colour at the same pixel, order R,G,B |

- **Units:** XYZ in 0.02 mm (`value_m / 2e-5`, rounded to nearest, clamped to `[-32767, 32767]`, i.e. ±655 mm). Quantisation error is ≤ 0.01 mm.
- **Invalid:** a pixel with any non-finite component (or an all-NaN ground-truth pixel) stores `-32768` (`INT16_MIN`) in all three components. The viewer neither draws nor scores such samples.
- **Coordinates:** rectified-left camera frame, x right, y down, z forward.

### `frames/<id>.js` (one per frame, `<id>` matches `[A-Za-z0-9_.-]+`)

```js
CloudViewer.addFrame("<id>", "<base64 payload>");
```

### `frames/manifest.js` (always rewritten with the full frame list)

```js
CloudViewer.setManifest({
  "title": "…",
  "generated": "<ISO-8601 timestamp>",
  "frames": [
    { "id": "<id>", "label": "<human label>", "has_gt": true,
      "metrics": { "valid_fraction": 0.84, "z_p05_m": 0.052, "z_median_m": 0.059, "z_p95_m": 0.074,
                   "matching_ms": 339.3, "reconstruction_ms": 58.1,
                   "coverage": 0.855, "mae_3d_m": 0.00084, "mae_z_m": 0.00076, "rmse_3d_m": 0.00143,
                   "median_3d_m": 0.00059, "within_1mm": 0.627, "within_2mm": 0.799, "within_5mm": 0.846 } }
  ]
});
```

`metrics` holds the benchmark JSON keys listed above in the benchmark's SI units (metres, fractions, milliseconds); any key may be absent and the viewer shows only those present. The argument is valid JSON, so `json.dumps` / `nlohmann::json::dump` produce it. The CLI additionally keeps `frames/index.json` (the same entries as a plain JSON array) so that it can rebuild the manifest from all frames still present in `frames/`.

### Site layout

```
index.html  viewer.css  viewer.js
vendor/three.min.js  vendor/OrbitControls.js  vendor/LICENSE-three.txt
frames/manifest.js  frames/<id>.js ...
```

`index.html` loads the vendor scripts, `viewer.js` and `frames/manifest.js` with plain `<script>` tags. `viewer.js` defines `window.CloudViewer = { setManifest, addFrame }` and loads `frames/<id>.js` lazily by inserting a `<script>` tag when a frame is first selected. No `fetch()`, no ES modules, no network URLs, so a site works from `file://` and can be moved as a directory. The template lives in `scripts/cloud_viewer/site/`; `build_site.py` (Python) and CLI option 6 (C++) copy it and add `frames/`.

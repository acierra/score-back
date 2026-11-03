# Tasks

> **Execution strategy.** Delegate each group to an explicit `Agent` subagent with `model: "fable"`. Brief each subagent with the group's tasks, the relevant spec and design sections, and "do not commit anything, including planning artifacts".
>
> - **Order.** Groups 1 → 2 → 3 run sequentially: they share `PointCloudBuilder` and the benchmark. Group 4 (Python) can run in parallel with group 3 once group 2's JSON keys exist.
> - **Measurement runs.** Group 5 is run by the orchestrator, not a subagent. Its decisions compare numbers against the baseline and must be reported to the user.
> - **Docs.** Group 6 comes last.
> - **Build and test.** Use `build-opencv/`. Run `meson test -C build-opencv` for all tests.

## 1. Builder: offset, outlier filter, build trace

- [x] 1.1 Extend `PointCloudBuilder::Options`:
  - add `disparity_offset_px` (default 0), applied only to raw disparity > 0, with corrected ≤ 0 ⇒ invalid;
  - add an optional `cv::Mat* reason` out-parameter on `build()`: `CV_8U`, with codes for valid / matcher_rejected / depth_range / outlier_filter.

  Verify with new `tests/test_point_cloud_builder.cpp` cases:
  - offset 0 gives a byte-identical cloud;
  - a +0.5 px offset gives the depth of 50.5 px;
  - 0.25 px with a −0.5 px offset gives an invalid pixel;
  - the reason codes are correct for each invalidation path.
- [x] 1.2 Add the outlier filter (options `enabled`, `window`, `abs_threshold_m`, `rel_threshold`, `min_neighbours`; default disabled) after the depth-range filter. It decides against the unfiltered input and runs rows under `cv::parallel_for_`.

  Verify with unit tests:
  - an isolated 200 mm spike on an 80 mm plane is removed and nothing else changes;
  - a 45° plane interior is untouched;
  - with the filter disabled, the cloud is identical;
  - invalid windows are rejected;
  - the timing of a 1280×1024 synthetic cloud is logged.

## 2. Diagnostics and benchmark

- [x] 2.1 Add a diagnostics unit (e.g. `include/cloud_point/reconstruction_diagnostics.hpp` and `src/cloud_point/reconstruction_diagnostics.cpp`). It takes the predicted cloud, the ground truth, Q, the reason map and the matcher range, and computes:
  - depth-binned signed bias (10 mm bins, ≥ 200 points);
  - `bias_z_median_m`;
  - offset and scale fits with residuals, per design D3;
  - tail fractions and `max_3d_m`;
  - `coverage_loss` in the D4 order;
  - `unscored_fraction`.

  Verify with `tests/test_reconstruction_diagnostics.cpp`, using synthetic inputs:
  - a −0.1 px disparity shift recovers δ = +0.1 ± 0.01 with the offset residual below the scale residual;
  - a ×1.01 depth scale recovers s = 1.01 ± 0.001;
  - the 10-in-10 000 tail case;
  - the losses plus coverage equal 1 ± 1e-9;
  - near pixels beyond the search range are attributed to `out_of_search_range`.
- [x] 2.2 Add the tuning flags from design D6 to `scared_dataset_benchmark`:
  - construct `CpuStereoMatcher` directly from `Params`;
  - pass offset and filter options to the builder;
  - emit `settings` and all new JSON keys;
  - fail on unknown or malformed flags before loading the dataset.

  Verify:
  - `scared_dataset_benchmark datasets/scared/dataset_1/keyframe_1 160 - 0.02 0.30` without flags prints the same headline metrics as before (MAE 0.84 mm, RMSE 1.43 mm);
  - `--outlier-window=abc` exits non-zero with usage;
  - `--speckle-window=800` shows up in `settings`.

## 3. Configuration and client wiring

- [x] 3.1 Add `cloud_point.disparity_offset_px` and `cloud_point.outlier_filter.{enabled,window,abs_threshold_m,rel_threshold,min_neighbours}` to `include/cloud_point_rpc/config.hpp`. Validation names the offending key. Verify with `tests/test_config.cpp` cases:
  - defaults when the keys are omitted;
  - overrides are read;
  - an even window, a negative threshold, a non-finite offset and an oversized `min_neighbours` are each rejected by name.
- [x] 3.2 Thread the new options through `CliStereoOptions`, `src/main.cpp` and `CloudPointClient`'s `PointCloudBuilder::Options`. Verify with a `tests/test_cloud_point_client.cpp` case that the client and a directly built `PointCloudBuilder` with the same settings produce identical clouds from the same pair. This is the spec scenario "Benchmark matches client".
- [x] 3.3 Add commented examples of the new keys to `config.yaml.example` and `config.scared.yml`, keeping the current values. Verify that both files still load in `tests/test_config.cpp`, or with a CLI start that parses them.

## 4. Overview tool

- [x] 4.1 Add the new `COLUMNS` entries at the end of `scripts/scared_overview.py`: median signed z bias (mm), fitted disparity offset (px), tail > 5 mm %, tail > 20 mm %, max 3D error (mm), lost to search range %, lost to matcher %. They stay empty without ground truth and for failed keyframes. Verify with `tests/python/test_overview_outputs.py` canned-JSON cases for GT, no-GT and failed rows, in both CSV and XLSX.
- [x] 4.2 Add a repeatable `--bench-arg=FLAG` option: flags are forwarded in order to every benchmark call and listed on the XLSX `run` sheet, and a benchmark rejection yields a `failed` row. Verify with a unit test using a stub benchmark that echoes its argv, plus an XLSX `run`-sheet assertion.
- [x] 4.3 Add `--fit-offset`:
  - point-weighted combined offset;
  - leave-one-dataset-out table (before/after MAE and signed bias per held-out dataset), re-running the held-out keyframes with `--disparity-offset`;
  - a single-dataset message;
  - no file writes.

  Verify with a stub-benchmark unit test for the aggregation, the holdout and the single-dataset path, and with `git status` unchanged after a real run.

## 5. Measurement and default decisions (orchestrator)

- [x] 5.1 Record the baseline: run `scripts/scared_overview.py --depth-range 0.02 0.30 --csv out/quality/baseline.csv --xlsx out/quality/baseline.xlsx datasets/scared/dataset_*/keyframe_* datasets/scared/test_dataset_8/keyframe_*`. Verify that the headline numbers match the README results table and that the new bias, tail and loss columns are filled for the 7 ground-truth keyframes.
- [x] 5.2 Bias: run `--fit-offset` on the 7 ground-truth keyframes, compare the offset and scale residuals, and apply the D7 offset target. If it passes, set `disparity_offset_px` in `config.scared.yml`. Either way, record the fitted δ per dataset, the holdout table and the decision in `out/quality/decisions.md`. Verify the decision against the D7 criteria in that file.
- [x] 5.3 Outliers:
  - sweep speckle window {100, 200, 400, 800, 1600} × range {1, 2, 4} with the filter off;
  - then sweep outlier-filter window {5, 7} × abs {1, 2, 4 mm} × rel {0.02, 0.05} on the best speckle setting, via `--bench-arg`;
  - pick by the D7 outlier target, and inspect dataset_1 keyframe_3 and dataset_3 in the viewer's error colouring for removed true edges;
  - update the `config.scared.yml` defaults only if the target is met.

  Verify with the sweep CSVs under `out/quality/` and the decision recorded in `decisions.md`.
- [x] 5.4 Coverage: run with 160, 192 and 224 disparity levels, re-checking `min_depth_m` for each. Compare `out_of_search_range`, `image_border`, coverage and matching time, and apply the D7 range target. Verify with `decisions.md` entries showing the dataset_2 coverage delta and the runtime delta.
- [x] 5.5 Final run with the chosen SCARED settings, written to `out/quality/final.{csv,xlsx}` with `--view --no-open`. Verify that every D7 criterion holds against `baseline.csv` for each adopted default, that `meson test -C build-opencv` passes, and that the viewer shows the reduced bias in its depth-bias chart.

## 6. Documentation sync

- [x] 6.1 Update `README.md`:
  - pipeline stages: the offset and outlier filter;
  - results table: the final numbers next to the baseline;
  - a short "Reconstruction diagnostics" section on bias bins, fits, tail and coverage loss, plus `--bench-arg` and `--fit-offset`;
  - the note that the SCARED offset corrects towards SCARED's ground truth;
  - the PlantUML pipeline diagram, updated with the new stages.

  Verify by running each documented command once.
- [x] 6.2 Update `API.md` with the new `cloud_point` config keys, the `PointCloudBuilder::Options` fields, the benchmark flags and the new benchmark JSON keys. Verify the key names by grepping them against `config.hpp` and the benchmark source.
- [x] 6.3 Leave generated OpenWiki pages to the scheduled regeneration and do not hand-edit them. Verify that `git diff --stat openwiki/` is empty.

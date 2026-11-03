# Design

## Context

Current pipeline, in client, CLI and benchmark:

```
rectify (stereoRectify, CALIB_ZERO_DISPARITY, alpha=0)
  -> CpuStereoMatcher (SGBM, 160 levels; LRC 1 px, uniqueness 10,
                       speckle window 100 / range 2; optional median, WLS)
  -> PointCloudBuilder::build (reprojectImageTo3D with Q, depth range filter)
  -> PointCloudEvaluator::evaluate (benchmark only; unsigned metrics)
```

Facts that shape this design:

- **Where the pieces live.** `PointCloudBuilder` is the one place all entry points share after matching. `CloudPointClient` and `scared_dataset_benchmark` both build one.
- **Benchmark tuning.** The benchmark builds its matcher from `StereoMatcherFactory` defaults, so every SGBM parameter is fixed at compile time.
- **Evaluator output.** `PointCloudEvaluator` returns only unsigned aggregates. The signed, depth-binned view exists only in the viewer's JavaScript chart.
- **Rig geometry.** On SCARED, fx ≈ 1035 px and the baseline B ≈ 4.14 mm, so f·B ≈ 4290 px·mm. A disparity error δ therefore moves depth by about z²·δ / (f·B):

  | Depth | Depth error per 0.1 px of disparity error |
  |---|---|
  | 60 mm | 0.08 mm |
  | 150 mm | 0.52 mm |

- **What the bias fits.** The measured dataset_3 bias divided by z² is roughly constant: 0.3/60² ≈ 8e-5 and 1.9/150² ≈ 8e-5 mm⁻¹. Divided by z it is not: 0.005 at 60 mm against 0.013 at 150 mm. So a disparity offset of about 0.1 px is the leading hypothesis and a focal or baseline scale error the alternative. The diagnostics must discriminate between them rather than assume one.
- **Where near tissue ends.** At 160 levels the nearest reconstructable depth is f·B / 160 ≈ 27 mm.

## Goals / Non-Goals

**Goals:**
- Make each quality problem measurable per keyframe:
  - bias as a function of depth;
  - the error tail;
  - lost coverage, with its cause.
- Add correction and filtering steps that every entry point shares, each off by default in code.
- Change `config.scared.yml` defaults only when a candidate meets the acceptance targets below, measured against a recorded baseline.

**Non-Goals:**
- Re-calibrating the SCARED rigs or editing `endoscope_calibration.yaml`.
- Hole filling, inpainting or any step that creates points the matcher did not measure. The WLS path already masks its extrapolations for the same reason.
- New matchers (learned stereo, GPU SGM tuning), multi-frame fusion, and changes to the RPC protocol.
- Making WLS a default. Its accuracy cost is already documented.

## Decisions

### D1. Measure first, then change defaults, all against a recorded baseline
Diagnostics (tasks group 1–2) land before any fix. The first run records a baseline CSV/XLSX of all SCARED keyframes to `out/quality/baseline.*`.

Each later candidate is judged against that file with the acceptance targets in D7.

*Alternative:* tune directly on the headline metrics. Rejected: RMSE alone cannot tell a better outlier filter from one that simply drops hard pixels. Coverage attribution and tail counts show that.

### D2. Disparity offset applied inside `PointCloudBuilder`, before reprojection
`PointCloudBuilder::Options` gains `disparity_offset_px` (default 0).

The offset is added only to valid disparities (raw > 0), so SGBM's invalid sentinel stays invalid. A corrected value ≤ 0 becomes invalid.

Sign convention: corrected = raw + δ. A positive δ means larger disparities and nearer points. The observed "too far" bias predicts δ ≈ +0.07 to +0.1 px.

*Alternative:* fold δ into Q, since with CALIB_ZERO_DISPARITY adding δ to d is equivalent to adding a term to Q(3,3). Rejected: it hides the correction inside a matrix that other code (rectifier tests, API.md) treats as pure calibration. It also cannot apply the "non-positive ⇒ invalid" rule.

*Alternative:* correct z after reprojection with a fitted polynomial. Rejected: it is not physically grounded and does not transfer across rigs.

### D3. Calibration-error fits: robust per-point estimates
For each matched point with 3-D error below 20 mm, work in disparity space:
- d_g = f·B / z_gt is the ground-truth disparity.
- d_m = f·B / z_rec is the measured disparity.
- f·B is taken from Q.

The two models are then:
- **Offset model:** `fit_disparity_offset_px` is the median of (d_g − d_m).
- **Scale model:** `fit_depth_scale` is the median of (z_rec / z_gt).

Each model's residual is the RMS, over the depth bins, of the bin median signed error left after applying that model. Medians give the robustness the spec requires, and the 20 mm cut keeps gross mismatches out entirely.

*Alternative:* least squares over all points. Rejected: the 50–270 mm tail would dominate it.

*Alternative:* a joint (δ, s) fit. Deferred: two separate one-parameter fits with residuals already answer "which explains the data". A joint fit on 7 keyframes invites overfitting.

### D4. Coverage attribution through a per-pixel build trace
`PointCloudBuilder::build` gains an optional out-parameter, a `CV_8U` reason map. It records why each pixel is invalid:
- `matcher_rejected` (raw disparity ≤ 0)
- `depth_range`
- `outlier_filter`
- valid

The diagnostics code then classifies each ground-truth pixel with no reconstruction:
1. `out_of_search_range`: d_g − δ lies outside [minDisparity, minDisparity + numDisparities).
2. `image_border`: x < minDisparity + numDisparities, the left-image band where the search range extends beyond the right image. This is exactly the band `cv::StereoSGBM` emits as invalid regardless of scene content, so widening `numDisparities` moves loss from `matcher_rejected` into `image_border` instead of disguising it as matcher failures.
3. Otherwise, the trace's reason.

The geometric causes come first because the matcher cannot succeed on those pixels whatever its tuning.

`unscored_fraction` is the fraction of valid points whose ground-truth pixel is non-finite.

*Alternative:* infer causes by re-running stages in the benchmark. Rejected: it would duplicate the builder logic and could drift from it.

### D5. Outlier filter: depth-median consistency on the image grid
The filter is part of `PointCloudBuilder` and runs after the depth-range filter. Keeping it there means the client, CLI and benchmark cannot diverge.

For each valid pixel it:
1. gathers the valid z values in a `window × window` neighbourhood, excluding the pixel itself;
2. invalidates the pixel if there are fewer than `min_neighbours` of them;
3. otherwise computes their median z̃ and invalidates the pixel when |z − z̃| > max(abs_threshold_m, rel_threshold · z̃).

Decisions are made against the unfiltered input, not updated in place, so the result does not depend on scan order. Rows run under `cv::parallel_for_`.

Starting defaults, to be revisited by the sweep: window 5, abs 2 mm, rel 0.03, min_neighbours 6.

*Alternatives:*
- *Tuning only SGBM's speckle filter.* Kept as a sweep candidate, since it is free at runtime. On its own it may not suffice: speckle works on disparity connectivity, and large wrong blobs pass it.
- *PCL-style statistical outlier removal (kNN).* Rejected: it needs a spatial index, costs much more time and ignores the grid structure we already have.
- *`cv::medianBlur` on z.* Rejected: it cannot handle NaN holes, and it moves points instead of rejecting them.

### D6. Benchmark and overview-tool surface
**Benchmark flags.** `scared_dataset_benchmark` extends its existing `--viewer-*` extraction loop to `--key=value` flags:

| Group | Flags |
|---|---|
| Matcher | `--block-size`, `--uniqueness`, `--speckle-window`, `--speckle-range`, `--disp12-max-diff`, `--pre-filter-cap`, `--median-kernel` |
| WLS | `--wls` / `--no-wls` |
| Offset | `--disparity-offset` |
| Outlier filter | `--outlier-filter` / `--no-outlier-filter`, `--outlier-window`, `--outlier-abs-m`, `--outlier-rel`, `--outlier-min-neighbours` |

The matcher is constructed directly with `CpuStereoMatcher::Params`. Defaults come from the same `Params()` / `Options()` constructors the config loader uses, so "no flags" equals the CLI defaults.

Existing positional arguments and JSON keys keep their meaning. New keys are additive: `settings`, `depth_bias`, `bias_z_median_m`, `fit_*`, `tail_*`, `max_3d_m`, `coverage_loss`, `unscored_fraction`.

**Overview tool.** `scared_overview.py` gains:
- the new `COLUMNS` entries;
- `--bench-arg=FLAG` (repeatable), recorded on the XLSX `run` sheet;
- `--fit-offset`, which does the aggregation and leave-one-dataset-out check in Python from the per-keyframe JSON.

The dataset id is the `dataset_N` path component, and keyframes are weighted by matched points. For the holdout it re-runs the held-out keyframes with `--disparity-offset=<fit of the other datasets>`.

**Config keys.** `cloud_point.disparity_offset_px` and `cloud_point.outlier_filter.{enabled,window,abs_threshold_m,rel_threshold,min_neighbours}` go into `CloudPointConfig` and flow through to the `PointCloudBuilder::Options` that `CloudPointClient` constructs.

### D7. Acceptance targets for changing `config.scared.yml` defaults
All targets are measured on the 7 ground-truth keyframes against `out/quality/baseline.csv`. test_dataset_8 has no ground truth; its valid fraction must not fall by more than 2 pp.

| Candidate | Ships as SCARED default only if |
|---|---|
| Disparity offset | the leave-one-dataset-out check reduces \|median signed bias\| on every held-out dataset, and no keyframe's MAE worsens by more than 2 % |
| Speckle re-tune and/or outlier filter | mean RMSE over the keyframes falls by at least 30 %; no keyframe loses more than 1 pp coverage or 0.5 pp within-2 mm; no MAE worsens; build+filter time rises by 30 ms or less per frame |
| Wider disparity range | dataset_2 coverage rises by at least 3 pp; no keyframe's MAE worsens by more than 2 %; matching time rises by 40 % or less |

A candidate that misses its target stays available as an opt-in setting. Its measured numbers are documented in the README. That is a valid outcome of this change, not a failure.

A wider range pushes the nearest depth below 20 mm (224 levels ⇒ about 19 mm), so `min_depth_m` is re-checked together with it.

## Risks / Trade-offs

- **[Risk] The bias is in SCARED's ground truth, not in our pipeline.** SCARED's structured-light ground truth carries its own calibration error. → The code default stays 0. The holdout check guards transfer. The offset ships only in the SCARED config, and the README states that it corrects towards SCARED's ground truth.
- **[Risk] Overfitting to 7 keyframes from 3 datasets.** → Leave-one-dataset-out for the offset, coarse sweep grids, and the documented target of a 30 % mean-RMSE reduction rather than a per-keyframe optimum.
- **[Risk] The outlier filter deletes real depth edges**, such as tissue folds and instrument silhouettes. → The relative threshold scales with depth, the slope scenario is a unit test, and the filtered frames are inspected in the viewer's error colouring before a default changes.
- **[Risk] Sub-pixel "pixel locking" in SGBM.** Disparities bunch near integers, so the error is not a pure constant offset. → The offset model's residual reveals it. If it stays large, record it as a finding; fixing SGBM's sub-pixel interpolation is out of scope.
- **[Trade-off] A wider disparity range costs matching time** roughly in proportion (about 325 ms → about 455 ms at 224 levels). It also widens the invalid left band. → The attribution shows `image_border` growth next to `out_of_search_range` shrinkage, and D7 caps the time.
- **[Risk] Filter cost on 1280×1024.** → parallel_for_, a small window and a 30 ms budget measured in the benchmark's `reconstruction_ms`.

## Migration Plan

This change is purely additive:
- Configs without the new keys behave exactly as before.
- JSON consumers that ignore unknown keys are unaffected.
- CSV/XLSX gain columns at the end, so existing column positions are unchanged.

Rollback means removing the keys from `config.scared.yml`.

# Proposal

## Why

The SCARED overview now makes reconstruction errors visible. It shows three separate problems:

**1. Systematic depth bias.**
- The median signed depth error (reconstructed z − ground-truth z) is positive on all 7 ground-truth keyframes, between +0.2 and +0.9 mm.
- On dataset_3 the bias grows with depth: about +0.3 mm at 60 mm, +1.1 mm at 90 mm, +1.5 mm at 110 mm and +1.9 mm at 150 mm.
- The growth fits dz ∝ z², not dz ∝ z. That is the pattern of a constant disparity offset of roughly 0.1 px. A baseline or focal-length scale error would give dz ∝ z instead.

**2. Heavy-tailed outliers.**
- Fewer than 1.5 % of points on dataset_1 are off by more than 5 mm, yet single errors reach 50–270 mm.
- These few points dominate RMSE. dataset_1 keyframe_3 has MAE 1.09 mm but RMSE 8.15 mm.
- They survive SGBM's left-right, uniqueness and speckle checks. The speckle window is 100 px², a tiny area on a 1280×1024 image.

**3. Coverage gaps.**
- 12–26 % of ground-truth pixels get no reconstruction. dataset_2 is worst at 73.9 %.
- On dataset_2, near tissue needs more than the 160-disparity search range, which ends at z ≈ 27 mm.
- The benchmark does not say why a pixel is missing, so the gaps can't be fixed on purpose.

These were deferred from `add-cloud-viewer-and-reports` as their own change. The CSV/XLSX tables and the viewer now exist, so each fix can be measured against a recorded baseline.

## What Changes

**Diagnostics come first.** Each fix is chosen and tuned from data rather than guessed.
- The benchmark reports signed depth bias per depth bin.
- It fits a disparity-offset model and a scale model to that bias, with residuals.
- It counts the error tail: points above 5, 20 and 50 mm.
- It attributes lost coverage to a cause: outside the disparity search range, rejected by the matcher, outside the depth range, removed by the outlier filter, or image border.
- The matcher and filter parameters become benchmark flags, so sweeps no longer need a rebuild.

**Disparity offset correction.**
- A configurable sub-pixel offset is added to the disparity before reprojection. It addresses the depth-dependent bias.
- A fitting mode estimates the offset from ground-truth keyframes, with a leave-one-dataset-out check against overfitting.
- The shipped default stays 0 (no change) unless the fitted value transfers across datasets. In that case `config.scared.yml` sets it.

**Outlier filter.**
- A new opt-in step runs after reprojection. It invalidates points whose depth disagrees with the local neighbourhood median.
- The speckle-window defaults are re-tuned from a parameter sweep.
- New defaults ship only if they meet the acceptance targets in design.md, e.g. lower RMSE with at most 1 percentage point of coverage loss.

**Coverage.**
- The disparity search range can be set per run, e.g. 192 or 224 levels. The runtime cost is measured.
- `config.scared.yml` picks a value from the coverage-attribution data.
- Hole filling that invents geometry is explicitly out of scope.

**Reports.**
- The overview tool's CSV/XLSX tables gain bias and outlier-tail columns.
- The overview tool passes matcher and filter flags through to the benchmark.

**Not breaking.** Every new option defaults to current behaviour in code. Only `config.scared.yml` may change its defaults, and only on evidence.

## Capabilities

### New Capabilities
- `reconstruction-accuracy`: the tunable post-processing that turns a disparity map into a cloud. This covers the disparity offset correction, the neighbourhood-consistency outlier filter and the configurable disparity search range, with their configuration, defaults and validation.
- `reconstruction-diagnostics`: what the SCARED benchmark measures beyond the headline metrics. This covers the depth-binned signed bias, the offset and scale fits, the error-tail counts, the coverage-loss attribution, the offset-fitting mode and the tuning flags.

### Modified Capabilities
- `quality-reports`:
  - The "Metrics table columns" requirement gains bias and outlier-tail columns.
  - The overview tool forwards reconstruction-tuning flags to the benchmark. Those flags are recorded in the XLSX run-parameters sheet.

## Impact

- **C++**:
  - `PointCloudBuilder` gets a disparity offset option and the outlier filter. The filter could instead be a new `PointCloudFilter` unit; design.md decides.
  - `CpuStereoMatcher::Params` defaults may be re-tuned.
  - `PointCloudEvaluator` gains signed and binned statistics and coverage attribution.
  - `scared_dataset_benchmark.cpp` gains new flags and JSON keys.
  - `CloudPointClient` and `config.hpp` gain new `cloud_point:` keys.
- **Python**: `scripts/scared_overview.py` gets new columns and flag pass-through.
- **Config**: `config.yaml.example` and `config.scared.yml` get new keys.
- **Docs**: README results table and pipeline section, API.md for the config keys and benchmark JSON, PlantUML where the pipeline is drawn, and a regenerated OpenWiki.
- **Tests**: unit tests for the offset, filter and evaluator statistics; Python tests for the new columns.
- **Runtime**: the outlier filter must stay within the budget in design.md. A wider disparity range costs matching time roughly in proportion.
- **No new dependencies.**

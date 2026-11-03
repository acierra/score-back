# Spec Delta

## Purpose

Explains why a SCARED reconstruction is wrong or incomplete, not only how far off it is. The SCARED benchmark reports signed depth bias by depth, fitted calibration-error models, the error tail and the reason each ground-truth pixel went unreconstructed. It also accepts the reconstruction settings as flags, so tuning can be measured without a rebuild.

## ADDED Requirements

### Requirement: Depth-binned signed bias
When ground truth is available, the benchmark JSON SHALL contain a `depth_bias` list. Its bins SHALL be 10 mm wide, cover the configured depth range and be keyed by ground-truth depth. Each entry SHALL give:
- the bin's lower and upper depth in metres;
- the matched-point count;
- the median signed depth error (reconstructed z − ground-truth z) in metres;
- the median absolute 3-D error in metres.

Bins with fewer than 200 matched points SHALL be omitted. The JSON SHALL also contain `bias_z_median_m`, the median signed depth error over all matched points.

#### Scenario: Bias reported per bin
- **WHEN** the benchmark runs on a keyframe with ground truth spanning 55–150 mm
- **THEN** `depth_bias` lists 10 mm bins from 50 mm upward, each with count ≥ 200, a signed median and an absolute median, and bins outside the populated range are absent

#### Scenario: No ground truth
- **WHEN** the keyframe has no `point_cloud.obj`
- **THEN** neither `depth_bias` nor `bias_z_median_m` appear in the JSON

### Requirement: Calibration-error model fits
When ground truth is available, the benchmark SHALL fit two models to the matched points and report both in the JSON. Each model SHALL have its parameter and the RMS of its residual signed depth error over the depth bins, in metres.
- **Disparity-offset model**: `fit_disparity_offset_px` is the constant δ that, added to every disparity, best removes the signed depth error.
- **Depth-scale model**: `fit_depth_scale` is the factor s that best maps ground-truth depth to reconstructed depth.

The fit SHALL be robust to the error tail, so points beyond 20 mm error cannot dominate it.

#### Scenario: Synthetic offset recovered
- **WHEN** the benchmark evaluates a synthetic reconstruction whose disparities are the ground-truth disparities minus 0.1 px
- **THEN** `fit_disparity_offset_px` is +0.1 px within ±0.01 px, and the offset model's residual is smaller than the scale model's

#### Scenario: Synthetic scale recovered
- **WHEN** the benchmark evaluates a synthetic reconstruction whose depths are ground-truth depths × 1.01
- **THEN** `fit_depth_scale` is 1.01 within ±0.001, and the scale model's residual is smaller than the offset model's

### Requirement: Error-tail counts
When ground truth is available, the benchmark JSON SHALL report the fraction of matched points whose 3-D error exceeds 5 mm, 20 mm and 50 mm, as `tail_over_5mm`, `tail_over_20mm` and `tail_over_50mm`. It SHALL also report the largest 3-D error as `max_3d_m`.

#### Scenario: Tail reflects outliers
- **WHEN** 10 of 10 000 matched points are 60 mm off and the rest are within 1 mm
- **THEN** `tail_over_50mm` is 0.001 and `max_3d_m` is about 0.06

### Requirement: Coverage-loss attribution
When ground truth is available, the benchmark JSON SHALL contain `coverage_loss`. It SHALL assign every valid ground-truth pixel without a reconstructed point to exactly one cause, as a fraction of valid ground-truth pixels. The causes, checked in this order, are:
1. `out_of_search_range`: the pixel's ground-truth disparity lies outside the matcher's disparity search range.
2. `image_border`: the pixel lies in the left-image band where the search range extends beyond the right image.
3. `matcher_rejected`: the matcher produced no valid disparity.
4. `depth_range`: the reprojected depth fell outside the configured range.
5. `outlier_filter`: the outlier filter removed the point.

The fractions plus `coverage` SHALL sum to 1 within floating-point tolerance. The JSON SHALL also report `unscored_fraction`: the fraction of reconstructed points that have no ground truth.

#### Scenario: Losses sum up
- **WHEN** the benchmark runs on any ground-truth keyframe
- **THEN** `coverage` plus the five `coverage_loss` fractions equals 1 ± 1e-9

#### Scenario: Near tissue attributed to search range
- **WHEN** a keyframe's near tissue has a ground-truth disparity above the configured number of disparity levels
- **THEN** those pixels are counted in `out_of_search_range`, not in `matcher_rejected`

### Requirement: Tuning flags
The benchmark SHALL accept flags that override every reconstruction setting that affects accuracy:
- the matcher's block size, uniqueness ratio, speckle window, speckle range, left-right tolerance, pre-filter cap and median kernel;
- WLS on/off;
- the disparity offset;
- each outlier-filter setting.

The benchmark SHALL echo every effective setting in its JSON under `settings`. Unknown or malformed flags SHALL fail with a usage message and a non-zero exit status. Without flags, the benchmark SHALL use the same defaults as the CLI configuration.

#### Scenario: Speckle sweep without rebuild
- **WHEN** the benchmark runs twice with `--speckle-window=100` and `--speckle-window=800`
- **THEN** both runs succeed, and each JSON's `settings.speckle_window_size` shows the value used

#### Scenario: Malformed flag
- **WHEN** the benchmark is given `--outlier-window=abc`
- **THEN** it exits non-zero with a usage message before loading the dataset

### Requirement: Offset fitting across keyframes
The overview tool SHALL provide an offset-fitting mode. The mode SHALL:
1. Run the benchmark on the given ground-truth keyframes with the current settings.
2. Combine the per-keyframe `fit_disparity_offset_px` values into one recommended offset, weighted by matched points.
3. Run a leave-one-dataset-out check. For each SCARED dataset, fit the offset on the other datasets, apply it to the held-out dataset, and report that dataset's MAE and median signed bias before and after.

The mode SHALL print the recommended offset and the check table. It SHALL NOT modify any configuration file.

#### Scenario: Fit printed with holdout check
- **WHEN** the user runs the fitting mode on keyframes from dataset_1, dataset_2 and dataset_3
- **THEN** the tool prints one recommended offset in pixels, plus one holdout row per dataset with before/after MAE and signed bias, and no file under the repository changes

#### Scenario: Single dataset
- **WHEN** all given keyframes come from one dataset
- **THEN** the tool prints the recommended offset and states that the holdout check needs at least two datasets

# Spec Delta

## MODIFIED Requirements

### Requirement: Metrics table columns
Every table output of the overview tool SHALL contain one row per keyframe, with these columns in this order:
1. keyframe
2. status (`ok` or `failed`)
3. has ground truth
4. valid points %
5. depth p5, median and p95 (mm)
6. matching time (ms)
7. reconstruction time (ms)
8. coverage %
9. MAE 3D (mm)
10. MAE z (mm)
11. RMSE 3D (mm)
12. median 3D error (mm)
13. within 1 mm %, within 2 mm % and within 5 mm %
14. median signed z bias (mm)
15. fitted disparity offset (px)
16. tail over 5 mm %, tail over 20 mm %
17. max 3D error (mm)
18. lost to search range %, lost to matcher %

Lengths SHALL be in millimetres and fractions in percent. A value that does not apply SHALL be empty, never zero. This covers accuracy, bias, tail and loss columns without ground truth, and every metric of a failed keyframe.

#### Scenario: Keyframe without ground truth
- **WHEN** a keyframe without `point_cloud.obj` is included
- **THEN** its row has status `ok`, "has ground truth" false, filled depth and timing columns, and empty accuracy, bias, tail and loss columns

#### Scenario: Failed keyframe
- **WHEN** the benchmark fails for a keyframe
- **THEN** its row has status `failed` and all metric columns empty, and the other rows are still written

#### Scenario: Bias columns filled
- **WHEN** a ground-truth keyframe is benchmarked
- **THEN** its median signed z bias and fitted disparity offset columns hold numbers taken from the benchmark JSON's `bias_z_median_m` (converted to mm) and `fit_disparity_offset_px`

### Requirement: XLSX output
The overview tool SHALL accept an XLSX output path. The workbook it writes SHALL have two sheets:
- **Metrics sheet (first):** the metrics table with numeric cells, a frozen header row and fixed number formats.
- **Run-parameters sheet (second):**
  - the number of disparities and the depth range;
  - every reconstruction-tuning flag passed to the benchmark, with its value;
  - the date and time;
  - the repository's git commit, when available;
  - the benchmark binary path.

#### Scenario: XLSX written
- **WHEN** the user runs the overview tool with an XLSX path
- **THEN** the workbook opens in spreadsheet software with the metrics sheet and the run-parameters sheet, and metric cells are numbers, not text

#### Scenario: XLSX support missing
- **WHEN** an XLSX path is given but the XLSX library is not installed
- **THEN** the tool exits with a non-zero status and a message naming the missing package and how to install it, before running any benchmark

#### Scenario: Tuning flags recorded
- **WHEN** the user runs the overview tool with `--bench-arg=--speckle-window=800` and an XLSX path
- **THEN** the run-parameters sheet lists `--speckle-window=800`

## ADDED Requirements

### Requirement: Reconstruction flag pass-through
The overview tool SHALL accept repeatable `--bench-arg=FLAG` options and pass each FLAG unchanged to every benchmark invocation, in the order given. The benchmark still validates the flags: if it rejects one, that keyframe SHALL be reported as `failed` with the benchmark's error message.

#### Scenario: Offset applied to all keyframes
- **WHEN** the user runs the overview tool with `--bench-arg=--disparity-offset=0.1` on three keyframes
- **THEN** each keyframe's benchmark JSON shows `settings.disparity_offset_px` equal to 0.1

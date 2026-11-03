# quality-reports Specification

## Purpose

Makes SCARED reconstruction metrics portable: the overview tool writes per-keyframe results as CSV or XLSX tables that can be archived, compared between runs and opened in spreadsheet software.

## Requirements

### Requirement: Metrics table columns
Every table output of the overview tool SHALL contain one row per keyframe with these columns, in this order: keyframe, status (`ok` or `failed`), has ground truth, valid points %, depth p5 / median / p95 (mm), matching time (ms), reconstruction time (ms), coverage %, MAE 3D (mm), MAE z (mm), RMSE 3D (mm), median 3D error (mm), within 1 mm %, within 2 mm %, within 5 mm %. Lengths SHALL be in millimetres and fractions in percent. Values that do not apply, such as accuracy columns without ground truth or any metric of a failed keyframe, SHALL be empty, never zero.

#### Scenario: Keyframe without ground truth
- **WHEN** a keyframe without `point_cloud.obj` is included
- **THEN** its row has status `ok`, "has ground truth" false, filled depth and timing columns, and empty accuracy columns

#### Scenario: Failed keyframe
- **WHEN** the benchmark fails for a keyframe
- **THEN** its row has status `failed` and all metric columns empty, and the other rows are still written

### Requirement: CSV output
The overview tool SHALL accept a CSV output path. It SHALL write the metrics table as RFC 4180 CSV in UTF-8, with one header row, a dot decimal separator and no thousands separators. This output SHALL need nothing beyond the Python standard library.

#### Scenario: CSV written
- **WHEN** the user runs the overview tool with a CSV path on three keyframes
- **THEN** the file has a header row plus three data rows, and spreadsheet software parses the numeric columns as numbers

### Requirement: XLSX output
The overview tool SHALL accept an XLSX output path. It SHALL write a workbook whose first sheet holds the metrics table with numeric cells, a frozen header row and fixed number formats. A second sheet SHALL record the run parameters: number of disparities, depth range, date and time, the repository's git commit (when available) and the benchmark binary path.

#### Scenario: XLSX written
- **WHEN** the user runs the overview tool with an XLSX path
- **THEN** the workbook opens in spreadsheet software with the metrics sheet and the run-parameters sheet, and metric cells are numbers, not text

#### Scenario: XLSX support missing
- **WHEN** an XLSX path is given but the XLSX library is not installed
- **THEN** the tool exits with a non-zero status and a message naming the missing package and how to install it, before running any benchmark

### Requirement: Outputs combine in one run
The overview tool SHALL run the benchmark once per keyframe no matter how many outputs are requested. CSV, XLSX, JSON, depth images and the viewer site can be requested together in one run, and all of them SHALL reflect the same results. The Markdown table SHALL still be printed to standard output.

#### Scenario: All outputs at once
- **WHEN** the user requests CSV, XLSX and the viewer in one invocation
- **THEN** each keyframe is benchmarked once, and the CSV, the XLSX and the viewer's metrics show the same values for every keyframe

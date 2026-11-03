# Spec Delta

## Purpose

Defines the tunable steps between a disparity map and the published point cloud that set reconstruction accuracy and completeness. These are the disparity offset correction, the neighbourhood-consistency outlier filter and the disparity search range, together with how they are configured.

## ADDED Requirements

### Requirement: Disparity offset correction
The reconstruction SHALL add a configurable disparity offset, in pixels and possibly fractional, to every valid disparity before reprojecting it to 3-D.
- A pixel whose corrected disparity is zero or negative SHALL be invalid.
- The offset SHALL be set by the `cloud_point.disparity_offset_px` configuration key and SHALL default to 0.
- With offset 0, the output SHALL be identical to reconstruction without the correction.

#### Scenario: Default leaves output unchanged
- **WHEN** a stereo pair is reconstructed with no `disparity_offset_px` configured
- **THEN** every point equals the point produced before this change

#### Scenario: Positive offset moves points closer
- **WHEN** a constant disparity map of 50 px is reconstructed with offset +0.5 px
- **THEN** every point's depth equals the depth of a 50.5 px disparity under the same reprojection matrix

#### Scenario: Offset drives disparity non-positive
- **WHEN** a pixel's disparity is 0.25 px and the offset is −0.5 px
- **THEN** that pixel is reported as invalid, not as a point behind the camera

### Requirement: Neighbourhood-consistency outlier filter
The reconstruction SHALL offer an outlier filter that runs after the depth-range filter.
- It SHALL invalidate a point when its depth differs from the median depth of the valid points in a square window around it by more than max(absolute threshold, relative threshold × median depth).
- It SHALL also invalidate a point when the window holds fewer valid neighbours than a configured minimum.
- The filter SHALL be controlled by `cloud_point.outlier_filter`, which holds `enabled`, `window` (odd, ≥ 3), `abs_threshold_m`, `rel_threshold` and `min_neighbours`.
- The filter SHALL be disabled by default.
- The filter SHALL only invalidate points. It SHALL never move or add a point.

#### Scenario: Isolated spike removed
- **WHEN** a smooth surface at 80 mm contains a single point at 200 mm, and the filter is enabled with a 5×5 window and a 2 mm absolute threshold
- **THEN** the 200 mm point is invalid and every other point is unchanged

#### Scenario: Genuine slope kept
- **WHEN** a plane tilted 45° is reconstructed with the filter enabled at default thresholds
- **THEN** no point of the plane interior is invalidated

#### Scenario: Filter disabled
- **WHEN** `outlier_filter.enabled` is false or the section is absent
- **THEN** the cloud is identical to the unfiltered cloud

### Requirement: Configurable disparity search range
The number of disparity levels SHALL remain configurable through `cloud_point.num_disparities`. The value SHALL be a positive multiple of 16. Changing it SHALL change the nearest reconstructable depth accordingly. Any default change for the SCARED configuration SHALL be documented, together with its measured runtime cost.

#### Scenario: Wider range reaches nearer tissue
- **WHEN** a keyframe whose near tissue has a disparity above 160 px is reconstructed with 224 disparity levels instead of 160
- **THEN** pixels with disparity between 160 and 224 px can be reconstructed where before they were reported as outside the search range

### Requirement: Configuration validation
Loading the configuration SHALL reject any of the following with an error that names the offending key:
- a non-finite `disparity_offset_px`;
- an `outlier_filter.window` that is even or less than 3;
- a negative threshold;
- a negative `min_neighbours`, or one at least window² (the most neighbours a window can hold is window² − 1).

Omitted keys SHALL take their defaults.

#### Scenario: Even window rejected
- **WHEN** the configuration sets `outlier_filter.window: 4`
- **THEN** loading fails with a message naming `outlier_filter.window`

#### Scenario: Keys omitted
- **WHEN** the configuration has a `cloud_point` section without the new keys
- **THEN** it loads with offset 0 and the outlier filter disabled

### Requirement: Same reconstruction everywhere
The CLI's compute-cloud options, the viewer launch option and the SCARED benchmark SHALL apply the offset correction and the outlier filter in the same order and with the same semantics. The same settings SHALL therefore produce the same cloud from the same stereo pair in every entry point.

#### Scenario: Benchmark matches client
- **WHEN** the benchmark and the CLI reconstruct the same keyframe with the same offset, filter and matcher settings
- **THEN** both produce identical point clouds

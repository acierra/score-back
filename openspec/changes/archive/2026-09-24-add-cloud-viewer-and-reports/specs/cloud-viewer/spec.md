# Spec Delta

## Purpose

Gives developers an offline, in-repo web viewer for reconstructed point clouds. It can be launched on request from the SCARED dataset tooling (with ground-truth error colouring) and from the interactive CLI (for clouds computed over RPC).

## ADDED Requirements

### Requirement: Offline viewer site
The system SHALL produce a self-contained viewer site: a directory with an `index.html` entry page, its scripts and per-frame data files. The site SHALL open directly from the local filesystem (`file://`) in current Chrome/Chromium and Firefox without a web server. It SHALL load no resources from the network.

#### Scenario: Opened from disk without network
- **WHEN** a generated viewer site's `index.html` is opened via `file://` on a machine with no network access
- **THEN** the page renders the first frame's point cloud and all controls work

#### Scenario: Site is relocatable
- **WHEN** a generated viewer site directory is copied or moved to another path or machine
- **THEN** it opens and renders the same way without regeneration

### Requirement: Interactive point-cloud display
The viewer SHALL display the selected frame's reconstructed points in the rectified left-camera frame, in millimetres. It SHALL support orbit, pan and zoom, a reset to the default view and a side view. It SHALL let the user adjust point size. When the site contains more than one frame, it SHALL let the user switch between them.

#### Scenario: Switching frames
- **WHEN** a site with several frames is open and the user selects another frame
- **THEN** the viewer replaces the displayed cloud with that frame's cloud and updates the frame's metrics

### Requirement: Colouring modes
The viewer SHALL colour points by one of: camera image colour, depth, 3D error against ground truth, or signed depth bias (reconstruction z minus ground-truth z). It SHALL show a legend with the value range for the depth, error and bias modes, and the user SHALL be able to set the upper limit of the error/bias colour scale. In the error and bias modes, points without a ground-truth counterpart SHALL be drawn in a neutral colour. The error and bias modes SHALL be disabled for frames without ground truth.

#### Scenario: Frame with ground truth
- **WHEN** a frame that has ground truth is shown in 3D error mode
- **THEN** each point is coloured by its Euclidean distance to the ground-truth point at the same pixel, and points with no ground truth are drawn neutral

#### Scenario: Frame without ground truth
- **WHEN** a frame without ground truth is selected while error or bias mode is active
- **THEN** the viewer switches to a mode that does not need ground truth, and the error and bias controls are disabled

### Requirement: Ground-truth overlay and statistics
For frames with ground truth, the viewer SHALL provide a toggle that overlays the ground-truth points in a distinct colour. It SHALL also show the frame's benchmark metrics (valid fraction, coverage, MAE, RMSE, median error, threshold accuracies, depth percentiles, matching time), a histogram of per-point 3D error and the median depth bias per depth bin.

#### Scenario: Overlay toggled
- **WHEN** the user enables the ground-truth overlay on a frame with ground truth
- **THEN** ground-truth points are drawn together with the reconstruction, in a colour that no colouring mode uses

### Requirement: Viewer frame export
The system SHALL export a frame for the viewer by sampling the dense reconstruction grid every N pixels in each direction, where N is the configurable stride with default 4. Samples SHALL be raw pixel values, not averages. Each sample SHALL carry the reconstructed XYZ, the rectified ground-truth XYZ when available, and the rectified left-image colour. Invalid values SHALL stay distinguishable from valid ones. Coordinates SHALL be accurate to within 0.02 mm of the values they were sampled from.

#### Scenario: Error statistics survive export
- **WHEN** a SCARED keyframe with ground truth is exported at stride 4 and its mean 3D error is recomputed from the exported samples
- **THEN** the result is within 0.05 mm of the full-resolution MAE that the benchmark reports for the same run

#### Scenario: Invalid pixels
- **WHEN** a pixel has no valid reconstruction or no ground truth
- **THEN** the exported frame marks that value as invalid, and the viewer neither draws it as a point nor scores it

### Requirement: Launch from dataset tooling
The SCARED overview tool SHALL accept a view option. It SHALL run the benchmark for the listed keyframes, build one viewer site that contains every keyframe that succeeded and its metrics, and open the site in the default browser. An output-directory option SHALL set where the site is written. A no-open option SHALL build the site without opening a browser. A serve option SHALL serve the site over HTTP, bound to 127.0.0.1 by default, on a configurable port. If that port is in use, the tool SHALL report it and exit without serving.

#### Scenario: View several keyframes
- **WHEN** the user runs the overview tool with the view option on five keyframes
- **THEN** it prints the metrics table as before, writes a site with five frames, prints the site path and opens it in the browser

#### Scenario: One keyframe fails
- **WHEN** the benchmark fails for one of the listed keyframes
- **THEN** the tool reports that keyframe as failed, and the site contains the remaining keyframes

#### Scenario: Headless environment
- **WHEN** no browser can be launched (for example inside the Docker dev container)
- **THEN** the tool still writes the site, prints its path, and exits with success

#### Scenario: Port already in use
- **WHEN** the serve option is given a port that another process holds
- **THEN** the tool prints an error naming the port and exits with a non-zero status

### Requirement: Launch from the CLI
The interactive CLI SHALL offer a menu option that computes a cloud over RPC with the configured stereo settings, writes a single-frame viewer site (no ground truth) to a configurable directory, and tries to open it in the default browser. The CLI SHALL print the site path whether or not a browser opened, and SHALL stay in its menu loop afterwards. In builds without OpenCV support, the option SHALL print the same "not built" message as the other reconstruction options.

#### Scenario: Viewing an RPC cloud
- **WHEN** the user selects the viewer option while connected to a server that serves a stereo pair
- **THEN** the CLI prints the cloud summary as the compute option does, writes the site, prints its path, and shows the menu again

#### Scenario: Browser cannot be opened
- **WHEN** the browser launch command fails or is unavailable
- **THEN** the CLI prints the site path together with a note that it could not open a browser, and does not treat this as an error

#### Scenario: Reconstruction fails
- **WHEN** the cloud computation returns an error
- **THEN** the CLI prints the error, writes no site, and shows the menu again

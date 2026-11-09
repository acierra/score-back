#pragma once

#include "cloud_point_rpc/preprocessing_config.hpp"
#include "export.h"
#include <iostream>
#include <string>
namespace score {

/**
 * @brief Offline viewer settings for CLI option 6 ("compute-cloud + open
 *        viewer"). Mirrors ViewerConfig from config.hpp.
 */
struct CliViewerOptions {
    /// Site directory; each capture adds a timestamped frame to it.
    std::string output_dir{"out/viewer-cli"};
    /// Viewer template directory; empty = compile-time default
    /// (CPRPC_VIEWER_TEMPLATE_DIR, the source-tree template).
    std::string template_dir;
    /// Executable launched as `open_command <index.html>` without a shell;
    /// treated as a single program name/path, not a command line.  Empty =
    /// do not try to open a browser.
    std::string open_command{"xdg-open"};
    /// Viewer frame sampling stride (>= 1).
    int stride{4};
};

/**
 * @brief Stereo reconstruction settings for CLI options 4, 5 and 6.
 *
 * Mirrors CloudPointConfig from config.hpp without pulling OpenCV into the
 * CLI interface.
 */
struct CliStereoOptions {
    /// Neighbourhood-consistency outlier filter applied after the depth-range
    /// filter.  Mirrors CloudPointConfig::OutlierFilter /
    /// PointCloudBuilder::Options::OutlierFilter.
    struct OutlierFilter {
        bool enabled{false};
        int window{5};                 ///< odd, >= 3
        double abs_threshold_m{0.002}; ///< >= 0
        double rel_threshold{0.03};    ///< >= 0, times the median depth
        int min_neighbours{6};         ///< 0 <= min_neighbours < window^2
    };

    bool use_gpu{true};       ///< GPU matcher (falls back to CPU) or CPU
    int num_disparities{128}; ///< SGBM levels, positive multiple of 16
    float min_depth_m{0.01f}; ///< Reject points nearer than this (m)
    float max_depth_m{10.0f}; ///< Reject points farther than this (m)
    int ply_stride{1};        ///< Grid decimation for PLY export
    bool wls_filter{false};   ///< WLS disparity smoothing (visualisation)
    /// Added to every valid disparity (px) before reprojection; 0 = off.
    double disparity_offset_px{0.0};
    OutlierFilter outlier_filter; ///< Disabled by default
    CliViewerOptions viewer;      ///< Option 6 viewer site settings
    PreprocessingConfig preprocessing; ///< None by default
};

/**
 * @brief Runs the CLI client.
 *
 * @param input Input stream (usually std::cin)
 * @param output Output stream (usually std::cout)
 * @param ip Server IP
 * @param port Server Port
 * @param stereo Stereo reconstruction and viewer settings (options 4/5/6)
 * @return int exit code
 */
int CRPC_EXPORT run_cli(std::istream &input, std::ostream &output,
                        const std::string &ip, int port,
                        const CliStereoOptions &stereo = CliStereoOptions{});

} // namespace score

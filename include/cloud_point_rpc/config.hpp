#pragma once

#include "cloud_point_rpc/preprocessing_config.hpp"
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace score {

struct ServerConfig {
    std::string ip;
    int port{0};
};

struct TestData {
    std::vector<double> intrinsic_params;
    std::vector<double> extrinsic_params;
    std::vector<std::vector<double>> cloud_point;
};

/// @brief Stereo reconstruction settings consumed by the CLI (option 4/5).
///
/// Depth limits are physical bounds on the scene: anything reprojected
/// outside [min_depth_m, max_depth_m] is discarded as a mismatch.  For the
/// SCARED endoscopic rig use roughly 0.02–0.3 m; the defaults are wide enough
/// for a generic Unity scene.
struct CloudPointConfig {
    std::string algorithm{"gpu"}; ///< "gpu" (falls back to CPU) or "cpu"
    int num_disparities{128};     ///< SGBM levels, positive multiple of 16
    double min_depth_m{0.01};
    double max_depth_m{10.0};
    int ply_stride{1}; ///< Grid decimation for PLY export (1 = full res)
    /// Edge-aware WLS smoothing of the disparity map (needs opencv
    /// ximgproc).  Smoother meshes, slightly lower accuracy, ~2x matching
    /// time; intended for visualisation.
    bool wls_filter{false};
    /// Added to every valid disparity (px, may be fractional) before
    /// reprojection; corrects a systematic matcher bias.  0 = untouched.
    double disparity_offset_px{0.0};

    /// @brief Neighbourhood-consistency outlier filter, applied after the
    ///        depth-range filter.  Mirrors
    ///        PointCloudBuilder::Options::OutlierFilter.
    struct OutlierFilter {
        bool enabled{false};
        int window{5};                 ///< odd, >= 3
        double abs_threshold_m{0.002}; ///< >= 0
        double rel_threshold{0.03};    ///< >= 0, times the median depth
        int min_neighbours{6};         ///< 0 <= min_neighbours < window^2
    };
    OutlierFilter outlier_filter;
    PreprocessingConfig
        preprocessing; ///< Ordered photometric chain, none by default
};

/// @brief Offline point-cloud viewer settings consumed by CLI option 6
///        ("compute-cloud + open viewer").
struct ViewerConfig {
    /// Site directory; repeated captures accumulate as frames in one site.
    std::string output_dir{"out/viewer-cli"};
    /// Directory holding the viewer template (index.html, viewer.js, ...).
    /// Empty = the source-tree template compiled into the CLI
    /// (CPRPC_VIEWER_TEMPLATE_DIR).
    std::string template_dir;
    /// Executable (name on PATH or full path) launched with the site's
    /// index.html as its only argument; no shell, no extra arguments.
    /// Empty = write the site but do not open a browser.
#ifdef __APPLE__
    std::string open_command{"open"};
#else
    std::string open_command{"xdg-open"};
#endif
    /// Sample every stride-th pixel for the viewer frame (>= 1).
    int stride{4};
};

struct Config {
    ServerConfig server;
    TestData test_data;
    CloudPointConfig cloud_point;
    ViewerConfig viewer;
};

/// @brief Read cloud_point.preprocessing without requiring OpenCV.
/// Unknown keys and implicit independent statistics are rejected.
inline PreprocessingConfig preprocessing_from_yaml(const YAML::Node &node) {
    PreprocessingConfig result;
    if (!node.IsDefined())
        return result;
    if (!node.IsMap())
        throw std::invalid_argument("cloud_point.preprocessing must be a map");
    for (const auto &entry : node) {
        const auto key = entry.first.as<std::string>();
        if (key != "enabled" && key != "chain")
            throw std::invalid_argument("unknown preprocessing key: " + key);
    }
    result.enabled   = node["enabled"].as<bool>(true);
    const auto chain = node["chain"];
    if (chain.IsDefined()) {
        if (!chain.IsSequence())
            throw std::invalid_argument(
                "preprocessing.chain must be a sequence");
        for (const auto &item : chain) {
            if (!item.IsMap() || !item["method"])
                throw std::invalid_argument(
                    "each preprocessing step must be a map with method");
            PreprocessingStep s;
            s.method = item["method"].as<std::string>();
            std::vector<std::string> allowed{"method"};
            if (s.method == "pair_normalize") {
                allowed.insert(allowed.end(), {"statistics", "epsilon"});
            } else if (s.method == "clahe") {
                allowed.insert(allowed.end(),
                               {"statistics", "clip_limit", "tile_grid_size"});
            } else if (s.method == "local_normalize" ||
                       s.method == "illumination_correction") {
                allowed.insert(allowed.end(),
                               {"statistics", "window_size", "epsilon"});
            } else if (s.method == "specular_mask") {
                allowed.insert(allowed.end(),
                               {"brightness_threshold", "dilation_radius"});
            } else if (s.method == "edge_preserving_denoise") {
                allowed.insert(allowed.end(),
                               {"diameter", "sigma_color", "sigma_space"});
            }
            for (const auto &entry : item) {
                const auto key = entry.first.as<std::string>();
                if (std::find(allowed.begin(), allowed.end(), key) ==
                    allowed.end())
                    throw std::invalid_argument("unknown preprocessing." +
                                                s.method + " key: " + key);
            }
            s.statistics     = item["statistics"].as<std::string>(s.statistics);
            s.clip_limit     = item["clip_limit"].as<double>(s.clip_limit);
            s.tile_grid_size = item["tile_grid_size"].as<int>(s.tile_grid_size);
            s.window_size    = item["window_size"].as<int>(s.window_size);
            s.epsilon        = item["epsilon"].as<double>(s.epsilon);
            s.diameter       = item["diameter"].as<int>(s.diameter);
            s.sigma_color    = item["sigma_color"].as<double>(s.sigma_color);
            s.sigma_space    = item["sigma_space"].as<double>(s.sigma_space);
            s.brightness_threshold =
                item["brightness_threshold"].as<int>(s.brightness_threshold);
            s.dilation_radius =
                item["dilation_radius"].as<int>(s.dilation_radius);
            result.chain.push_back(std::move(s));
        }
    }
    validate_preprocessing(result);
    return result;
}

class ConfigLoader {
  public:
    static Config load(const std::string &path) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            Config c;

            // Server
            if (config["server"]) {
                c.server.ip =
                    config["server"]["ip"].as<std::string>("127.0.0.1");
                c.server.port = config["server"]["port"].as<int>(8080);
            } else {
                c.server.ip   = "127.0.0.1";
                c.server.port = 8080;
                LOG(WARNING) << "No 'server' section, using defaults.";
            }

            // Test Data
            if (config["test_data"]) {
                c.test_data.intrinsic_params =
                    config["test_data"]["intrinsic_params"]
                        .as<std::vector<double>>();
                c.test_data.extrinsic_params =
                    config["test_data"]["extrinsic_params"]
                        .as<std::vector<double>>();

                // Parse cloud_point (list of lists)
                auto cp_node = config["test_data"]["cloud_point"];
                for (const auto &point_node : cp_node) {
                    c.test_data.cloud_point.push_back(
                        point_node.as<std::vector<double>>());
                }
            } else {
                LOG(WARNING) << "No 'test_data' section, using empty/defaults.";
            }

            // Cloud point (optional)
            if (config["cloud_point"]) {
                const auto &cp = config["cloud_point"];
                CloudPointConfig d;
                c.cloud_point.preprocessing =
                    preprocessing_from_yaml(cp["preprocessing"]);
                c.cloud_point.algorithm =
                    cp["algorithm"].as<std::string>(d.algorithm);
                c.cloud_point.num_disparities =
                    cp["num_disparities"].as<int>(d.num_disparities);
                c.cloud_point.min_depth_m =
                    cp["min_depth_m"].as<double>(d.min_depth_m);
                c.cloud_point.max_depth_m =
                    cp["max_depth_m"].as<double>(d.max_depth_m);
                c.cloud_point.ply_stride =
                    cp["ply_stride"].as<int>(d.ply_stride);
                c.cloud_point.wls_filter =
                    cp["wls_filter"].as<bool>(d.wls_filter);
                if (c.cloud_point.ply_stride < 1) {
                    throw std::runtime_error(
                        "cloud_point.ply_stride must be >= 1");
                }
                if (c.cloud_point.algorithm != "gpu" &&
                    c.cloud_point.algorithm != "cpu") {
                    throw std::runtime_error(
                        "cloud_point.algorithm must be \"gpu\" or \"cpu\"");
                }
                if (c.cloud_point.min_depth_m <= 0.0 ||
                    c.cloud_point.max_depth_m <= c.cloud_point.min_depth_m) {
                    throw std::runtime_error(
                        "cloud_point depth range must satisfy "
                        "0 < min_depth_m < max_depth_m");
                }

                c.cloud_point.disparity_offset_px =
                    cp["disparity_offset_px"].as<double>(d.disparity_offset_px);
                if (!std::isfinite(c.cloud_point.disparity_offset_px)) {
                    throw std::runtime_error(
                        "cloud_point.disparity_offset_px must be finite");
                }

                // Outlier filter: validated even when disabled so that a
                // typo does not surface only when the filter is switched on.
                auto &of = c.cloud_point.outlier_filter;
                if (const auto &ofn = cp["outlier_filter"]) {
                    of.enabled =
                        ofn["enabled"].as<bool>(d.outlier_filter.enabled);
                    of.window = ofn["window"].as<int>(d.outlier_filter.window);
                    of.abs_threshold_m = ofn["abs_threshold_m"].as<double>(
                        d.outlier_filter.abs_threshold_m);
                    of.rel_threshold = ofn["rel_threshold"].as<double>(
                        d.outlier_filter.rel_threshold);
                    of.min_neighbours = ofn["min_neighbours"].as<int>(
                        d.outlier_filter.min_neighbours);
                }
                if (of.window < 3 || of.window % 2 == 0) {
                    throw std::runtime_error(
                        "cloud_point.outlier_filter.window must be odd and "
                        ">= 3, got " +
                        std::to_string(of.window));
                }
                if (!(of.abs_threshold_m >= 0.0)) {
                    throw std::runtime_error(
                        "cloud_point.outlier_filter.abs_threshold_m must be "
                        ">= 0, got " +
                        std::to_string(of.abs_threshold_m));
                }
                if (!(of.rel_threshold >= 0.0)) {
                    throw std::runtime_error(
                        "cloud_point.outlier_filter.rel_threshold must be "
                        ">= 0, got " +
                        std::to_string(of.rel_threshold));
                }
                const long long max_neighbours =
                    static_cast<long long>(of.window) * of.window;
                if (of.min_neighbours < 0 ||
                    of.min_neighbours >= max_neighbours) {
                    throw std::runtime_error(
                        "cloud_point.outlier_filter.min_neighbours must be in "
                        "[0, window^2) = [0, " +
                        std::to_string(max_neighbours) + "), got " +
                        std::to_string(of.min_neighbours));
                }
            }

            // Viewer (optional)
            if (config["viewer"]) {
                const auto &v = config["viewer"];
                ViewerConfig d;
                c.viewer.output_dir =
                    v["output_dir"].as<std::string>(d.output_dir);
                c.viewer.template_dir =
                    v["template_dir"].as<std::string>(d.template_dir);
                c.viewer.open_command =
                    v["open_command"].as<std::string>(d.open_command);
                c.viewer.stride = v["stride"].as<int>(d.stride);
                if (c.viewer.stride < 1) {
                    throw std::runtime_error("viewer.stride must be >= 1");
                }
                if (c.viewer.output_dir.empty()) {
                    throw std::runtime_error(
                        "viewer.output_dir must not be empty");
                }
            }

            return c;
        } catch (const YAML::Exception &e) {
            LOG(ERROR) << "Failed to load config: " << e.what();
            throw std::runtime_error("Config load failed");
        }
    }
};

} // namespace score

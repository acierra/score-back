#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace score {

/// @brief OpenCV-free settings shared by YAML, CLI and compute code.
struct PreprocessingStep {
    std::string method{"none"};
    /// pair_normalize: left_reference; local methods: independent (explicit).
    std::string statistics;
    double clip_limit{2.0};
    int tile_grid_size{8};
    int window_size{15};
    double epsilon{1.0};
    int diameter{5};
    double sigma_color{20.0};
    double sigma_space{5.0};
    int brightness_threshold{245}; ///< Pixels >= threshold are unreliable
    int dilation_radius{1};        ///< Expand highlights by this many pixels
};

/// @brief Empty chain, or enabled=false, is the exact no-processing baseline.
struct PreprocessingConfig {
    bool enabled{true};
    std::vector<PreprocessingStep> chain;
};

/// @brief Validate even disabled steps, so a typo fails before reconstruction.
/// @throws std::invalid_argument naming the invalid method or parameter.
inline void validate_preprocessing(const PreprocessingConfig &config) {
    for (const auto &s : config.chain) {
        const auto fail = [&](const std::string &message) {
            throw std::invalid_argument("preprocessing." + s.method + ": " +
                                        message);
        };
        const auto positive = [](double x) {
            return std::isfinite(x) && x > 0.0;
        };
        if (s.method == "none") {
            if (!s.statistics.empty())
                fail("statistics is not used");
        } else if (s.method == "pair_normalize") {
            if (s.statistics != "left_reference")
                fail("set statistics: left_reference explicitly");
            if (!positive(s.epsilon))
                fail("epsilon must be finite and > 0");
        } else if (s.method == "clahe" || s.method == "local_normalize" ||
                   s.method == "illumination_correction") {
            if (s.statistics != "independent")
                fail("set statistics: independent explicitly");
            if (s.method == "clahe") {
                if (!positive(s.clip_limit) || s.tile_grid_size < 1 ||
                    s.tile_grid_size > 64)
                    fail("clip_limit must be finite and > 0; "
                         "tile_grid_size must be in [1, 64]");
            } else {
                if (s.window_size < 3 || s.window_size > 255 ||
                    s.window_size % 2 == 0)
                    fail("window_size must be odd and in [3, 255]");
                if (!positive(s.epsilon))
                    fail("epsilon must be finite and > 0");
            }
        } else if (s.method == "specular_mask") {
            if (!s.statistics.empty())
                fail("statistics is not used");
            if (s.brightness_threshold < 1 || s.brightness_threshold > 255 ||
                s.dilation_radius < 0 || s.dilation_radius > 15)
                fail("brightness_threshold must be in [1, 255]; "
                     "dilation_radius must be in [0, 15]");
        } else if (s.method == "edge_preserving_denoise") {
            if (!s.statistics.empty())
                fail("statistics is not used");
            if (s.diameter < 1 || s.diameter > 31 || s.diameter % 2 == 0 ||
                !positive(s.sigma_color) || !positive(s.sigma_space))
                fail("diameter must be odd and in [1, 31]; "
                     "sigma_color and sigma_space must be finite and > 0");
        } else {
            fail("unknown method");
        }
    }
}

} // namespace score

#pragma once

#include "cloud_point/cpu_stereo_matcher.hpp"
#include "cloud_point/stereo_matcher.hpp"
#include <memory>

namespace score {

enum class StereoAlgorithmType { CPU, GPU };

/// @brief Factory for creating CPU or GPU stereo matchers.
class StereoMatcherFactory {
  public:
    /// @brief Create a stereo matcher of the requested type.
    ///        If GPU is requested but unavailable, falls back to CPU.
    /// @param num_disparities  Number of disparity levels for SGBM (default
    /// 128).
    ///                         Must be a positive multiple of 16.
    /// @param cpu_params       SGBM tuning used for the CPU matcher (and the
    ///                         CPU fallback of the GPU matcher).
    /// @throws std::invalid_argument if @p num_disparities is not a positive
    ///         multiple of 16.
    [[nodiscard]] static std::unique_ptr<IStereoMatcher>
    create(StereoAlgorithmType type, int num_disparities = 128,
           CpuStereoMatcher::Params cpu_params = CpuStereoMatcher::Params{});
};

} // namespace score

#pragma once

#include <opencv2/core.hpp>

namespace score {

/// @brief Abstract interface for stereo disparity computation.
class IStereoMatcher {
  public:
    virtual ~IStereoMatcher() = default;

    /// @brief Compute disparity map from a rectified stereo pair.
    /// @param left  Left image (grayscale, CV_8UC1).
    /// @param right Right image (grayscale, CV_8UC1).
    /// @return Disparity map (CV_16S for CPU, type depends on backend for GPU).
    [[nodiscard]] virtual cv::Mat compute(const cv::Mat &left,
                                          const cv::Mat &right) = 0;
};

} // namespace score

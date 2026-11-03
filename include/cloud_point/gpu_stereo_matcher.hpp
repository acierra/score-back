#pragma once

#include "cloud_point/stereo_matcher.hpp"

namespace score {

/// @brief GPU-based stereo matcher using cv::cuda::StereoSGM.
///        Falls back to runtime error if CUDA is unavailable.
///
/// cv::cuda::StereoSGM only supports 64, 128 or 256 disparity levels; the
/// requested count is rounded up to the next supported value.
class GpuStereoMatcher : public IStereoMatcher {
  public:
    /// @param min_disparity    Minimum disparity (px).
    /// @param num_disparities  Requested disparity levels (rounded up to
    ///                         64/128/256).
    /// @param uniqueness_ratio Best/second-best cost margin (%).
    GpuStereoMatcher(int min_disparity = 0, int num_disparities = 128,
                     int uniqueness_ratio = 10);
    ~GpuStereoMatcher() override = default;

    [[nodiscard]] cv::Mat compute(const cv::Mat &left,
                                  const cv::Mat &right) override;

    /// @brief Round a disparity count up to the nearest value supported by
    ///        cv::cuda::StereoSGM (64, 128 or 256).
    /// @throws std::invalid_argument if num_disparities exceeds 256.
    [[nodiscard]] static int supported_num_disparities(int num_disparities);

  private:
#ifdef HAVE_OPENCV_CUDA
    cv::Ptr<cv::cuda::StereoSGM> sgm_;
#endif
};

} // namespace score

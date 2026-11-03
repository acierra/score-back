#pragma once

#include "cloud_point/stereo_matcher.hpp"
#include <opencv2/calib3d.hpp>

namespace score {

/// @brief CPU-based stereo matcher using cv::StereoSGBM.
///
/// The matcher is configured with the smoothness penalties and post-filters
/// recommended by OpenCV (P1 = 8·bs², P2 = 32·bs², uniqueness ratio,
/// left-right consistency check, speckle filter).  Without them SGBM
/// degenerates to unregularised winner-take-all block matching, which
/// produces a heavy tail of low-disparity outliers that reproject metres
/// away from the true surface.
class CpuStereoMatcher : public IStereoMatcher {
  public:
    /// @brief True when this build can apply the WLS disparity filter.
    [[nodiscard]] static bool wls_available() noexcept;

    /// @brief SGBM tuning parameters.
    struct Params {
        int block_size;          ///< Odd matching block size (SADWindowSize).
        int uniqueness_ratio;    ///< Best/second-best cost margin (%).
        int speckle_window_size; ///< Max blob size flagged as speckle (0 off).
        int speckle_range;       ///< Max disparity variation inside a blob.
        int disp12_max_diff;     ///< Max left-right disparity mismatch (px).
        int pre_filter_cap;      ///< x-derivative clipping value.
        /// Median filter applied to the disparity map (0 disables, else odd
        /// 3 or 5).  Only trims isolated spikes; SGBM's sub-pixel noise is
        /// correlated over several pixels, so prefer the WLS filter.
        int median_kernel;
        /// Edge-aware weighted-least-squares smoothing of the disparity
        /// (cv::ximgproc::DisparityWLSFilter).  Off by default: on SCARED it
        /// halves the fine-scale surface roughness (mesh normals 45 -> 23
        /// deg off-axis) but costs accuracy (MAE 0.84 -> 0.92 mm, within
        /// 2 mm 80 -> 78 %) and a second SGBM pass (~2x matching time).
        /// Enable for visualisation, keep off for measurement.  Ignored,
        /// with a warning, when OpenCV was built without ximgproc.
        bool wls_filter;
        double wls_lambda; ///< Smoothness weight (2000; 8000 over-smooths).
        double wls_sigma;  ///< Edge sensitivity (typical 0.8-2.0).
        // Explicit constructor avoids a GCC limitation with nested-struct
        // default-member-initialisers used as default function arguments.
        Params() noexcept
            : block_size(5), uniqueness_ratio(10), speckle_window_size(100),
              speckle_range(2), disp12_max_diff(1), pre_filter_cap(31),
              median_kernel(0), wls_filter(false), wls_lambda(2000.0),
              wls_sigma(1.5) {}
    };

    CpuStereoMatcher(int min_disparity = 0, int num_disparities = 128,
                     Params params = Params{});
    ~CpuStereoMatcher() override = default;

    [[nodiscard]] cv::Mat compute(const cv::Mat &left,
                                  const cv::Mat &right) override;

  private:
    cv::Ptr<cv::StereoSGBM> sgbm_;
    cv::Ptr<cv::StereoMatcher> right_matcher_; ///< Only with WLS.
    cv::Ptr<cv::Algorithm> wls_;               ///< DisparityWLSFilter.
    int median_kernel_;
    double wls_lambda_;
    double wls_sigma_;
};

} // namespace score

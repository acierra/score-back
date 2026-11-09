#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

#include "cloud_point/point_cloud_builder.hpp"

namespace score {

/// @brief Depth <-> disparity relation encoded in the reprojection matrix Q.
///
/// cv::stereoRectify produces
///
///     Q = | 1  0  0      -cx        |
///         | 0  1  0      -cy        |
///         | 0  0  0       f         |
///         | 0  0  -1/Tx  (cx-cx')/Tx|
///
/// and cv::reprojectImageTo3D computes (X, Y, Z, W)ᵀ = Q · (x, y, d, 1)ᵀ, so
///
///     z = Q(2,3) / (Q(3,2) · d + Q(3,3)).
///
/// With CALIB_ZERO_DISPARITY (cx == cx') the constant term Q(3,3) is 0 and,
/// since Tx = −B for a right camera on the +x side, Q(3,2) = 1/B.  The
/// familiar z = f·B / d follows, with f = Q(2,3) and f·B = Q(2,3) / |Q(3,2)|.
/// disparity() inverts the exact relation, keeping Q(3,3) in case a rig is
/// rectified without CALIB_ZERO_DISPARITY.
struct ReprojectionGeometry {
    double f{0.0};   ///< Q(2,3): rectified focal length in pixels.
    double q32{0.0}; ///< Q(3,2): −1/Tx, i.e. 1/B for the usual layout.
    double q33{0.0}; ///< Q(3,3): (cx − cx')/Tx, 0 with CALIB_ZERO_DISPARITY.
    double fB{0.0};  ///< f·B = f / |Q(3,2)| in px·m.

    /// @throws std::invalid_argument when q is not 4x4 CV_64F or Q(3,2) == 0.
    static ReprojectionGeometry from_q(const cv::Mat &q);

    /// @brief Depth (m) reprojected from a disparity (px).
    [[nodiscard]] double depth(double disparity_px) const noexcept {
        return f / (q32 * disparity_px + q33);
    }
    /// @brief Disparity (px) that reprojects to the given depth (m).
    [[nodiscard]] double disparity(double depth_m) const noexcept {
        return (f / depth_m - q33) / q32;
    }
};

/// @brief Inputs of diagnose_reconstruction() besides the two clouds.
struct ReconstructionDiagnosticsInputs {
    cv::Mat q;      ///< 4x4 CV_64F reprojection matrix used by the builder.
    cv::Mat reason; ///< CV_8U PointCloudBuilder::Reason map from build().
    int min_disparity{0};    ///< Matcher search range start (px).
    int num_disparities{0};  ///< Matcher search range length (px).
    double disparity_offset_px{0.0}; ///< Offset δ the builder applied.
    float min_depth_m{0.0f}; ///< Builder depth range (bins cover it).
    float max_depth_m{0.0f};
};

/// @brief Why a reconstruction is wrong or incomplete (spec
///        reconstruction-diagnostics, design D3/D4).
struct ReconstructionDiagnostics {
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    /// @brief 10 mm depth bin keyed by ground-truth z (≥ 200 matched points).
    struct DepthBin {
        double lo_m{0.0};
        double hi_m{0.0};
        std::size_t count{0};
        double bias_z_median_m{kNaN}; ///< median (z_rec − z_gt)
        double abs_3d_median_m{kNaN}; ///< median ‖p_rec − p_gt‖
    };

    /// @brief Fractions of valid ground-truth pixels, by loss cause (D4).
    ///
    /// Each unmatched valid ground-truth pixel is assigned the first cause
    /// that applies, in the order listed:
    ///   - out_of_search_range: its ground-truth disparity minus the applied
    ///     offset lies outside [min_disparity, min_disparity + num_disparities);
    ///   - image_border: its column is below min_disparity + num_disparities,
    ///     the left-image band cv::StereoSGBM always leaves invalid because
    ///     the search range there extends beyond the right image;
    ///   - matcher_rejected / depth_range / outlier_filter: the reason the
    ///     builder's trace recorded.
    struct CoverageLoss {
        double out_of_search_range{0.0};
        double image_border{0.0};
        double matcher_rejected{0.0};
        double depth_range{0.0};
        double outlier_filter{0.0};
        double preprocessing_mask{0.0}; ///< Excluded by stereo validity masks
    };

    std::size_t ground_truth_points{0}; ///< valid (finite) GT pixels
    std::size_t matched_points{0};      ///< GT pixels with a finite point
    double coverage{0.0}; ///< matched / ground_truth (as the evaluator)

    std::vector<DepthBin> depth_bias;
    double bias_z_median_m{kNaN}; ///< median (z_rec − z_gt), all matched

    /// Offset model: constant δ added to every measured disparity.
    double fit_disparity_offset_px{kNaN};
    double fit_disparity_offset_residual_m{kNaN}; ///< RMS of bin medians
    /// Scale model: z_rec ≈ s · z_gt.
    double fit_depth_scale{kNaN};
    double fit_depth_scale_residual_m{kNaN}; ///< RMS of bin medians

    double tail_over_5mm{0.0};  ///< fraction of matched with error > 5 mm
    double tail_over_20mm{0.0};
    double tail_over_50mm{0.0};
    double max_3d_m{kNaN}; ///< largest 3-D error over matched points

    CoverageLoss coverage_loss;
    /// Fraction of valid reconstructed points with a non-finite GT pixel.
    double unscored_fraction{0.0};
};

/// @brief Compute the diagnostics of @p predicted against @p ground_truth.
///
/// Pixel correspondence follows PointCloudEvaluator: a ground-truth pixel is
/// valid when all three components are finite, and matched when the
/// predicted point at the same pixel is finite too, so `coverage` equals the
/// evaluator's.  Every valid GT pixel without a match is assigned exactly one
/// `coverage_loss` cause (see CoverageLoss), so coverage plus the six
/// fractions is 1.  The `image_border` band is the leftmost
/// `min_disparity + num_disparities` columns, as cv::StereoSGBM leaves them,
/// so widening the search range moves loss from `matcher_rejected` into
/// `image_border` rather than hiding it.
///
/// Fits (design D3) use matched points with 3-D error below 20 mm:
///   - fit_disparity_offset_px = median(d_gt − d_rec), d = disparity(z);
///   - fit_depth_scale         = median(z_rec / z_gt).
/// Each residual is the RMS over the reported bins of the bin median of the
/// signed depth error left after applying the model (offset: reproject
/// d_rec + δ̂; scale: z_rec / ŝ).
///
/// @param predicted    Dense cloud in the rectified-left frame, metres.
/// @param ground_truth CV_32FC3 map, same frame and dimensions, metres.
/// @param inputs       Q, reason map, search range, offset and depth range.
/// @throws std::invalid_argument on a type/size mismatch, a reason map that
///         disagrees with the cloud, a degenerate Q, or an empty GT.
[[nodiscard]] ReconstructionDiagnostics
diagnose_reconstruction(const PointCloud &predicted,
                        const cv::Mat &ground_truth,
                        const ReconstructionDiagnosticsInputs &inputs);

} // namespace score

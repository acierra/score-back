#pragma once

#include <cstddef>
#include <limits>

#include <opencv2/core.hpp>

#include "cloud_point/point_cloud_builder.hpp"

namespace score {

/// @brief Accuracy and completeness measurements for an XYZ reconstruction.
struct PointCloudMetrics {
    std::size_t ground_truth_points{0};
    std::size_t matched_points{0};

    /// Fraction of valid ground-truth pixels with a finite prediction.
    double coverage{0.0};

    /// Absolute component errors over matched points, in metres.
    double mae_x_m{std::numeric_limits<double>::quiet_NaN()};
    double mae_y_m{std::numeric_limits<double>::quiet_NaN()};
    double mae_z_m{std::numeric_limits<double>::quiet_NaN()};

    /// Euclidean XYZ errors over matched points, in metres. These are NaN when
    /// no ground-truth pixel has a valid prediction.
    double mae_3d_m{std::numeric_limits<double>::quiet_NaN()};
    double rmse_3d_m{std::numeric_limits<double>::quiet_NaN()};
    double median_3d_m{std::numeric_limits<double>::quiet_NaN()};

    /// Fractions of all valid ground-truth pixels reconstructed within the
    /// threshold. Missing predictions therefore count as failures.
    double within_1mm{0.0};
    double within_2mm{0.0};
    double within_5mm{0.0};
};

/// @brief Pixel-aligned evaluation of a reconstructed point cloud.
class PointCloudEvaluator {
  public:
    /// @param predicted Dense point cloud in the rectified-left frame, metres.
    /// @param ground_truth CV_32FC3 point map in the same frame and dimensions,
    ///        metres. Any point with a non-finite component is unknown.
    /// @throws std::invalid_argument on a type, size, or storage mismatch, or
    ///         when the ground-truth map has no valid points.
    [[nodiscard]] static PointCloudMetrics
    evaluate(const PointCloud &predicted, const cv::Mat &ground_truth);
};

} // namespace score

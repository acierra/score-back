#include "cloud_point/point_cloud_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace score {

namespace {

bool finite(const cv::Vec3f &point) {
    return std::isfinite(point[0]) && std::isfinite(point[1]) &&
           std::isfinite(point[2]);
}

} // namespace

PointCloudMetrics PointCloudEvaluator::evaluate(const PointCloud &predicted,
                                                const cv::Mat &ground_truth) {
    if (ground_truth.type() != CV_32FC3) {
        throw std::invalid_argument(
            "PointCloudEvaluator: ground truth must be CV_32FC3");
    }
    if (predicted.width != ground_truth.cols ||
        predicted.height != ground_truth.rows) {
        throw std::invalid_argument(
            "PointCloudEvaluator: prediction and ground-truth dimensions do "
            "not match");
    }

    const auto pixel_count = static_cast<std::size_t>(predicted.width) *
                             static_cast<std::size_t>(predicted.height);
    if (predicted.data.size() != pixel_count * 3u) {
        throw std::invalid_argument(
            "PointCloudEvaluator: prediction storage size is invalid");
    }

    PointCloudMetrics metrics;
    std::vector<double> errors;
    errors.reserve(pixel_count);
    double sum_abs_x         = 0.0;
    double sum_abs_y         = 0.0;
    double sum_abs_z         = 0.0;
    double sum_error         = 0.0;
    double sum_error_squared = 0.0;
    std::size_t within_1mm   = 0;
    std::size_t within_2mm   = 0;
    std::size_t within_5mm   = 0;

    for (int row = 0; row < ground_truth.rows; ++row) {
        for (int column = 0; column < ground_truth.cols; ++column) {
            const cv::Vec3f truth = ground_truth.at<cv::Vec3f>(row, column);
            if (!finite(truth)) {
                continue;
            }
            ++metrics.ground_truth_points;

            const auto index =
                (static_cast<std::size_t>(row) * predicted.width + column) * 3u;
            const cv::Vec3f estimate(predicted.data[index],
                                     predicted.data[index + 1],
                                     predicted.data[index + 2]);
            if (!finite(estimate)) {
                continue;
            }

            ++metrics.matched_points;
            const cv::Vec3f delta = estimate - truth;
            const double abs_x    = std::abs(static_cast<double>(delta[0]));
            const double abs_y    = std::abs(static_cast<double>(delta[1]));
            const double abs_z    = std::abs(static_cast<double>(delta[2]));
            const double error =
                std::sqrt(abs_x * abs_x + abs_y * abs_y + abs_z * abs_z);

            sum_abs_x += abs_x;
            sum_abs_y += abs_y;
            sum_abs_z += abs_z;
            sum_error += error;
            sum_error_squared += error * error;
            errors.push_back(error);
            within_1mm += error <= 0.001;
            within_2mm += error <= 0.002;
            within_5mm += error <= 0.005;
        }
    }

    if (metrics.ground_truth_points == 0) {
        throw std::invalid_argument(
            "PointCloudEvaluator: ground truth has no valid points");
    }

    const double ground_truth_count =
        static_cast<double>(metrics.ground_truth_points);
    metrics.coverage =
        static_cast<double>(metrics.matched_points) / ground_truth_count;
    metrics.within_1mm = static_cast<double>(within_1mm) / ground_truth_count;
    metrics.within_2mm = static_cast<double>(within_2mm) / ground_truth_count;
    metrics.within_5mm = static_cast<double>(within_5mm) / ground_truth_count;

    if (metrics.matched_points == 0) {
        return metrics;
    }

    const double matched_count = static_cast<double>(metrics.matched_points);
    metrics.mae_x_m            = sum_abs_x / matched_count;
    metrics.mae_y_m            = sum_abs_y / matched_count;
    metrics.mae_z_m            = sum_abs_z / matched_count;
    metrics.mae_3d_m           = sum_error / matched_count;
    metrics.rmse_3d_m          = std::sqrt(sum_error_squared / matched_count);

    const auto middle =
        errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2);
    std::nth_element(errors.begin(), middle, errors.end());
    metrics.median_3d_m = *middle;
    if (errors.size() % 2 == 0) {
        const auto lower    = std::max_element(errors.begin(), middle);
        metrics.median_3d_m = (*lower + *middle) / 2.0;
    }

    return metrics;
}

} // namespace score

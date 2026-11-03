#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <opencv2/core.hpp>

#include "cloud_point/point_cloud_evaluator.hpp"

namespace {

score::PointCloud make_cloud(const cv::Mat &points) {
    score::PointCloud cloud;
    cloud.width  = points.cols;
    cloud.height = points.rows;
    cloud.data.reserve(static_cast<std::size_t>(points.total()) * 3u);
    for (int row = 0; row < points.rows; ++row) {
        for (int column = 0; column < points.cols; ++column) {
            const auto point = points.at<cv::Vec3f>(row, column);
            cloud.data.push_back(point[0]);
            cloud.data.push_back(point[1]);
            cloud.data.push_back(point[2]);
        }
    }
    return cloud;
}

} // namespace

TEST(PointCloudEvaluatorTest, ExactPredictionHasPerfectMetrics) {
    const cv::Mat truth(1, 2, CV_32FC3, cv::Scalar(0.1f, 0.2f, 0.3f));
    const auto metrics =
        score::PointCloudEvaluator::evaluate(make_cloud(truth), truth);

    EXPECT_EQ(metrics.ground_truth_points, 2u);
    EXPECT_EQ(metrics.matched_points, 2u);
    EXPECT_DOUBLE_EQ(metrics.coverage, 1.0);
    EXPECT_DOUBLE_EQ(metrics.rmse_3d_m, 0.0);
    EXPECT_DOUBLE_EQ(metrics.within_1mm, 1.0);
    EXPECT_DOUBLE_EQ(metrics.within_2mm, 1.0);
    EXPECT_DOUBLE_EQ(metrics.within_5mm, 1.0);
}

TEST(PointCloudEvaluatorTest, MeasuresFullXyzEuclideanError) {
    cv::Mat truth(1, 1, CV_32FC3, cv::Scalar(0.0f, 0.0f, 1.0f));
    cv::Mat prediction(1, 1, CV_32FC3, cv::Scalar(0.001f, 0.002f, 1.002f));

    const auto metrics =
        score::PointCloudEvaluator::evaluate(make_cloud(prediction), truth);
    const double expected = 0.003;

    EXPECT_NEAR(metrics.mae_x_m, 0.001, 1e-7);
    EXPECT_NEAR(metrics.mae_y_m, 0.002, 1e-7);
    EXPECT_NEAR(metrics.mae_z_m, 0.002, 1e-7);
    EXPECT_NEAR(metrics.mae_3d_m, expected, 1e-7);
    EXPECT_NEAR(metrics.rmse_3d_m, expected, 1e-7);
    EXPECT_DOUBLE_EQ(metrics.within_2mm, 0.0);
    EXPECT_DOUBLE_EQ(metrics.within_5mm, 1.0);
}

TEST(PointCloudEvaluatorTest, MissingPredictionReducesCoverageAndAccuracy) {
    cv::Mat truth(1, 2, CV_32FC3, cv::Scalar(0.0f, 0.0f, 1.0f));
    cv::Mat prediction             = truth.clone();
    const float nan                = std::numeric_limits<float>::quiet_NaN();
    prediction.at<cv::Vec3f>(0, 1) = cv::Vec3f(nan, nan, nan);

    const auto metrics =
        score::PointCloudEvaluator::evaluate(make_cloud(prediction), truth);

    EXPECT_EQ(metrics.ground_truth_points, 2u);
    EXPECT_EQ(metrics.matched_points, 1u);
    EXPECT_DOUBLE_EQ(metrics.coverage, 0.5);
    EXPECT_DOUBLE_EQ(metrics.within_1mm, 0.5);
}

TEST(PointCloudEvaluatorTest, NoPredictionsReportsNaNErrors) {
    cv::Mat truth(1, 1, CV_32FC3, cv::Scalar(0.0f, 0.0f, 1.0f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    cv::Mat prediction(1, 1, CV_32FC3, cv::Scalar(nan, nan, nan));

    const auto metrics =
        score::PointCloudEvaluator::evaluate(make_cloud(prediction), truth);

    EXPECT_DOUBLE_EQ(metrics.coverage, 0.0);
    EXPECT_TRUE(std::isnan(metrics.mae_3d_m));
    EXPECT_TRUE(std::isnan(metrics.rmse_3d_m));
}

TEST(PointCloudEvaluatorTest, IgnoresUnknownGroundTruthPixels) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    cv::Mat truth(1, 2, CV_32FC3);
    truth.at<cv::Vec3f>(0, 0) = cv::Vec3f(0.0f, 0.0f, 1.0f);
    truth.at<cv::Vec3f>(0, 1) = cv::Vec3f(nan, nan, nan);
    cv::Mat prediction(1, 2, CV_32FC3, cv::Scalar(0.0f, 0.0f, 1.0f));

    const auto metrics =
        score::PointCloudEvaluator::evaluate(make_cloud(prediction), truth);
    EXPECT_EQ(metrics.ground_truth_points, 1u);
    EXPECT_EQ(metrics.matched_points, 1u);
    EXPECT_DOUBLE_EQ(metrics.coverage, 1.0);
}

// Tests for StereoRectifier: construction, Q-matrix semantics, from_rpc
// round-trip, near-identity rectification, and invalid-calibration rejection.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "cloud_point/stereo_rectifier.hpp"
#include "cloud_point_rpc/rpc_dto.hpp"

namespace {

// ---------------------------------------------------------------------------
// Helpers — synthetic calibration
// ---------------------------------------------------------------------------

// fx = fy = 800, cx = 320, cy = 240, 640x480, zero distortion,
// R = identity, T = [-0.06, 0, 0] (6 cm horizontal baseline).
constexpr double kFx       = 800.0;
constexpr double kFy       = 800.0;
constexpr double kCx       = 320.0;
constexpr double kCy       = 240.0;
constexpr int kWidth       = 640;
constexpr int kHeight      = 480;
constexpr double kBaseline = 0.06;       // metres
constexpr double kTx       = -kBaseline; // OpenCV T: left→right camera

score::StereoRectifier::Calibration make_calib() {
    score::StereoRectifier::Calibration calib;

    // clang-format off
    calib.k_left = (cv::Mat_<double>(3, 3) <<
        kFx, 0,   kCx,
        0,   kFy, kCy,
        0,   0,   1);
    calib.k_right = calib.k_left.clone();

    calib.d_left  = cv::Mat::zeros(1, 5, CV_64F);
    calib.d_right = cv::Mat::zeros(1, 5, CV_64F);

    calib.r = cv::Mat::eye(3, 3, CV_64F);

    calib.t = (cv::Mat_<double>(3, 1) << kTx, 0.0, 0.0);
    // clang-format on

    calib.image_size = cv::Size(kWidth, kHeight);
    return calib;
}

score::StereoCalibrationRPC make_rpc() {
    score::StereoCalibrationRPC rpc;
    // Camera matrix: row-major 3x3
    rpc.left.camera_matrix  = {kFx, 0, kCx, 0, kFy, kCy, 0, 0, 1};
    rpc.right.camera_matrix = rpc.left.camera_matrix;
    rpc.left.dist_coeffs    = {0, 0, 0, 0, 0};
    rpc.right.dist_coeffs   = {0, 0, 0, 0, 0};
    rpc.rotation            = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    rpc.translation         = {kTx, 0.0, 0.0};
    rpc.width               = kWidth;
    rpc.height              = kHeight;
    return rpc;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction & Q-matrix semantics
// ---------------------------------------------------------------------------

TEST(StereoRectifierTest, QMatrixIs4x4) {
    score::StereoRectifier rectifier(make_calib());
    const cv::Mat &q = rectifier.q();
    EXPECT_EQ(q.rows, 4);
    EXPECT_EQ(q.cols, 4);
    EXPECT_EQ(q.type(), CV_64F);
}

TEST(StereoRectifierTest, QMatrixFocalEntry) {
    // OpenCV Q layout (horizontal stereo):
    //   Q = [1  0  0   -cx  ]
    //       [0  1  0   -cy  ]
    //       [0  0  0    f   ]   ← Q(2,3) = focal length
    //       [0  0 -1/Tx  ...]
    // With fx = fy = 800, Q(2,3) should be ≈ 800.
    score::StereoRectifier rectifier(make_calib());
    const cv::Mat &q = rectifier.q();
    EXPECT_NEAR(q.at<double>(2, 3), kFx, 1.0)
        << "Q(2,3) should equal the focal length (~" << kFx << ")";
}

TEST(StereoRectifierTest, QMatrixBaselineEntry) {
    // Q(3,2) = -1/Tx.  With Tx = -0.06, -1/Tx = +16.667 (positive).
    // Observed sign convention: Q(3,2) > 0 for Tx < 0.
    score::StereoRectifier rectifier(make_calib());
    const cv::Mat &q      = rectifier.q();

    const double expected = -1.0 / kTx; // = +16.667 for Tx=-0.06
    EXPECT_NEAR(q.at<double>(3, 2), expected, 1.0)
        << "Q(3,2) should be -1/Tx = " << expected
        << "  (observed sign: positive for Tx < 0)";
}

TEST(StereoRectifierTest, DepthFromQConsistency) {
    // z = Q(2,3) / (-Q(3,2)) / disparity  →  = fx * |Tx| / disparity.
    // For d=32: z = 800 * 0.06 / 32 = 1.5 m.
    score::StereoRectifier rectifier(make_calib());
    const cv::Mat &q = rectifier.q();

    const double f   = q.at<double>(2, 3);
    const double q32 = q.at<double>(3, 2);
    // baseline_in_Q = -1/q32 gives Tx, |Tx| = baseline
    const double baseline_q = std::abs(-1.0 / q32);
    const double disparity  = 32.0;
    const double z          = f * baseline_q / disparity;

    EXPECT_NEAR(z, 1.5, 1e-3)
        << "z = fx * baseline / disparity = 800 * 0.06 / 32 should be 1.5 m";
}

// ---------------------------------------------------------------------------
// from_rpc round-trip
// ---------------------------------------------------------------------------

TEST(StereoRectifierTest, FromRpcRoundTrip) {
    const auto rpc   = make_rpc();
    const auto calib = score::StereoRectifier::Calibration::from_rpc(rpc);

    // Sizes
    EXPECT_EQ(calib.k_left.rows, 3);
    EXPECT_EQ(calib.k_left.cols, 3);
    EXPECT_EQ(calib.d_left.rows, 1);
    EXPECT_EQ(calib.d_left.cols, 5);
    EXPECT_EQ(calib.r.rows, 3);
    EXPECT_EQ(calib.r.cols, 3);
    EXPECT_EQ(calib.t.rows, 3);
    EXPECT_EQ(calib.t.cols, 1);
    EXPECT_EQ(calib.image_size, cv::Size(kWidth, kHeight));

    // Values — camera matrix diagonal
    EXPECT_DOUBLE_EQ(calib.k_left.at<double>(0, 0), kFx);
    EXPECT_DOUBLE_EQ(calib.k_left.at<double>(1, 1), kFy);
    EXPECT_DOUBLE_EQ(calib.k_left.at<double>(0, 2), kCx);
    EXPECT_DOUBLE_EQ(calib.k_left.at<double>(1, 2), kCy);

    // Translation
    EXPECT_DOUBLE_EQ(calib.t.at<double>(0, 0), kTx);
    EXPECT_DOUBLE_EQ(calib.t.at<double>(1, 0), 0.0);
    EXPECT_DOUBLE_EQ(calib.t.at<double>(2, 0), 0.0);

    // Rotation is identity
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_DOUBLE_EQ(calib.r.at<double>(r, c), r == c ? 1.0 : 0.0);
}

TEST(StereoRectifierTest, FromRpcProducesValidRectifier) {
    const auto rpc   = make_rpc();
    const auto calib = score::StereoRectifier::Calibration::from_rpc(rpc);
    EXPECT_NO_THROW(score::StereoRectifier rectifier(calib));
}

// ---------------------------------------------------------------------------
// Near-identity rectification
// ---------------------------------------------------------------------------

TEST(StereoRectifierTest, RectifyOutputSizeUnchanged) {
    score::StereoRectifier rectifier(make_calib());

    // Deterministic gradient image (use fixed seed via theRNG)
    cv::theRNG().state = 42;
    cv::Mat left(kHeight, kWidth, CV_8UC1);
    cv::randn(left, 128, 40);
    cv::Mat right = left.clone();

    auto [rl, rr] = rectifier.rectify(left, right);

    EXPECT_EQ(rl.rows, kHeight);
    EXPECT_EQ(rl.cols, kWidth);
    EXPECT_EQ(rr.rows, kHeight);
    EXPECT_EQ(rr.cols, kWidth);
}

TEST(StereoRectifierTest, RectifyNearIdentityWithZeroDistortion) {
    // With R=I, identical K, and zero distortion the rectification maps are
    // near-identity.  The mean absolute difference between input and output
    // should be small (< 5 intensity levels out of 255).
    score::StereoRectifier rectifier(make_calib());

    // Deterministic gradient: pixel value = (row + col) % 256
    cv::Mat left(kHeight, kWidth, CV_8UC1);
    for (int r = 0; r < kHeight; ++r)
        for (int c = 0; c < kWidth; ++c)
            left.at<uchar>(r, c) = static_cast<uchar>((r + c) % 256);
    cv::Mat right = left.clone();

    auto [rl, rr] = rectifier.rectify(left, right);

    // Compare rectified-left row sums to original row sums
    cv::Mat diff;
    cv::absdiff(rl, left, diff);
    const double mean_diff = cv::mean(diff)[0];
    EXPECT_LT(mean_diff, 5.0)
        << "Mean absolute pixel difference after rectification is " << mean_diff
        << " — expected near-identity for zero-distortion identical cameras";
}

// ---------------------------------------------------------------------------
// Ground-truth point-map rectification
// ---------------------------------------------------------------------------

TEST(StereoRectifierTest, RectifyLeftPointMapPreservesIdentityGeometry) {
    score::StereoRectifier rectifier(make_calib());
    cv::Mat points(kHeight, kWidth, CV_32FC3, cv::Scalar(0.1f, -0.2f, 1.5f));

    const cv::Mat rectified = rectifier.rectify_left_point_map(points);

    EXPECT_EQ(rectified.type(), CV_32FC3);
    EXPECT_EQ(rectified.size(), points.size());
    const cv::Vec3f center = rectified.at<cv::Vec3f>(kHeight / 2, kWidth / 2);
    EXPECT_NEAR(center[0], 0.1f, 1e-6f);
    EXPECT_NEAR(center[1], -0.2f, 1e-6f);
    EXPECT_NEAR(center[2], 1.5f, 1e-6f);
}

TEST(StereoRectifierTest, RectifyLeftPointMapRejectsWrongType) {
    score::StereoRectifier rectifier(make_calib());
    cv::Mat points(kHeight, kWidth, CV_32FC1, cv::Scalar(1.0f));
    EXPECT_THROW(rectifier.rectify_left_point_map(points),
                 std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Invalid calibration → throws
// ---------------------------------------------------------------------------

TEST(StereoRectifierTest, InvalidKSizeThrows) {
    auto calib   = make_calib();
    calib.k_left = cv::Mat::eye(2, 3, CV_64F); // wrong: 2x3 instead of 3x3
    EXPECT_THROW(score::StereoRectifier{calib}, std::invalid_argument);
}

TEST(StereoRectifierTest, InvalidKTypeThrows) {
    auto calib   = make_calib();
    calib.k_left = cv::Mat::eye(3, 3, CV_32F); // wrong type
    EXPECT_THROW(score::StereoRectifier{calib}, std::invalid_argument);
}

TEST(StereoRectifierTest, InvalidDistSizeThrows) {
    auto calib    = make_calib();
    calib.d_right = cv::Mat::zeros(1, 4, CV_64F); // 1x4, needs 1x5
    EXPECT_THROW(score::StereoRectifier{calib}, std::invalid_argument);
}

TEST(StereoRectifierTest, InvalidRotationSizeThrows) {
    auto calib = make_calib();
    calib.r    = cv::Mat::eye(3, 2, CV_64F); // 3x2, needs 3x3
    EXPECT_THROW(score::StereoRectifier{calib}, std::invalid_argument);
}

TEST(StereoRectifierTest, InvalidTranslationSizeThrows) {
    auto calib = make_calib();
    calib.t    = cv::Mat::zeros(1, 3, CV_64F); // 1x3, needs 3x1
    EXPECT_THROW(score::StereoRectifier{calib}, std::invalid_argument);
}

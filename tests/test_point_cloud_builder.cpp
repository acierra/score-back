// Tests for PointCloudBuilder: CV_16S / CV_32F handling, depth filtering,
// NaN propagation, and valid_points() correctness.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include "cloud_point/point_cloud_builder.hpp"
#include "cloud_point/stereo_rectifier.hpp"

namespace {

// ---------------------------------------------------------------------------
// Shared synthetic calibration (same as test_stereo_rectifier.cpp)
// ---------------------------------------------------------------------------

constexpr double kFx       = 800.0;
constexpr double kFy       = 800.0;
constexpr double kCx       = 320.0;
constexpr double kCy       = 240.0;
constexpr int kWidth       = 640;
constexpr int kHeight      = 480;
constexpr double kBaseline = 0.06; // metres
constexpr double kTx       = -kBaseline;

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
    calib.r       = cv::Mat::eye(3, 3, CV_64F);
    calib.t       = (cv::Mat_<double>(3, 1) << kTx, 0.0, 0.0);
    // clang-format on
    calib.image_size = cv::Size(kWidth, kHeight);
    return calib;
}

cv::Mat make_q() {
    score::StereoRectifier rectifier(make_calib());
    return rectifier.q().clone();
}

// ---------------------------------------------------------------------------
// Helper: count non-NaN (valid) pixels in the cloud
// ---------------------------------------------------------------------------

int count_valid(const score::PointCloud &cloud) {
    int n = 0;
    for (int i = 0; i < cloud.height * cloud.width; ++i) {
        if (!std::isnan(cloud.data[static_cast<size_t>(i) * 3]))
            ++n;
    }
    return n;
}

} // namespace

// ---------------------------------------------------------------------------
// Constant-disparity CV_16S → expected depth = fx * baseline / disparity
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderTest, ConstantDisparityCV16S) {
    const cv::Mat q = make_q();
    score::PointCloudBuilder builder(q);

    // d = 32 px, stored as SGBM fixed-point (d * 16)
    constexpr float kDisp = 32.0f;
    constexpr float kExpectedZ =
        static_cast<float>(kFx * kBaseline / kDisp); // 1.5 m

    cv::Mat disparity(kHeight, kWidth, CV_16S,
                      cv::Scalar(static_cast<short>(kDisp * 16)));

    const auto cloud = builder.build(disparity);

    EXPECT_EQ(cloud.width, kWidth);
    EXPECT_EQ(cloud.height, kHeight);
    EXPECT_EQ(static_cast<int>(cloud.data.size()), kWidth * kHeight * 3);

    // Every point should be valid and at z ≈ 1.5 m
    const auto valid = cloud.valid_points();
    EXPECT_EQ(static_cast<int>(valid.size()), kWidth * kHeight);

    for (const auto &pt : valid) {
        EXPECT_NEAR(pt[2], kExpectedZ, 1e-3f)
            << "z should be fx*baseline/disparity = " << kExpectedZ << " m";
    }
}

TEST(PointCloudBuilderTest, CenterPixelXYNearZero) {
    const cv::Mat q = make_q();
    score::PointCloudBuilder builder(q);

    constexpr float kDisp = 32.0f;
    cv::Mat disparity(kHeight, kWidth, CV_16S,
                      cv::Scalar(static_cast<short>(kDisp * 16)));

    const auto cloud = builder.build(disparity);

    // At the image center (row=cy=240, col=cx=320), x≈0 and y≈0.
    const int center_idx =
        (static_cast<int>(kCy) * kWidth + static_cast<int>(kCx)) * 3;
    EXPECT_NEAR(cloud.data[static_cast<size_t>(center_idx)], 0.0f, 0.05f)
        << "x at image center should be near 0";
    EXPECT_NEAR(cloud.data[static_cast<size_t>(center_idx) + 1], 0.0f, 0.05f)
        << "y at image center should be near 0";
}

// ---------------------------------------------------------------------------
// Zero / negative disparity → NaN
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderTest, ZeroDisparityProducesNaN) {
    const cv::Mat q = make_q();
    score::PointCloudBuilder builder(q);

    cv::Mat disparity = cv::Mat::zeros(kHeight, kWidth, CV_16S);
    const auto cloud  = builder.build(disparity);

    EXPECT_EQ(count_valid(cloud), 0)
        << "All points should be NaN when disparity is zero";
    EXPECT_EQ(static_cast<int>(cloud.valid_points().size()), 0);
}

TEST(PointCloudBuilderTest, NegativeDisparityProducesNaN) {
    const cv::Mat q = make_q();
    score::PointCloudBuilder builder(q);

    cv::Mat disparity(kHeight, kWidth, CV_16S, cv::Scalar(-16)); // -1 px
    const auto cloud = builder.build(disparity);

    EXPECT_EQ(count_valid(cloud), 0)
        << "All points should be NaN when disparity is negative";
}

// ---------------------------------------------------------------------------
// Out-of-range depth filter → all NaN
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderTest, MaxDepthFilterRejectsAll) {
    const cv::Mat q = make_q();
    // kExpectedZ = 1.5 m; set max_depth_m = 1.0 to reject all points.
    score::PointCloudBuilder::Options opts;
    opts.max_depth_m = 1.0f;
    score::PointCloudBuilder builder(q, opts);

    constexpr float kDisp = 32.0f; // → z = 1.5 m, which exceeds max
    cv::Mat disparity(kHeight, kWidth, CV_16S,
                      cv::Scalar(static_cast<short>(kDisp * 16)));

    const auto cloud = builder.build(disparity);
    EXPECT_EQ(count_valid(cloud), 0)
        << "All points should be NaN when z=1.5 m exceeds max_depth_m=1.0 m";
}

TEST(PointCloudBuilderTest, MinDepthFilterRejectsAll) {
    const cv::Mat q = make_q();
    // kExpectedZ = 1.5 m; set min_depth_m = 2.0 to reject all points.
    score::PointCloudBuilder::Options opts;
    opts.min_depth_m = 2.0f;
    score::PointCloudBuilder builder(q, opts);

    constexpr float kDisp = 32.0f; // → z = 1.5 m, below min
    cv::Mat disparity(kHeight, kWidth, CV_16S,
                      cv::Scalar(static_cast<short>(kDisp * 16)));

    const auto cloud = builder.build(disparity);
    EXPECT_EQ(count_valid(cloud), 0)
        << "All points should be NaN when z=1.5 m is below min_depth_m=2.0 m";
}

// ---------------------------------------------------------------------------
// valid_points() count matches non-NaN count
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderTest, ValidPointsCountMatchesNonNaN) {
    const cv::Mat q = make_q();
    score::PointCloudBuilder builder(q);

    // Half the image has positive disparity, half has zero
    cv::Mat disparity(kHeight, kWidth, CV_16S, cv::Scalar(0));
    const cv::Rect left_half(0, 0, kWidth / 2, kHeight);
    disparity(left_half).setTo(cv::Scalar(32 * 16));

    const auto cloud       = builder.build(disparity);

    const int manual_count = count_valid(cloud);
    const int method_count = static_cast<int>(cloud.valid_points().size());
    EXPECT_EQ(manual_count, method_count)
        << "valid_points() size must equal the number of non-NaN pixels";
}

// ---------------------------------------------------------------------------
// CV_32F input works identically
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderTest, CV32FInputMatchesCV16S) {
    const cv::Mat q = make_q();
    score::PointCloudBuilder builder(q);

    constexpr float kDisp = 32.0f;

    // CV_16S reference
    cv::Mat disp16(kHeight, kWidth, CV_16S,
                   cv::Scalar(static_cast<short>(kDisp * 16)));
    const auto cloud16 = builder.build(disp16);

    // CV_32F equivalent
    cv::Mat disp32(kHeight, kWidth, CV_32F, cv::Scalar(kDisp));
    const auto cloud32 = builder.build(disp32);

    ASSERT_EQ(cloud16.data.size(), cloud32.data.size());

    // Every z-coordinate should match within floating-point tolerance
    for (int i = 0; i < kHeight * kWidth; ++i) {
        const float z16 = cloud16.data[static_cast<size_t>(i) * 3 + 2];
        const float z32 = cloud32.data[static_cast<size_t>(i) * 3 + 2];
        EXPECT_NEAR(z16, z32, 1e-3f) << "at pixel " << i;
    }
}

// ---------------------------------------------------------------------------
// Unsupported disparity type → throws
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderTest, CV8UInputThrows) {
    const cv::Mat q = make_q();
    score::PointCloudBuilder builder(q);

    cv::Mat bad_disparity(kHeight, kWidth, CV_8U, cv::Scalar(32));
    EXPECT_THROW(builder.build(bad_disparity), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Invalid Q matrix → constructor throws
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderTest, InvalidQSizeThrows) {
    cv::Mat bad_q = cv::Mat::eye(3, 4, CV_64F); // not 4x4
    EXPECT_THROW(score::PointCloudBuilder{bad_q}, std::invalid_argument);
}

TEST(PointCloudBuilderTest, InvalidQTypeThrows) {
    cv::Mat bad_q = cv::Mat::eye(4, 4, CV_32F); // not CV_64F
    EXPECT_THROW(score::PointCloudBuilder{bad_q}, std::invalid_argument);
}

// ===========================================================================
// Disparity offset, reason map and outlier filter
// (OpenSpec change improve-reconstruction-quality, tasks 1.1 / 1.2)
// ===========================================================================

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>

#include <opencv2/calib3d.hpp>

namespace {

using Reason  = score::PointCloudBuilder::Reason;
using Options = score::PointCloudBuilder::Options;

// SCARED-like rig: fx ≈ 1035 px, baseline ≈ 4.14 mm, so f·B ≈ 4.29 px·m.
constexpr double kScaredFx       = 1035.0;
constexpr double kScaredBaseline = 0.00414; // metres
constexpr double kScaredFB       = kScaredFx * kScaredBaseline;

cv::Mat make_scared_q(int width, int height) {
    score::StereoRectifier::Calibration calib;
    // clang-format off
    calib.k_left = (cv::Mat_<double>(3, 3) <<
        kScaredFx, 0,         width / 2.0,
        0,         kScaredFx, height / 2.0,
        0,         0,         1);
    // clang-format on
    calib.k_right = calib.k_left.clone();
    calib.d_left  = cv::Mat::zeros(1, 5, CV_64F);
    calib.d_right = cv::Mat::zeros(1, 5, CV_64F);
    calib.r       = cv::Mat::eye(3, 3, CV_64F);
    calib.t       = (cv::Mat_<double>(3, 1) << -kScaredBaseline, 0.0, 0.0);
    calib.image_size = cv::Size(width, height);
    score::StereoRectifier rectifier(calib);
    return rectifier.q().clone();
}

// Byte-level equality of two clouds (NaN patterns included).
bool clouds_identical(const score::PointCloud &a, const score::PointCloud &b) {
    return a.width == b.width && a.height == b.height &&
           a.data.size() == b.data.size() &&
           std::memcmp(a.data.data(), b.data.data(),
                       a.data.size() * sizeof(float)) == 0;
}

bool point_identical(const score::PointCloud &a, const score::PointCloud &b,
                     int r, int c) {
    const size_t idx =
        (static_cast<size_t>(r) * static_cast<size_t>(a.width) +
         static_cast<size_t>(c)) *
        3u;
    return std::memcmp(&a.data[idx], &b.data[idx], 3 * sizeof(float)) == 0;
}

bool point_is_nan(const score::PointCloud &a, int r, int c) {
    const size_t idx =
        (static_cast<size_t>(r) * static_cast<size_t>(a.width) +
         static_cast<size_t>(c)) *
        3u;
    return std::isnan(a.data[idx]) && std::isnan(a.data[idx + 1]) &&
           std::isnan(a.data[idx + 2]);
}

float z_at(const score::PointCloud &a, int r, int c) {
    return a.data[(static_cast<size_t>(r) * static_cast<size_t>(a.width) +
                   static_cast<size_t>(c)) *
                      3u +
                  2];
}

std::uint8_t reason_at(const cv::Mat &reason, int r, int c) {
    return reason.at<std::uint8_t>(r, c);
}

// Reference reconstruction implementing the pre-change rule (reproject with
// Q, invalidate d ≤ 0, |z| ≥ 10000 and out-of-range depths as NaN).
score::PointCloud reference_cloud(const cv::Mat &disparity, const cv::Mat &q,
                                  float min_depth_m, float max_depth_m) {
    cv::Mat disp32;
    if (disparity.type() == CV_16S) {
        disparity.convertTo(disp32, CV_32F, 1.0 / 16.0);
    } else {
        disp32 = disparity;
    }
    cv::Mat xyz;
    cv::reprojectImageTo3D(disp32, xyz, q, false);

    score::PointCloud cloud;
    cloud.width  = disparity.cols;
    cloud.height = disparity.rows;
    cloud.data.assign(static_cast<size_t>(cloud.width) *
                          static_cast<size_t>(cloud.height) * 3u,
                      std::numeric_limits<float>::quiet_NaN());
    for (int r = 0; r < cloud.height; ++r) {
        for (int c = 0; c < cloud.width; ++c) {
            const auto &pt = xyz.at<cv::Vec3f>(r, c);
            const float d  = disp32.at<float>(r, c);
            const float z  = pt[2];
            const bool invalid = (d <= 0.0f) || (std::abs(z) >= 10000.0f) ||
                                 (z < min_depth_m) || (z > max_depth_m);
            if (!invalid) {
                const size_t idx =
                    (static_cast<size_t>(r) *
                         static_cast<size_t>(cloud.width) +
                     static_cast<size_t>(c)) *
                    3u;
                cloud.data[idx]     = pt[0];
                cloud.data[idx + 1] = pt[1];
                cloud.data[idx + 2] = z;
            }
        }
    }
    return cloud;
}

// A varied CV_16S map: a horizontal disparity ramp, a zero band, a negative
// band, and a near band that lands outside the depth range.
cv::Mat make_varied_disparity(int width, int height) {
    cv::Mat disp(height, width, CV_16S);
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            short v = static_cast<short>((20 + (c * 60) / width) * 16 + (r % 7));
            if (r < height / 8) {
                v = 0; // matcher sentinel
            } else if (r < height / 4) {
                v = -16; // negative sentinel
            } else if (r > height - height / 8) {
                v = 2000 * 16; // very near → below min_depth_m
            }
            disp.at<short>(r, c) = v;
        }
    }
    return disp;
}

Options scared_opts() {
    Options opts;
    opts.min_depth_m = 0.02f;
    opts.max_depth_m = 0.30f;
    return opts;
}

// Disparity of a plane tilted 45° in depth,
//     Z = z0 + X·cos φ + Y·sin φ      (X, Y lateral in metres, φ = 30°),
// i.e. |∇Z| = 1, so the normal makes 45° with the optical axis.  The tilt
// azimuth φ is chosen so that no neighbour in a square window shares the
// centre pixel's depth: with an axis-aligned or diagonal tilt the window is
// point-symmetric and the neighbour median equals the centre exactly, which
// would make the filter test vacuous.  Output is SGBM fixed-point CV_16S.
constexpr double kPlaneAzimuthRad = 30.0 * M_PI / 180.0;

cv::Mat make_tilted_plane_disparity(int width, int height, double z0_m) {
    cv::Mat disp(height, width, CV_16S);
    const double cx = width / 2.0;
    const double cy = height / 2.0;
    const double a  = std::cos(kPlaneAzimuthRad) / kScaredFx;
    const double b  = std::sin(kPlaneAzimuthRad) / kScaredFx;
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            // X = (c-cx) Z/f, Y = (r-cy) Z/f  ⇒  Z = z0 / (1 - a(c-cx) - b(r-cy))
            const double z = z0_m / (1.0 - a * (c - cx) - b * (r - cy));
            const double d = kScaredFB / z;
            disp.at<short>(r, c) = static_cast<short>(std::lround(d * 16.0));
        }
    }
    return disp;
}

} // namespace

// ---------------------------------------------------------------------------
// 1.1 Disparity offset
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderOffsetTest, OffsetZeroIsByteIdentical) {
    const int w = 320, h = 200;
    const cv::Mat q    = make_scared_q(w, h);
    const cv::Mat disp = make_varied_disparity(w, h);

    Options opts = scared_opts();
    const auto reference =
        reference_cloud(disp, q, opts.min_depth_m, opts.max_depth_m);

    // Default options (offset 0, filter disabled) reproduce the pre-change
    // reconstruction bit for bit.
    score::PointCloudBuilder builder(q, opts);
    const auto cloud = builder.build(disp);
    EXPECT_TRUE(clouds_identical(cloud, reference));

    // An explicit offset of exactly 0 takes the same untouched path.
    opts.disparity_offset_px = 0.0;
    score::PointCloudBuilder explicit_zero(q, opts);
    EXPECT_TRUE(clouds_identical(explicit_zero.build(disp), reference));

    // Also for CV_32F input.
    cv::Mat disp32;
    disp.convertTo(disp32, CV_32F, 1.0 / 16.0);
    EXPECT_TRUE(clouds_identical(
        builder.build(disp32),
        reference_cloud(disp32, q, opts.min_depth_m, opts.max_depth_m)));

    // Some pixels of each class exist in the fixture.
    EXPECT_GT(count_valid(cloud), 0);
    EXPECT_LT(count_valid(cloud), w * h);
}

TEST(PointCloudBuilderOffsetTest, PositiveOffsetGivesDepthOfShiftedDisparity) {
    const int w = 64, h = 48;
    const cv::Mat q = make_scared_q(w, h);

    Options opts = scared_opts();
    opts.disparity_offset_px = 0.5;
    score::PointCloudBuilder shifted(q, opts);
    score::PointCloudBuilder plain(q, scared_opts());

    // 50 px + 0.5 px offset must equal a 50.5 px map without offset.
    cv::Mat disp50(h, w, CV_32F, cv::Scalar(50.0f));
    cv::Mat disp50_5(h, w, CV_32F, cv::Scalar(50.5f));
    const auto with_offset = shifted.build(disp50);
    const auto expected    = plain.build(disp50_5);
    EXPECT_TRUE(clouds_identical(with_offset, expected));

    const float expected_z = static_cast<float>(kScaredFB / 50.5);
    EXPECT_EQ(count_valid(with_offset), w * h);
    for (const auto &pt : with_offset.valid_points()) {
        EXPECT_NEAR(pt[2], expected_z, 1e-6f);
    }
    // Sanity: the points really moved closer than at 50 px.
    EXPECT_LT(expected_z, static_cast<float>(kScaredFB / 50.0));

    // Same for SGBM fixed-point input (50 px = 800).
    cv::Mat disp16(h, w, CV_16S, cv::Scalar(800));
    EXPECT_TRUE(clouds_identical(shifted.build(disp16), expected));
}

TEST(PointCloudBuilderOffsetTest, NegativeOffsetDrivingDisparityNonPositiveIsInvalid) {
    const int w = 16, h = 8;
    const cv::Mat q = make_scared_q(w, h);

    Options opts = scared_opts();
    opts.disparity_offset_px = -0.5;
    score::PointCloudBuilder builder(q, opts);

    // All pixels at 50 px except (3,5) at 0.25 px and (2,2) at raw 0.
    cv::Mat disp(h, w, CV_32F, cv::Scalar(50.0f));
    disp.at<float>(3, 5) = 0.25f;
    disp.at<float>(2, 2) = 0.0f;

    cv::Mat reason;
    const auto cloud = builder.build(disp, &reason);

    EXPECT_TRUE(point_is_nan(cloud, 3, 5)) << "0.25 − 0.5 ≤ 0 must be invalid";
    EXPECT_EQ(reason_at(reason, 3, 5),
              static_cast<std::uint8_t>(Reason::matcher_rejected));
    EXPECT_TRUE(point_is_nan(cloud, 2, 2));
    EXPECT_EQ(reason_at(reason, 2, 2),
              static_cast<std::uint8_t>(Reason::matcher_rejected));
    EXPECT_EQ(count_valid(cloud), w * h - 2);

    // No point may end up behind the camera.
    for (const auto &pt : cloud.valid_points()) {
        EXPECT_GT(pt[2], 0.0f);
    }

    // Exactly-zero corrected disparity (0.5 − 0.5) is also invalid.
    disp.at<float>(3, 5) = 0.5f;
    const auto cloud2 = builder.build(disp, &reason);
    EXPECT_TRUE(point_is_nan(cloud2, 3, 5));
    EXPECT_EQ(reason_at(reason, 3, 5),
              static_cast<std::uint8_t>(Reason::matcher_rejected));
}

TEST(PointCloudBuilderOffsetTest, OffsetNotAppliedToInvalidRawDisparity) {
    const int w = 8, h = 8;
    const cv::Mat q = make_scared_q(w, h);
    Options opts = scared_opts();
    opts.disparity_offset_px = 5.0; // would make 0 and −1 px positive
    score::PointCloudBuilder builder(q, opts);

    cv::Mat disp(h, w, CV_32F, cv::Scalar(0.0f));
    disp.at<float>(1, 1) = -1.0f;
    cv::Mat reason;
    const auto cloud = builder.build(disp, &reason);
    EXPECT_EQ(count_valid(cloud), 0);
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            EXPECT_EQ(reason_at(reason, r, c),
                      static_cast<std::uint8_t>(Reason::matcher_rejected));
}

TEST(PointCloudBuilderOffsetTest, NonFiniteOffsetRejected) {
    const cv::Mat q = make_scared_q(8, 8);
    Options opts = scared_opts();
    opts.disparity_offset_px = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(score::PointCloudBuilder(q, opts), std::invalid_argument);
    opts.disparity_offset_px = std::numeric_limits<double>::infinity();
    EXPECT_THROW(score::PointCloudBuilder(q, opts), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// 1.1 Reason map
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderReasonTest, ReasonCodesCoverEveryInvalidationPath) {
    const int w = 32, h = 32;
    const cv::Mat q = make_scared_q(w, h);

    Options opts = scared_opts(); // 20–300 mm
    opts.outlier_filter.enabled = true; // window 5, abs 2 mm, rel 0.03, min 6
    score::PointCloudBuilder builder(q, opts);

    // Base: 80 mm plane.
    const float d80 = static_cast<float>(kScaredFB / 0.080);
    cv::Mat disp(h, w, CV_32F, cv::Scalar(d80));
    disp.at<float>(2, 2)   = 0.0f;                                  // matcher
    disp.at<float>(4, 4)   = -3.0f;                                 // matcher
    disp.at<float>(8, 8)   = static_cast<float>(kScaredFB / 0.010); // 10 mm
    disp.at<float>(12, 12) = static_cast<float>(kScaredFB / 0.500); // 500 mm
    disp.at<float>(20, 20) = static_cast<float>(kScaredFB / 0.200); // spike

    cv::Mat reason;
    const auto cloud = builder.build(disp, &reason);

    ASSERT_EQ(reason.type(), CV_8U);
    ASSERT_EQ(reason.rows, h);
    ASSERT_EQ(reason.cols, w);

    const auto expect = [&](int r, int c, Reason why) {
        EXPECT_EQ(reason_at(reason, r, c), static_cast<std::uint8_t>(why))
            << "at (" << r << "," << c << ") expected "
            << score::PointCloudBuilder::reason_name(why) << " got "
            << score::PointCloudBuilder::reason_name(
                   static_cast<Reason>(reason_at(reason, r, c)));
        EXPECT_EQ(point_is_nan(cloud, r, c), why != Reason::valid);
    };
    expect(2, 2, Reason::matcher_rejected);
    expect(4, 4, Reason::matcher_rejected);
    expect(8, 8, Reason::depth_range);
    expect(12, 12, Reason::depth_range);
    expect(20, 20, Reason::outlier_filter);
    expect(0, 0, Reason::valid);
    expect(h - 1, w - 1, Reason::valid);
    expect(16, 16, Reason::valid);

    // Every other pixel is valid and coded as such.
    int n_valid = 0;
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            if (reason_at(reason, r, c) ==
                static_cast<std::uint8_t>(Reason::valid))
                ++n_valid;
    EXPECT_EQ(n_valid, w * h - 5);
    EXPECT_EQ(count_valid(cloud), n_valid);

    // Enum values are stable for downstream (diagnostics, JSON).
    EXPECT_EQ(static_cast<int>(Reason::valid), 0);
    EXPECT_EQ(static_cast<int>(Reason::matcher_rejected), 1);
    EXPECT_EQ(static_cast<int>(Reason::depth_range), 2);
    EXPECT_EQ(static_cast<int>(Reason::outlier_filter), 3);
    EXPECT_STREQ(score::PointCloudBuilder::reason_name(Reason::outlier_filter),
                 "outlier_filter");
}

TEST(PointCloudBuilderReasonTest, ReasonMapMatchesNaNPatternWithoutFilter) {
    const int w = 320, h = 200;
    const cv::Mat q    = make_scared_q(w, h);
    const cv::Mat disp = make_varied_disparity(w, h);
    score::PointCloudBuilder builder(q, scared_opts());

    cv::Mat reason;
    const auto cloud = builder.build(disp, &reason);
    // Passing the out-parameter does not change the cloud.
    EXPECT_TRUE(clouds_identical(cloud, builder.build(disp)));

    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const auto why = static_cast<Reason>(reason_at(reason, r, c));
            EXPECT_EQ(point_is_nan(cloud, r, c), why != Reason::valid);
            EXPECT_NE(why, Reason::outlier_filter);
            if (disp.at<short>(r, c) <= 0) {
                EXPECT_EQ(why, Reason::matcher_rejected);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 1.2 Outlier filter
// ---------------------------------------------------------------------------

TEST(PointCloudBuilderOutlierFilterTest, IsolatedSpikeRemovedNothingElseChanges) {
    const int w = 64, h = 48;
    const cv::Mat q = make_scared_q(w, h);

    Options opts = scared_opts();
    opts.outlier_filter.enabled         = true;
    opts.outlier_filter.window          = 5;
    opts.outlier_filter.abs_threshold_m = 0.002;
    score::PointCloudBuilder filtered(q, opts);
    score::PointCloudBuilder unfiltered(q, scared_opts());

    cv::Mat disp(h, w, CV_32F,
                 cv::Scalar(static_cast<float>(kScaredFB / 0.080)));
    const int sr = 20, sc = 30;
    disp.at<float>(sr, sc) = static_cast<float>(kScaredFB / 0.200);

    cv::Mat reason;
    const auto before = unfiltered.build(disp);
    const auto after  = filtered.build(disp, &reason);

    // The spike was a valid 200 mm point before filtering …
    EXPECT_NEAR(z_at(before, sr, sc), 0.200f, 1e-4f);
    // … and is invalid after.
    EXPECT_TRUE(point_is_nan(after, sr, sc));
    EXPECT_EQ(reason_at(reason, sr, sc),
              static_cast<std::uint8_t>(Reason::outlier_filter));

    int changed = 0;
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (r == sr && c == sc) continue;
            if (!point_identical(before, after, r, c)) ++changed;
            EXPECT_EQ(reason_at(reason, r, c),
                      static_cast<std::uint8_t>(Reason::valid));
        }
    }
    EXPECT_EQ(changed, 0) << "only the spike may change";
    EXPECT_EQ(count_valid(after), count_valid(before) - 1);
}

TEST(PointCloudBuilderOutlierFilterTest, TooFewNeighboursInvalidates) {
    const int w = 32, h = 32;
    const cv::Mat q = make_scared_q(w, h);
    Options opts = scared_opts();
    opts.outlier_filter.enabled        = true;
    opts.outlier_filter.min_neighbours = 6;
    score::PointCloudBuilder builder(q, opts);

    // One lonely valid pixel in a sea of matcher rejections, plus a pair.
    cv::Mat disp = cv::Mat::zeros(h, w, CV_32F);
    const float d80 = static_cast<float>(kScaredFB / 0.080);
    disp.at<float>(10, 10) = d80;          // 0 neighbours → rejected
    disp.at<float>(20, 20) = d80;          // 1 neighbour  → rejected
    disp.at<float>(20, 21) = d80;
    // A 2×2 block: each pixel sees 3 neighbours in a 5×5 window → all go.
    disp(cv::Rect(4, 4, 2, 2)).setTo(cv::Scalar(d80));
    // A 3×3 block: each pixel sees the other 8 → all stay.
    disp(cv::Rect(4, 26, 3, 3)).setTo(cv::Scalar(d80));

    cv::Mat reason;
    const auto cloud = builder.build(disp, &reason);
    EXPECT_EQ(reason_at(reason, 10, 10),
              static_cast<std::uint8_t>(Reason::outlier_filter));
    EXPECT_EQ(reason_at(reason, 20, 20),
              static_cast<std::uint8_t>(Reason::outlier_filter));
    EXPECT_EQ(reason_at(reason, 20, 21),
              static_cast<std::uint8_t>(Reason::outlier_filter));
    for (int r = 4; r < 6; ++r) {
        for (int c = 4; c < 6; ++c) {
            EXPECT_EQ(reason_at(reason, r, c),
                      static_cast<std::uint8_t>(Reason::outlier_filter));
        }
    }
    for (int r = 26; r < 29; ++r) {
        for (int c = 4; c < 7; ++c) {
            EXPECT_EQ(reason_at(reason, r, c),
                      static_cast<std::uint8_t>(Reason::valid));
        }
    }
    EXPECT_EQ(count_valid(cloud), 9);
    // Untouched matcher rejections keep their own reason.
    EXPECT_EQ(reason_at(reason, 0, 0),
              static_cast<std::uint8_t>(Reason::matcher_rejected));
}

TEST(PointCloudBuilderOutlierFilterTest, GenuineSlopeKeptAtDefaultThresholds) {
    // 45° plane Z = z0 + X cos φ + Y sin φ spanning ~81–149 mm across a
    // 480×384 image with a SCARED-like rig, quantised to SGBM's 1/16 px.
    const int w = 480, h = 384;
    const double z0 = 0.105;
    const cv::Mat q    = make_scared_q(w, h);
    const cv::Mat disp = make_tilted_plane_disparity(w, h, z0);

    Options opts = scared_opts();
    opts.outlier_filter.enabled = true; // defaults: 5, 2 mm, 0.03, 6
    score::PointCloudBuilder filtered(q, opts);
    score::PointCloudBuilder unfiltered(q, scared_opts());

    const auto before = unfiltered.build(disp);
    ASSERT_EQ(count_valid(before), w * h);
    // Confirm the fixture really is a 45° slope at endoscopic depth: the
    // reconstructed points satisfy the plane equation (to within the 1/16 px
    // quantisation, ≈ 0.3 mm at 150 mm) and depth spans ~81–149 mm.
    EXPECT_GT(z_at(before, 0, 0), 0.075f);
    EXPECT_LT(z_at(before, h - 1, w - 1), 0.155f);
    {
        double worst_plane_residual = 0.0;
        for (int r = 0; r < h; r += 31) {
            for (int c = 0; c < w; c += 37) {
                const size_t i = (static_cast<size_t>(r) * w + c) * 3u;
                const double X = before.data[i], Y = before.data[i + 1],
                             Z = before.data[i + 2];
                worst_plane_residual = std::max(
                    worst_plane_residual,
                    std::abs(Z - z0 - X * std::cos(kPlaneAzimuthRad) -
                             Y * std::sin(kPlaneAzimuthRad)));
            }
        }
        EXPECT_LT(worst_plane_residual, 0.0004) << "fixture must be the plane";
        // The corner-to-corner depth change is a genuine 45° slope: about
        // 68 mm of Z over the diagonal's lateral extent.
        EXPECT_GT(z_at(before, h - 1, w - 1) - z_at(before, 0, 0), 0.060f);
    }

    cv::Mat reason;
    const auto after = filtered.build(disp, &reason);

    const int radius = opts.outlier_filter.window / 2;
    int interior_rejected = 0, interior_changed = 0, total_rejected = 0;
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const bool rejected =
                reason_at(reason, r, c) ==
                static_cast<std::uint8_t>(Reason::outlier_filter);
            total_rejected += rejected;
            const bool interior = r >= radius && r < h - radius &&
                                  c >= radius && c < w - radius;
            if (interior) {
                interior_rejected += rejected;
                interior_changed += !point_identical(before, after, r, c);
            }
        }
    }
    EXPECT_EQ(interior_rejected, 0) << "plane interior must be untouched";
    EXPECT_EQ(interior_changed, 0);
    EXPECT_EQ(total_rejected, 0) << "borders still have ≥ 8 neighbours";
    EXPECT_TRUE(clouds_identical(before, after));

    // Report the worst-case |z − neighbourhood median| against the threshold,
    // so a future default change can be judged against real margins.
    double worst_ratio = 0.0;
    for (int r = radius; r < h - radius; r += 7) {
        for (int c = radius; c < w - radius; c += 5) {
            std::vector<double> zs;
            for (int rr = r - radius; rr <= r + radius; ++rr)
                for (int cc = c - radius; cc <= c + radius; ++cc)
                    if (rr != r || cc != c) zs.push_back(z_at(before, rr, cc));
            std::nth_element(zs.begin(), zs.begin() + 12, zs.end());
            const double med = zs[12];
            const double tol = std::max(0.002, 0.03 * med);
            worst_ratio = std::max(worst_ratio,
                                   std::abs(z_at(before, r, c) - med) / tol);
        }
    }
    std::cout << "[ INFO ] 45° plane: worst |z-median|/threshold = "
              << worst_ratio << " (must stay < 1)\n";
    EXPECT_LT(worst_ratio, 1.0);
}

TEST(PointCloudBuilderOutlierFilterTest, DisabledFilterLeavesCloudIdentical) {
    const int w = 320, h = 200;
    const cv::Mat q    = make_scared_q(w, h);
    cv::Mat disp       = make_varied_disparity(w, h);
    disp.at<short>(100, 100) = static_cast<short>(kScaredFB / 0.25 * 16); // spike

    Options plain = scared_opts();
    Options off   = scared_opts();
    off.outlier_filter.enabled         = false;
    off.outlier_filter.window          = 7;
    off.outlier_filter.abs_threshold_m = 0.0;
    off.outlier_filter.rel_threshold   = 0.0;
    off.outlier_filter.min_neighbours  = 48;
    Options on = scared_opts();
    on.outlier_filter.enabled = true;

    const auto ref     = score::PointCloudBuilder(q, plain).build(disp);
    cv::Mat reason_off;
    const auto cloud_off =
        score::PointCloudBuilder(q, off).build(disp, &reason_off);
    EXPECT_TRUE(clouds_identical(cloud_off, ref));
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            EXPECT_NE(reason_at(reason_off, r, c),
                      static_cast<std::uint8_t>(Reason::outlier_filter));

    // Sanity: enabling it does change something on this fixture.
    const auto cloud_on = score::PointCloudBuilder(q, on).build(disp);
    EXPECT_FALSE(clouds_identical(cloud_on, ref));
    EXPECT_TRUE(point_is_nan(cloud_on, 100, 100));
}

TEST(PointCloudBuilderOutlierFilterTest, InvalidOptionsRejected) {
    const cv::Mat q = make_scared_q(8, 8);
    const auto expect_throw = [&](auto mutate, const char *what) {
        Options opts = scared_opts();
        opts.outlier_filter.enabled = true;
        mutate(opts.outlier_filter);
        EXPECT_THROW(score::PointCloudBuilder(q, opts), std::invalid_argument)
            << what;
    };
    using F = Options::OutlierFilter;
    expect_throw([](F &f) { f.window = 4; }, "even window");
    expect_throw([](F &f) { f.window = 2; }, "window 2");
    expect_throw([](F &f) { f.window = 1; }, "window 1");
    expect_throw([](F &f) { f.window = -5; }, "negative window");
    expect_throw([](F &f) { f.abs_threshold_m = -0.001; }, "negative abs");
    expect_throw([](F &f) { f.rel_threshold = -0.1; }, "negative rel");
    expect_throw([](F &f) { f.abs_threshold_m = std::nan(""); }, "NaN abs");
    expect_throw([](F &f) { f.min_neighbours = -1; }, "negative min");
    expect_throw([](F &f) { f.min_neighbours = 25; }, "min ≥ window²");

    // Valid edge values are accepted.
    Options ok = scared_opts();
    ok.outlier_filter.enabled        = true;
    ok.outlier_filter.window         = 3;
    ok.outlier_filter.min_neighbours = 8;
    ok.outlier_filter.abs_threshold_m = 0.0;
    ok.outlier_filter.rel_threshold   = 0.0;
    EXPECT_NO_THROW(score::PointCloudBuilder(q, ok));
    ok.outlier_filter.window         = 7;
    ok.outlier_filter.min_neighbours = 0;
    EXPECT_NO_THROW(score::PointCloudBuilder(q, ok));

    // Validation applies only while the filter is enabled.
    Options disabled = scared_opts();
    disabled.outlier_filter.window = 4;
    EXPECT_NO_THROW(score::PointCloudBuilder(q, disabled));
}

TEST(PointCloudBuilderOutlierFilterTest, LargerWindowAndEvenNeighbourCount) {
    // window 7 (48 neighbours, even count → median averages the middle pair)
    const int w = 40, h = 40;
    const cv::Mat q = make_scared_q(w, h);
    Options opts = scared_opts();
    opts.outlier_filter.enabled = true;
    opts.outlier_filter.window  = 7;
    score::PointCloudBuilder builder(q, opts);

    cv::Mat disp(h, w, CV_32F,
                 cv::Scalar(static_cast<float>(kScaredFB / 0.100)));
    disp.at<float>(15, 15) = static_cast<float>(kScaredFB / 0.150);
    cv::Mat reason;
    const auto cloud = builder.build(disp, &reason);
    EXPECT_TRUE(point_is_nan(cloud, 15, 15));
    EXPECT_EQ(count_valid(cloud), w * h - 1);
}

TEST(PointCloudBuilderOutlierFilterTest, TimingOnSyntheticFullFrame) {
    const int w = 1280, h = 1024;
    const cv::Mat q = make_scared_q(w, h);

    // Realistic-ish fixture: tilted plane, 1/16 px quantisation, ±0.5 px
    // noise, 5 % random matcher holes, and a sprinkle of gross outliers.
    cv::Mat disp = make_tilted_plane_disparity(w, h, 0.105);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> noise(-8, 8);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            short &v = disp.at<short>(r, c);
            const double p = u(rng);
            if (p < 0.05) {
                v = 0;
            } else if (p < 0.055) {
                v = static_cast<short>(v / 2); // twice as far
            } else {
                v = static_cast<short>(v + noise(rng));
            }
        }
    }

    Options off = scared_opts();
    Options on  = scared_opts();
    on.outlier_filter.enabled = true;
    score::PointCloudBuilder plain(q, off);
    score::PointCloudBuilder filtered(q, on);

    using clock = std::chrono::steady_clock;
    const auto time_ms = [&](const score::PointCloudBuilder &b) {
        double best = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 5; ++i) {
            const auto t0 = clock::now();
            const auto cloud = b.build(disp);
            const auto t1 = clock::now();
            best = std::min(best, std::chrono::duration<double, std::milli>(
                                      t1 - t0)
                                      .count());
            (void)cloud;
        }
        return best;
    };

    const double ms_off = time_ms(plain);
    const double ms_on  = time_ms(filtered);
    const auto before   = plain.build(disp);
    const auto after    = filtered.build(disp);

    std::cout << "[ INFO ] 1280x1024 build: " << ms_off
              << " ms without filter, " << ms_on
              << " ms with outlier filter (window 5) -> filter cost ~"
              << (ms_on - ms_off) << " ms; threads="
              << cv::getNumThreads() << "; valid " << count_valid(before)
              << " -> " << count_valid(after) << "\n";
    ::testing::Test::RecordProperty("build_ms_no_filter", ms_off);
    ::testing::Test::RecordProperty("build_ms_with_filter", ms_on);
    ::testing::Test::RecordProperty("outlier_filter_ms", ms_on - ms_off);

    EXPECT_LT(count_valid(after), count_valid(before));
    EXPECT_GT(count_valid(after), count_valid(before) * 9 / 10);
}

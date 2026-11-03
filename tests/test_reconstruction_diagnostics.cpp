// Tests for diagnose_reconstruction(): depth-binned bias, calibration-error
// fits, tail counts, coverage-loss attribution and unscored fraction, on
// synthetic inputs with a SCARED-like reprojection matrix.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

#include "cloud_point/point_cloud_builder.hpp"
#include "cloud_point/reconstruction_diagnostics.hpp"

namespace {

// SCARED-like rig: f ≈ 1035 px, B ≈ 4.14 mm, so f·B ≈ 4.285 px·m.
constexpr double kF        = 1035.0;
constexpr double kBaseline = 0.00414; // metres
constexpr double kCx       = 160.0;
constexpr double kCy       = 120.0;
constexpr int kWidth       = 320;
constexpr int kHeight      = 240;
constexpr float kNaN       = std::numeric_limits<float>::quiet_NaN();

using Reason = score::PointCloudBuilder::Reason;

/// Q exactly as cv::stereoRectify lays it out with CALIB_ZERO_DISPARITY and
/// Tx = −B (Q(3,2) = −1/Tx = 1/B, Q(3,3) = 0).
cv::Mat make_q() {
    cv::Mat q          = cv::Mat::zeros(4, 4, CV_64F);
    q.at<double>(0, 0) = 1.0;
    q.at<double>(0, 3) = -kCx;
    q.at<double>(1, 1) = 1.0;
    q.at<double>(1, 3) = -kCy;
    q.at<double>(2, 3) = kF;
    q.at<double>(3, 2) = 1.0 / kBaseline;
    return q;
}

double disparity_of(double z) { return kF * kBaseline / z; }

/// Ground-truth depth ramp: z_lo at the top row to z_hi at the bottom row.
cv::Mat depth_ramp(double z_lo, double z_hi, int height = kHeight,
                   int width = kWidth) {
    cv::Mat z(height, width, CV_64F);
    for (int r = 0; r < height; ++r) {
        const double t  = static_cast<double>(r) / (height - 1);
        const double zr = z_lo + t * (z_hi - z_lo);
        for (int c = 0; c < width; ++c)
            z.at<double>(r, c) = zr;
    }
    return z;
}

cv::Mat disparity_from_depth(const cv::Mat &z) {
    cv::Mat d(z.size(), CV_32F);
    for (int r = 0; r < z.rows; ++r)
        for (int c = 0; c < z.cols; ++c)
            d.at<float>(r, c) =
                static_cast<float>(disparity_of(z.at<double>(r, c)));
    return d;
}

/// CV_32FC3 ground-truth map built with the same Q as the prediction.
cv::Mat gt_from_disparity(const cv::Mat &disparity) {
    score::PointCloudBuilder builder(make_q(),
                                     score::PointCloudBuilder::Options{0.001f,
                                                                       10.0f});
    const auto cloud = builder.build(disparity);
    cv::Mat gt(cloud.height, cloud.width, CV_32FC3);
    for (int r = 0; r < cloud.height; ++r)
        for (int c = 0; c < cloud.width; ++c) {
            const auto i =
                (static_cast<std::size_t>(r) * cloud.width + c) * 3u;
            gt.at<cv::Vec3f>(r, c) = cv::Vec3f(
                cloud.data[i], cloud.data[i + 1], cloud.data[i + 2]);
        }
    return gt;
}

score::PointCloud cloud_from_map(const cv::Mat &map) {
    score::PointCloud cloud;
    cloud.width  = map.cols;
    cloud.height = map.rows;
    cloud.data.reserve(static_cast<std::size_t>(map.total()) * 3u);
    for (int r = 0; r < map.rows; ++r)
        for (int c = 0; c < map.cols; ++c) {
            const auto p = map.at<cv::Vec3f>(r, c);
            cloud.data.push_back(p[0]);
            cloud.data.push_back(p[1]);
            cloud.data.push_back(p[2]);
        }
    return cloud;
}

void set_invalid(score::PointCloud &cloud, int r, int c) {
    const auto i     = (static_cast<std::size_t>(r) * cloud.width + c) * 3u;
    cloud.data[i]     = kNaN;
    cloud.data[i + 1] = kNaN;
    cloud.data[i + 2] = kNaN;
}

score::ReconstructionDiagnosticsInputs
make_inputs(const cv::Mat &reason, double offset = 0.0, float min_depth = 0.02f,
            float max_depth = 0.30f) {
    score::ReconstructionDiagnosticsInputs in;
    in.q                   = make_q();
    in.reason              = reason;
    in.min_disparity       = 0;
    in.num_disparities     = 160;
    in.disparity_offset_px = offset;
    in.min_depth_m         = min_depth;
    in.max_depth_m         = max_depth;
    return in;
}

double loss_sum(const score::ReconstructionDiagnostics &d) {
    const auto &l = d.coverage_loss;
    return d.coverage + l.out_of_search_range + l.image_border +
           l.matcher_rejected + l.depth_range + l.outlier_filter;
}

} // namespace

// ---------------------------------------------------------------------------
// Q geometry
// ---------------------------------------------------------------------------

TEST(ReprojectionGeometryTest, ReadsFocalAndBaselineFromQ) {
    const auto g = score::ReprojectionGeometry::from_q(make_q());
    EXPECT_DOUBLE_EQ(g.f, kF);
    EXPECT_NEAR(g.fB, kF * kBaseline, 1e-12);
    EXPECT_NEAR(g.disparity(0.06), disparity_of(0.06), 1e-9);
    EXPECT_NEAR(g.depth(g.disparity(0.06)), 0.06, 1e-12);
    // The builder and the geometry agree on the depth of a disparity.
    score::PointCloudBuilder builder(make_q());
    cv::Mat d(1, 1, CV_32F, cv::Scalar(50.0f));
    const auto cloud = builder.build(d);
    EXPECT_NEAR(cloud.data[2], g.depth(50.0), 1e-6);
}

TEST(ReprojectionGeometryTest, RejectsDegenerateQ) {
    cv::Mat q = make_q();
    q.at<double>(3, 2) = 0.0;
    EXPECT_THROW(score::ReprojectionGeometry::from_q(q),
                 std::invalid_argument);
    EXPECT_THROW(score::ReprojectionGeometry::from_q(cv::Mat::eye(3, 3, CV_64F)),
                 std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Depth-binned bias
// ---------------------------------------------------------------------------

TEST(ReconstructionDiagnosticsTest, BiasReportedPerTenMillimetreBin) {
    // GT spans 55–149.9 mm; prediction is 0.5 mm too far everywhere.
    const cv::Mat gt = gt_from_disparity(
        disparity_from_depth(depth_ramp(0.055, 0.1499)));
    cv::Mat pred_map = gt.clone();
    for (int r = 0; r < pred_map.rows; ++r)
        for (int c = 0; c < pred_map.cols; ++c)
            pred_map.at<cv::Vec3f>(r, c)[2] += 0.0005f;
    const auto pred = cloud_from_map(pred_map);
    cv::Mat reason  = cv::Mat::zeros(kHeight, kWidth, CV_8U);

    // Self-check of the fixture: an exact prediction has no bias.
    const auto exact = score::diagnose_reconstruction(cloud_from_map(gt), gt,
                                                      make_inputs(reason));
    EXPECT_NEAR(exact.bias_z_median_m, 0.0, 1e-9);

    const auto diag = score::diagnose_reconstruction(pred, gt,
                                                     make_inputs(reason));
    ASSERT_EQ(diag.depth_bias.size(), 10u); // [0.05,0.06) … [0.14,0.15)
    EXPECT_DOUBLE_EQ(diag.depth_bias.front().lo_m, 0.05);
    EXPECT_DOUBLE_EQ(diag.depth_bias.front().hi_m, 0.06);
    EXPECT_DOUBLE_EQ(diag.depth_bias.back().lo_m, 0.14);
    EXPECT_DOUBLE_EQ(diag.depth_bias.back().hi_m, 0.15);
    std::size_t total = 0;
    for (const auto &bin : diag.depth_bias) {
        EXPECT_GE(bin.count, 200u);
        EXPECT_NEAR(bin.hi_m - bin.lo_m, 0.01, 1e-12);
        EXPECT_NEAR(bin.bias_z_median_m, 0.0005, 1e-6);
        EXPECT_NEAR(bin.abs_3d_median_m, 0.0005, 1e-6);
        total += bin.count;
    }
    EXPECT_EQ(total, static_cast<std::size_t>(kWidth) * kHeight);
    EXPECT_NEAR(diag.bias_z_median_m, 0.0005, 1e-6);
    EXPECT_EQ(diag.matched_points, static_cast<std::size_t>(kWidth) * kHeight);
    EXPECT_DOUBLE_EQ(diag.coverage, 1.0);
}

TEST(ReconstructionDiagnosticsTest, SparseBinsAreOmittedAndRangeIsHonoured) {
    // 20 rows × 5 columns = 100 points per 10 mm step: below the 200 minimum.
    const cv::Mat gt = gt_from_disparity(
        disparity_from_depth(depth_ramp(0.055, 0.0649, 20, 5)));
    cv::Mat reason = cv::Mat::zeros(20, 5, CV_8U);
    const auto sparse =
        score::diagnose_reconstruction(cloud_from_map(gt), gt,
                                       make_inputs(reason));
    EXPECT_TRUE(sparse.depth_bias.empty());
    EXPECT_EQ(sparse.matched_points, 100u);
    EXPECT_NEAR(sparse.bias_z_median_m, 0.0, 1e-9);

    // Bins only cover the configured depth range: with min_depth 60 mm the
    // [0.05, 0.06) bin is absent even though GT populates it.
    const cv::Mat wide = gt_from_disparity(
        disparity_from_depth(depth_ramp(0.055, 0.1499)));
    cv::Mat reason_wide = cv::Mat::zeros(kHeight, kWidth, CV_8U);
    const auto clipped  = score::diagnose_reconstruction(
        cloud_from_map(wide), wide,
        make_inputs(reason_wide, 0.0, 0.06f, 0.30f));
    ASSERT_FALSE(clipped.depth_bias.empty());
    EXPECT_DOUBLE_EQ(clipped.depth_bias.front().lo_m, 0.06);
}

// ---------------------------------------------------------------------------
// Calibration-error fits (D3)
// ---------------------------------------------------------------------------

TEST(ReconstructionDiagnosticsTest, SyntheticDisparityOffsetRecovered) {
    // Reconstruct from d_gt − 0.1 px: the offset model should recover +0.1.
    const cv::Mat d_gt = disparity_from_depth(depth_ramp(0.04, 0.15));
    const cv::Mat gt   = gt_from_disparity(d_gt);
    cv::Mat d_rec      = d_gt - 0.1f;

    score::PointCloudBuilder builder(make_q(),
                                     score::PointCloudBuilder::Options{0.02f,
                                                                       0.30f});
    cv::Mat reason;
    const auto pred = builder.build(d_rec, &reason);

    const auto diag = score::diagnose_reconstruction(pred, gt,
                                                     make_inputs(reason));
    EXPECT_NEAR(diag.fit_disparity_offset_px, 0.1, 0.01);
    EXPECT_TRUE(std::isfinite(diag.fit_disparity_offset_residual_m));
    EXPECT_TRUE(std::isfinite(diag.fit_depth_scale_residual_m));
    EXPECT_LT(diag.fit_disparity_offset_residual_m,
              diag.fit_depth_scale_residual_m);
    EXPECT_LT(diag.fit_disparity_offset_residual_m, 1e-5);
    // The bias is "too far" (positive) and grows with depth.
    EXPECT_GT(diag.bias_z_median_m, 0.0);
    ASSERT_GE(diag.depth_bias.size(), 2u);
    EXPECT_LT(diag.depth_bias.front().bias_z_median_m,
              diag.depth_bias.back().bias_z_median_m);
}

TEST(ReconstructionDiagnosticsTest, SyntheticDepthScaleRecovered) {
    // Reconstruction is the ground truth scaled by 1.01 about the camera.
    const cv::Mat gt = gt_from_disparity(
        disparity_from_depth(depth_ramp(0.04, 0.15)));
    cv::Mat pred_map = gt * 1.01;
    const auto pred  = cloud_from_map(pred_map);
    cv::Mat reason   = cv::Mat::zeros(kHeight, kWidth, CV_8U);

    const auto diag = score::diagnose_reconstruction(pred, gt,
                                                     make_inputs(reason));
    EXPECT_NEAR(diag.fit_depth_scale, 1.01, 0.001);
    EXPECT_LT(diag.fit_depth_scale_residual_m,
              diag.fit_disparity_offset_residual_m);
    EXPECT_LT(diag.fit_depth_scale_residual_m, 1e-5);
}

TEST(ReconstructionDiagnosticsTest, FitsIgnoreTheErrorTail) {
    // Exact reconstruction except a 25 % block that is 30 mm too far: the
    // fits (error < 20 mm only) still say "no offset, no scale".
    const cv::Mat gt = gt_from_disparity(
        disparity_from_depth(depth_ramp(0.04, 0.15)));
    cv::Mat pred_map = gt.clone();
    for (int r = 0; r < kHeight; ++r)
        for (int c = 0; c < kWidth / 4; ++c)
            pred_map.at<cv::Vec3f>(r, c)[2] += 0.03f;
    cv::Mat reason = cv::Mat::zeros(kHeight, kWidth, CV_8U);
    const auto diag = score::diagnose_reconstruction(
        cloud_from_map(pred_map), gt, make_inputs(reason));
    EXPECT_NEAR(diag.fit_disparity_offset_px, 0.0, 1e-4);
    EXPECT_NEAR(diag.fit_depth_scale, 1.0, 1e-6);
    EXPECT_NEAR(diag.tail_over_20mm, 0.25, 1e-9);
}

// ---------------------------------------------------------------------------
// Error tail
// ---------------------------------------------------------------------------

TEST(ReconstructionDiagnosticsTest, TailReflectsOutliers) {
    // 100 × 100 = 10 000 matched points; 10 of them are 60 mm off.
    const cv::Mat gt = gt_from_disparity(
        disparity_from_depth(depth_ramp(0.05, 0.10, 100, 100)));
    cv::Mat pred_map = gt.clone();
    for (int c = 0; c < 10; ++c)
        pred_map.at<cv::Vec3f>(50, c)[2] += 0.06f;
    cv::Mat reason = cv::Mat::zeros(100, 100, CV_8U);
    const auto diag = score::diagnose_reconstruction(
        cloud_from_map(pred_map), gt, make_inputs(reason));

    EXPECT_EQ(diag.matched_points, 10000u);
    EXPECT_NEAR(diag.tail_over_50mm, 0.001, 1e-12);
    EXPECT_NEAR(diag.tail_over_20mm, 0.001, 1e-12);
    EXPECT_NEAR(diag.tail_over_5mm, 0.001, 1e-12);
    EXPECT_NEAR(diag.max_3d_m, 0.06, 1e-6);
}

// ---------------------------------------------------------------------------
// Coverage-loss attribution (D4)
// ---------------------------------------------------------------------------

TEST(ReconstructionDiagnosticsTest, LossesPlusCoverageSumToOne) {
    // Depth 40–150 mm ⇒ d_gt ≈ 107 … 28.6 px, all inside [0, 160).
    cv::Mat gt = gt_from_disparity(
        disparity_from_depth(depth_ramp(0.04, 0.15)));
    auto pred      = cloud_from_map(gt);
    cv::Mat reason = cv::Mat::zeros(kHeight, kWidth, CV_8U);

    // Rows 0–9: GT unknown (NaN) but reconstructed ⇒ unscored.
    for (int r = 0; r < 10; ++r)
        for (int c = 0; c < kWidth; ++c)
            gt.at<cv::Vec3f>(r, c) = cv::Vec3f(kNaN, kNaN, kNaN);
    // Rows 20–29: matcher rejected; rows 30–39: depth range; rows 40–49:
    // outlier filter.  Columns 0–159 there also lie in SGBM's invalid left
    // band (numDisparities = 160), which wins over the trace reason.
    auto invalidate = [&](int r0, int r1, Reason why) {
        for (int r = r0; r < r1; ++r)
            for (int c = 0; c < kWidth; ++c) {
                set_invalid(pred, r, c);
                reason.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>(why);
            }
    };
    invalidate(20, 30, Reason::matcher_rejected);
    invalidate(30, 40, Reason::depth_range);
    invalidate(40, 50, Reason::outlier_filter);
    // Row 60: GT at 20 mm (d ≈ 214 px > 160) and rejected by the matcher ⇒
    // out_of_search_range, not matcher_rejected.
    const float d_near = static_cast<float>(disparity_of(0.02));
    for (int c = 0; c < kWidth; ++c) {
        gt.at<cv::Vec3f>(60, c) = cv::Vec3f(0.0f, 0.0f, 0.02f);
        set_invalid(pred, 60, c);
        reason.at<std::uint8_t>(60, c) =
            static_cast<std::uint8_t>(Reason::matcher_rejected);
    }
    EXPECT_GT(d_near, 160.0f);

    const auto diag = score::diagnose_reconstruction(pred, gt,
                                                     make_inputs(reason));
    const std::size_t gt_valid = static_cast<std::size_t>(kWidth) * (kHeight - 10);
    EXPECT_EQ(diag.ground_truth_points, gt_valid);
    EXPECT_EQ(diag.matched_points, gt_valid - 31u * kWidth);
    EXPECT_NEAR(loss_sum(diag), 1.0, 1e-9);

    const double n = static_cast<double>(gt_valid);
    EXPECT_NEAR(diag.coverage_loss.out_of_search_range, kWidth / n, 1e-12);
    // Border band: the first minDisparity + numDisparities = 160 columns of
    // the 30 traced rows (d_gt there, ~93 … ~66 px, is inside the search
    // range, so the band is what decides).  The remaining 160 columns per
    // row keep their trace reason.
    constexpr std::size_t kBand = 160;
    const std::size_t border    = 30u * kBand;
    const std::size_t matcher   = 10u * (kWidth - kBand);
    const std::size_t depth     = 10u * (kWidth - kBand);
    const std::size_t outlier   = 10u * (kWidth - kBand);
    EXPECT_NEAR(diag.coverage_loss.image_border, border / n, 1e-12);
    EXPECT_NEAR(diag.coverage_loss.matcher_rejected, matcher / n, 1e-12);
    EXPECT_NEAR(diag.coverage_loss.depth_range, depth / n, 1e-12);
    EXPECT_NEAR(diag.coverage_loss.outlier_filter, outlier / n, 1e-12);

    // Unscored: 10 rows of reconstructed points without GT out of all the
    // reconstructed points.
    const double reconstructed = static_cast<double>(kWidth) * (kHeight - 31);
    EXPECT_NEAR(diag.unscored_fraction, (10.0 * kWidth) / reconstructed, 1e-12);
}

TEST(ReconstructionDiagnosticsTest, SearchRangeCheckUsesTheAppliedOffset) {
    // GT disparity 160.05 px: raw = d_gt − δ.  With δ = 0 the pixel is out
    // of [0, 160); with δ = +0.1 the raw disparity 159.95 was searchable, so
    // the loss falls through to the trace reason.
    const double z = disparity_of(160.05); // ≈ 26.8 mm
    cv::Mat gt(1, 400, CV_32FC3, cv::Scalar(0.0f, 0.0f, static_cast<float>(z)));
    score::PointCloud pred;
    pred.width  = 400;
    pred.height = 1;
    pred.data.assign(400 * 3, kNaN);
    cv::Mat reason(1, 400, CV_8U,
                   cv::Scalar(static_cast<double>(Reason::matcher_rejected)));

    const auto without = score::diagnose_reconstruction(
        pred, gt, make_inputs(reason, 0.0));
    EXPECT_DOUBLE_EQ(without.coverage_loss.out_of_search_range, 1.0);
    EXPECT_DOUBLE_EQ(without.coverage_loss.matcher_rejected, 0.0);

    const auto with = score::diagnose_reconstruction(
        pred, gt, make_inputs(reason, 0.1));
    EXPECT_DOUBLE_EQ(with.coverage_loss.out_of_search_range, 0.0);
    // Columns 0–159 (numDisparities = 160) are SGBM's invalid left band, the
    // rest matcher_rejected.
    const double border = 160.0 / 400.0;
    EXPECT_NEAR(with.coverage_loss.image_border, border, 1e-12);
    EXPECT_NEAR(with.coverage_loss.matcher_rejected, 1.0 - border, 1e-12);
    EXPECT_NEAR(loss_sum(with), 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(with.unscored_fraction, 0.0);
    EXPECT_TRUE(std::isnan(with.bias_z_median_m));
    EXPECT_TRUE(with.depth_bias.empty());
}

TEST(ReconstructionDiagnosticsTest, BorderBandFollowsTheSearchRange) {
    // The band is what cv::StereoSGBM leaves invalid: the leftmost
    // minDisparity + numDisparities columns, independent of the scene.  A
    // 100 px GT disparity is searchable in every range below, so widening the
    // range moves loss from matcher_rejected into image_border one-for-one.
    const double z = disparity_of(100.0);
    cv::Mat gt(1, 400, CV_32FC3, cv::Scalar(0.0f, 0.0f, static_cast<float>(z)));
    score::PointCloud pred;
    pred.width  = 400;
    pred.height = 1;
    pred.data.assign(400 * 3, kNaN);
    cv::Mat reason(1, 400, CV_8U,
                   cv::Scalar(static_cast<double>(Reason::matcher_rejected)));

    struct Case {
        int min_disparity;
        int num_disparities;
        int band;
    };
    for (const Case c : {Case{0, 160, 160}, Case{0, 224, 224},
                         Case{16, 160, 176}}) {
        auto in            = make_inputs(reason);
        in.min_disparity   = c.min_disparity;
        in.num_disparities = c.num_disparities;
        const auto diag    = score::diagnose_reconstruction(pred, gt, in);
        const double band  = c.band / 400.0;
        EXPECT_DOUBLE_EQ(diag.coverage_loss.out_of_search_range, 0.0)
            << c.num_disparities;
        EXPECT_NEAR(diag.coverage_loss.image_border, band, 1e-12)
            << c.num_disparities;
        EXPECT_NEAR(diag.coverage_loss.matcher_rejected, 1.0 - band, 1e-12)
            << c.num_disparities;
        EXPECT_NEAR(loss_sum(diag), 1.0, 1e-9) << c.num_disparities;
    }
}

TEST(ReconstructionDiagnosticsTest, RejectsInconsistentInputs) {
    // 1 m depth ⇒ d_gt ≈ 4.3 px.  With numDisparities = 6 the band is
    // columns 0–5, so column 6 is outside it and the trace reason is
    // consulted.
    const cv::Mat gt = gt_from_disparity(
        disparity_from_depth(depth_ramp(1.0, 1.0, 4, 8)));
    auto pred = cloud_from_map(gt);
    set_invalid(pred, 1, 6);
    cv::Mat reason = cv::Mat::zeros(4, 8, CV_8U); // says (1,6) is valid
    auto narrow    = make_inputs(reason);
    narrow.num_disparities = 6;
    EXPECT_THROW((void)score::diagnose_reconstruction(pred, gt, narrow),
                 std::invalid_argument);

    cv::Mat wrong_size = cv::Mat::zeros(3, 8, CV_8U);
    EXPECT_THROW((void)score::diagnose_reconstruction(cloud_from_map(gt), gt,
                                                      make_inputs(wrong_size)),
                 std::invalid_argument);

    auto bad = make_inputs(cv::Mat::zeros(4, 8, CV_8U));
    bad.num_disparities = 0;
    EXPECT_THROW((void)score::diagnose_reconstruction(cloud_from_map(gt), gt,
                                                      bad),
                 std::invalid_argument);
}

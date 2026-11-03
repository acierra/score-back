#include <cmath>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tuple>

#include "cloud_point/cpu_stereo_matcher.hpp"
#include "cloud_point/gpu_stereo_matcher.hpp"
#include "cloud_point/stereo_matcher_factory.hpp"

using namespace score;

namespace {

/// @brief Create a simple synthetic stereo pair with a horizontal shift.
std::pair<cv::Mat, cv::Mat> make_synthetic_stereo(int shift = 2) {
    cv::Mat left  = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(100, 100, CV_8UC1);

    for (int y = 0; y < 100; ++y) {
        for (int x = 0; x < 100; ++x) {
            left.at<uchar>(y, x)  = static_cast<uchar>(x % 256);
            right.at<uchar>(y, x) = static_cast<uchar>((x + shift) % 256);
        }
    }
    return {left, right};
}

} // namespace

TEST(StereoMatcherTest, CpuMatcherComputesDisparity) {
    auto [left, right] = make_synthetic_stereo();
    CpuStereoMatcher matcher;

    cv::Mat disparity = matcher.compute(left, right);

    EXPECT_FALSE(disparity.empty());
    EXPECT_EQ(disparity.rows, left.rows);
    EXPECT_EQ(disparity.cols, left.cols);
}

TEST(StereoMatcherTest, FactoryCpuCreatesNonNull) {
    auto matcher = StereoMatcherFactory::create(StereoAlgorithmType::CPU);
    ASSERT_NE(matcher, nullptr);

    auto [left, right] = make_synthetic_stereo();
    cv::Mat disparity  = matcher->compute(left, right);

    EXPECT_FALSE(disparity.empty());
}

TEST(StereoMatcherTest, CpuMatcherRejectsEvenBlockSize) {
    CpuStereoMatcher::Params params;
    params.block_size = 4;
    EXPECT_THROW(CpuStereoMatcher(0, 64, params), std::invalid_argument);
}

TEST(StereoMatcherTest, CpuMatcherRejectsBadMedianKernel) {
    CpuStereoMatcher::Params params;
    params.median_kernel = 4;
    EXPECT_THROW(CpuStereoMatcher(0, 64, params), std::invalid_argument);
}

TEST(StereoMatcherTest, WlsKeepsSgbmHolesInvalid) {
    if (!CpuStereoMatcher::wls_available())
        GTEST_SKIP() << "OpenCV built without ximgproc";
    // Left 64 px of the right image have no counterpart -> SGBM leaves the
    // left border invalid; WLS must not fill it with extrapolated values.
    cv::Mat left(120, 200, CV_8UC1);
    cv::randu(left, 0, 255);
    cv::blur(left, left, cv::Size(3, 3));
    cv::Mat right = cv::Mat::zeros(left.size(), CV_8UC1);
    left(cv::Rect(8, 0, left.cols - 8, left.rows))
        .copyTo(right(cv::Rect(0, 0, left.cols - 8, left.rows)));

    CpuStereoMatcher::Params raw;
    raw.wls_filter = false;
    CpuStereoMatcher::Params wls;
    wls.wls_filter = true;
    cv::Mat d_raw  = CpuStereoMatcher(0, 64, raw).compute(left, right);
    cv::Mat d_wls  = CpuStereoMatcher(0, 64, wls).compute(left, right);
    ASSERT_EQ(d_wls.type(), CV_16S);
    ASSERT_EQ(d_wls.size(), d_raw.size());
    const int raw_invalid = cv::countNonZero(d_raw <= 0);
    EXPECT_GT(raw_invalid, 0);
    // No SGBM hole may be filled with an extrapolated disparity.
    const int filled = cv::countNonZero((d_raw <= 0) & (d_wls > 0));
    EXPECT_EQ(filled, 0);
    // The filter must still produce a valid map elsewhere.
    EXPECT_GT(cv::countNonZero(d_wls > 0), 0);
}

TEST(StereoMatcherTest, CpuMatcherRecoversKnownShift) {
    // Textured synthetic pair: right image is the left shifted by 8 px, so
    // the (16x fixed-point) disparity in the interior should be ~8 px.
    constexpr int kShift = 8;
    cv::Mat left(120, 200, CV_8UC1);
    cv::randu(left, 0, 255);
    cv::blur(left, left, cv::Size(3, 3));
    cv::Mat right = cv::Mat::zeros(left.size(), CV_8UC1);
    left(cv::Rect(kShift, 0, left.cols - kShift, left.rows))
        .copyTo(right(cv::Rect(0, 0, left.cols - kShift, left.rows)));

    CpuStereoMatcher matcher(0, 64);
    cv::Mat disparity = matcher.compute(left, right);
    ASSERT_EQ(disparity.type(), CV_16S);

    int good = 0, total = 0;
    for (int y = 10; y < left.rows - 10; ++y) {
        for (int x = 70; x < left.cols - 20; ++x) {
            const float d = disparity.at<short>(y, x) / 16.0f;
            if (d <= 0)
                continue;
            ++total;
            if (std::abs(d - kShift) <= 1.0f)
                ++good;
        }
    }
    ASSERT_GT(total, 0);
    EXPECT_GT(static_cast<double>(good) / total, 0.9);
}

TEST(StereoMatcherTest, GpuSupportedNumDisparitiesRoundsUp) {
    EXPECT_EQ(GpuStereoMatcher::supported_num_disparities(16), 64);
    EXPECT_EQ(GpuStereoMatcher::supported_num_disparities(64), 64);
    EXPECT_EQ(GpuStereoMatcher::supported_num_disparities(96), 128);
    EXPECT_EQ(GpuStereoMatcher::supported_num_disparities(160), 256);
    EXPECT_THROW(std::ignore = GpuStereoMatcher::supported_num_disparities(272),
                 std::invalid_argument);
}

TEST(StereoMatcherTest, FactoryRejectsInvalidNumDisparities) {
    EXPECT_THROW(std::ignore =
                     StereoMatcherFactory::create(StereoAlgorithmType::CPU, 0),
                 std::invalid_argument);
    EXPECT_THROW(std::ignore = StereoMatcherFactory::create(
                     StereoAlgorithmType::CPU, -16),
                 std::invalid_argument);
    EXPECT_THROW(std::ignore = StereoMatcherFactory::create(
                     StereoAlgorithmType::CPU, 150),
                 std::invalid_argument);
}

TEST(StereoMatcherTest, FactoryAcceptsValidNumDisparities) {
    auto matcher = StereoMatcherFactory::create(StereoAlgorithmType::CPU, 160);
    ASSERT_NE(matcher, nullptr);
}

TEST(StereoMatcherTest, FactoryGpuFallsBackToCpuWhenUnavailable) {
    // On this machine CUDA is absent; factory should fall back to CPU.
    auto matcher = StereoMatcherFactory::create(StereoAlgorithmType::GPU);
    ASSERT_NE(matcher, nullptr);

    auto [left, right] = make_synthetic_stereo();
    EXPECT_NO_THROW(std::ignore = matcher->compute(left, right));
}

TEST(StereoMatcherTest, GpuMatcherThrowsOnThisMachine) {
    // Direct construction of GpuStereoMatcher should throw because
    // HAVE_OPENCV_CUDA is undefined here.
    EXPECT_THROW(GpuStereoMatcher(), std::runtime_error);
}

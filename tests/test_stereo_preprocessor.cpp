#include "cloud_point/stereo_preprocessor.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <opencv2/imgproc.hpp>

using namespace score;

namespace {
PreprocessingConfig config_for(const std::string &method) {
    PreprocessingStep step;
    step.method = method;
    if (method == "pair_normalize")
        step.statistics = "left_reference";
    if (method == "clahe" || method == "local_normalize" ||
        method == "illumination_correction")
        step.statistics = "independent";
    return {true, {step}};
}

class AffineStage : public IStereoPreprocessor {
  public:
    AffineStage(double gain, double offset) : gain_(gain), offset_(offset) {}
    PreprocessedStereoPair process(const cv::Mat &l,
                                   const cv::Mat &r) override {
        PreprocessedStereoPair out;
        l.convertTo(out.left, CV_8U, gain_, offset_);
        r.convertTo(out.right, CV_8U, gain_, offset_);
        return out;
    }

  private:
    double gain_, offset_;
};
} // namespace

TEST(StereoPreprocessorTest, NoneIsExactIdentityWithoutCopy) {
    cv::Mat l(32, 40, CV_8U), r(32, 40, CV_8U);
    cv::RNG rng(123);
    rng.fill(l, cv::RNG::UNIFORM, 0, 256);
    rng.fill(r, cv::RNG::UNIFORM, 0, 256);
    for (auto config :
         {PreprocessingConfig{}, config_for("none"),
          PreprocessingConfig{false, config_for("clahe").chain}}) {
        StereoPreprocessorChain chain(config);
        const auto out = chain.process(l, r);
        EXPECT_EQ(out.left.data, l.data);
        EXPECT_EQ(out.right.data, r.data);
        EXPECT_EQ(cv::norm(out.left, l, cv::NORM_INF), 0);
        EXPECT_EQ(cv::norm(out.right, r, cv::NORM_INF), 0);
    }
}

TEST(StereoPreprocessorTest, EveryStageRunsInConfiguredOrderOnBothViews) {
    cv::Mat l(4, 5, CV_8U, cv::Scalar(10));
    cv::Mat r(4, 5, CV_8U, cv::Scalar(20));
    StereoPreprocessorChain chain;
    chain.append(std::make_unique<AffineStage>(1, 5));
    chain.append(std::make_unique<AffineStage>(2, 0));
    const auto out = chain.process(l, r);
    EXPECT_EQ(out.left.at<uchar>(0, 0), 30);
    EXPECT_EQ(out.right.at<uchar>(0, 0), 50);
    EXPECT_EQ(l.at<uchar>(0, 0), 10);
    EXPECT_EQ(r.at<uchar>(0, 0), 20);
    EXPECT_THROW(chain.append(nullptr), std::invalid_argument);
}

TEST(StereoPreprocessorTest,
     NormalizationUsesLeftReferenceAndHandlesFlatImages) {
    cv::Mat l = (cv::Mat_<uchar>(2, 4) << 50, 70, 90, 110, 130, 150, 170, 190);
    cv::Mat r;
    l.convertTo(r, CV_8U, 0.5, 10);
    StereoPreprocessorChain chain(config_for("pair_normalize"));
    const auto out = chain.process(l, r);
    EXPECT_EQ(cv::norm(out.left, l, cv::NORM_INF), 0);
    EXPECT_LE(cv::norm(out.right, l, cv::NORM_INF), 1);
    auto flat = chain.process(cv::Mat(8, 8, CV_8U, cv::Scalar(100)),
                              cv::Mat(8, 8, CV_8U, cv::Scalar(20)));
    EXPECT_EQ(cv::mean(flat.right)[0], 100);
}

TEST(StereoPreprocessorTest, BuiltinsPreserveInputsGeometryAndEqualViews) {
    cv::Mat storage(64, 80, CV_8U);
    cv::RNG rng(42);
    rng.fill(storage, cv::RNG::UNIFORM, 20, 90);
    const cv::Mat image = storage(cv::Rect(3, 4, 50, 48)); // non-contiguous ROI
    const auto original = image.clone();
    for (const auto *method :
         {"pair_normalize", "clahe", "local_normalize",
          "illumination_correction", "edge_preserving_denoise"}) {
        SCOPED_TRACE(method);
        StereoPreprocessorChain chain(config_for(method));
        const auto out = chain.process(image, image);
        EXPECT_EQ(out.left.size(), image.size());
        EXPECT_EQ(out.right.size(), image.size());
        EXPECT_EQ(out.left.type(), CV_8UC1);
        EXPECT_EQ(out.right.type(), CV_8UC1);
        EXPECT_EQ(cv::norm(image, original, cv::NORM_INF), 0);
        EXPECT_EQ(cv::norm(out.left, out.right, cv::NORM_INF), 0);
        EXPECT_NE(out.left.data, image.data);
        if (std::string(method) == "clahe")
            EXPECT_GT(cv::norm(out.left, image, cv::NORM_INF), 0);
        // Repeated frames are deterministic and don't retain image statistics.
        const auto repeated = chain.process(image, image);
        EXPECT_EQ(cv::norm(out.left, repeated.left, cv::NORM_INF), 0);
    }
}

TEST(StereoPreprocessorTest, DarkAndConstantImagesRemainFinite) {
    for (const auto *method : {"local_normalize", "illumination_correction"}) {
        StereoPreprocessorChain chain(config_for(method));
        for (int value : {0, 80, 255}) {
            cv::Mat image(20, 20, CV_8U, cv::Scalar(value));
            auto out = chain.process(image, image);
            EXPECT_EQ(cv::norm(out.left, image, cv::NORM_INF), 0);
        }
    }
}

TEST(StereoPreprocessorTest, RejectsInvalidInputsAndConfiguration) {
    StereoPreprocessorChain chain;
    const cv::Mat gray(8, 8, CV_8U);
    EXPECT_THROW(chain.process({}, gray), std::invalid_argument);
    EXPECT_THROW(chain.process(gray, cv::Mat(9, 8, CV_8U)),
                 std::invalid_argument);
    EXPECT_THROW(chain.process(gray, cv::Mat(8, 8, CV_8UC3)),
                 std::invalid_argument);
    EXPECT_THROW(chain.process(gray, cv::Mat(8, 8, CV_32F)),
                 std::invalid_argument);
    auto bad    = config_for("typo");
    bad.enabled = false;
    EXPECT_THROW(StereoPreprocessorChain{bad}, std::invalid_argument);
    bad = config_for("clahe");
    bad.chain[0].statistics.clear();
    EXPECT_THROW(StereoPreprocessorChain{bad}, std::invalid_argument);
    bad                      = config_for("local_normalize");
    bad.chain[0].window_size = 4;
    EXPECT_THROW(StereoPreprocessorChain{bad}, std::invalid_argument);
    bad                  = config_for("pair_normalize");
    bad.chain[0].epsilon = std::numeric_limits<double>::infinity();
    EXPECT_THROW(StereoPreprocessorChain{bad}, std::invalid_argument);
}

TEST(StereoPreprocessorTest, RejectsCustomStageChangingPixelGrid) {
    class BadStage : public IStereoPreprocessor {
        PreprocessedStereoPair process(const cv::Mat &,
                                       const cv::Mat &) override {
            return {cv::Mat(2, 2, CV_8U), cv::Mat(2, 2, CV_8U)};
        }
    };
    StereoPreprocessorChain chain;
    chain.append(std::make_unique<BadStage>());
    const cv::Mat image(8, 8, CV_8U);
    EXPECT_THROW(chain.process(image, image), std::runtime_error);
}

TEST(StereoPreprocessorTest, ConfiguredChainEqualsSequentialStages) {
    auto config = config_for("edge_preserving_denoise");
    config.chain.push_back(config_for("clahe").chain.front());
    cv::Mat l(48, 64, CV_8U), r(48, 64, CV_8U);
    cv::RNG rng(99);
    rng.fill(l, cv::RNG::UNIFORM, 0, 256);
    rng.fill(r, cv::RNG::UNIFORM, 0, 256);
    StereoPreprocessorChain chain(config);
    StereoPreprocessorChain first(config_for("edge_preserving_denoise"));
    StereoPreprocessorChain second(config_for("clahe"));
    const auto intermediate = first.process(l, r);
    const auto expected = second.process(intermediate.left, intermediate.right);
    const auto actual   = chain.process(l, r);
    EXPECT_EQ(cv::norm(actual.left, expected.left, cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(actual.right, expected.right, cv::NORM_INF), 0);
}

TEST(StereoPreprocessorTest, SpecularMaskKeepsPixelsAndSurvivesFollowingSteps) {
    auto config                     = config_for("specular_mask");
    config.chain[0].dilation_radius = 0;
    cv::Mat left(3, 10, CV_8U, cv::Scalar(100));
    cv::Mat right         = left.clone();
    left.at<uchar>(1, 8)  = 245; // threshold is inclusive
    right.at<uchar>(1, 3) = 255;
    StereoPreprocessorChain chain(config);
    const auto out = chain.process(left, right);
    EXPECT_EQ(out.left.data, left.data);
    EXPECT_EQ(out.right.data, right.data);
    EXPECT_EQ(out.left_valid_mask.at<uchar>(1, 8), 0);
    EXPECT_EQ(out.right_valid_mask.at<uchar>(1, 3), 0);
    EXPECT_EQ(out.right_valid_mask.at<uchar>(1, 4), 255);
    EXPECT_EQ(left.at<uchar>(1, 8), 245);
    chain.append(std::make_unique<AffineStage>(0, 0));
    const auto after = chain.process(left, right);
    EXPECT_EQ(after.left.at<uchar>(1, 8), 0);
    EXPECT_EQ(after.left_valid_mask.at<uchar>(1, 8), 0);
}

TEST(StereoPreprocessorTest, RejectsRightCorrespondenceRatherThanSameColumn) {
    cv::Mat image(3, 10, CV_8U, cv::Scalar(100));
    PreprocessedStereoPair pair{image, image,
                                cv::Mat(3, 10, CV_8U, cv::Scalar(255)),
                                cv::Mat(3, 10, CV_8U, cv::Scalar(255))};
    pair.left_valid_mask.at<uchar>(1, 8)  = 0;
    pair.right_valid_mask.at<uchar>(1, 3) = 0;
    const cv::Mat disparity(3, 10, CV_16S, cv::Scalar(32)); // 2 px
    cv::Mat rejected;
    const auto out = apply_preprocessing_masks(disparity, pair, &rejected);
    EXPECT_EQ(out.at<short>(1, 5), 0);        // right x = 5-2 = 3
    EXPECT_EQ(out.at<short>(1, 3), 32);       // right x = 1 is valid
    EXPECT_EQ(out.at<short>(1, 8), 0);        // left mask
    EXPECT_EQ(out.at<short>(1, 0), 0);        // outside right image
    EXPECT_EQ(disparity.at<short>(1, 5), 32); // input is read-only
    EXPECT_EQ(rejected.at<uchar>(1, 5), 255);
    EXPECT_EQ(rejected.at<uchar>(1, 3), 0);
    cv::Mat fractional(3, 10, CV_32F, cv::Scalar(1.5));
    EXPECT_EQ(apply_preprocessing_masks(fractional, pair).at<float>(1, 5), 0);
    pair.right_valid_mask.at<uchar>(1, 3) = 255;
    pair.right_valid_mask.at<uchar>(1, 4) = 0;
    EXPECT_EQ(apply_preprocessing_masks(fractional, pair).at<float>(1, 5), 0);
    pair.left_valid_mask.release();
    pair.right_valid_mask.release();
    EXPECT_EQ(apply_preprocessing_masks(disparity, pair).data, disparity.data);
}

TEST(StereoPreprocessorTest, SpecularMaskDilationAndRepeatedMasksAccumulate) {
    auto config = config_for("specular_mask");
    cv::Mat image(9, 9, CV_8U, cv::Scalar(20));
    image.at<uchar>(4, 4) = 255;
    config.chain.push_back(config.chain.front());
    StereoPreprocessorChain chain(config);
    const auto out = chain.process(image, image);
    EXPECT_EQ(out.left_valid_mask.at<uchar>(4, 4), 0);
    EXPECT_EQ(out.left_valid_mask.at<uchar>(4, 5), 0);
    EXPECT_EQ(out.left_valid_mask.at<uchar>(4, 6), 255);
    config.chain[0].brightness_threshold = 0;
    EXPECT_THROW(StereoPreprocessorChain{config}, std::invalid_argument);
    PreprocessedStereoPair bad{image, image, cv::Mat(2, 2, CV_8U), {}};
    EXPECT_THROW(apply_preprocessing_masks(cv::Mat(9, 9, CV_16S), bad),
                 std::invalid_argument);
}

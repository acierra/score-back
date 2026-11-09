#include "cloud_point/stereo_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <utility>

namespace score {
namespace {

void validate_pair(const cv::Mat &left, const cv::Mat &right) {
    if (left.empty() || right.empty() || left.dims != 2 || right.dims != 2 ||
        left.size() != right.size() || left.type() != CV_8UC1 ||
        right.type() != CV_8UC1)
        throw std::invalid_argument(
            "preprocessing: expected nonempty same-size CV_8UC1 stereo pair");
}

void validate_mask(const cv::Mat &mask, cv::Size size) {
    if (!mask.empty() &&
        (mask.dims != 2 || mask.type() != CV_8UC1 || mask.size() != size))
        throw std::invalid_argument("preprocessing: invalid validity mask");
}

cv::Mat merge_masks(const cv::Mat &a, const cv::Mat &b) {
    if (a.empty())
        return b;
    if (b.empty())
        return a;
    cv::Mat merged;
    cv::bitwise_and(a, b, merged);
    return merged;
}

class SpecularMaskStage final : public IStereoPreprocessor {
  public:
    explicit SpecularMaskStage(PreprocessingStep step)
        : step_(std::move(step)) {}
    PreprocessedStereoPair process(const cv::Mat &left,
                                   const cv::Mat &right) override {
        return {left, right, validity(left), validity(right)};
    }

  private:
    cv::Mat validity(const cv::Mat &image) const {
        cv::Mat invalid, valid;
        cv::compare(image, step_.brightness_threshold, invalid, cv::CMP_GE);
        if (step_.dilation_radius > 0) {
            const int size = step_.dilation_radius * 2 + 1;
            cv::dilate(
                invalid, invalid,
                cv::getStructuringElement(cv::MORPH_ELLIPSE, {size, size}));
        }
        cv::bitwise_not(invalid, valid);
        return valid;
    }
    PreprocessingStep step_;
};

/// Built-in photometric stage. All automatic per-camera statistics are
/// explicitly selected in PreprocessingStep::statistics.
class PhotometricStage final : public IStereoPreprocessor {
  public:
    explicit PhotometricStage(PreprocessingStep step)
        : step_(std::move(step)) {}

    PreprocessedStereoPair process(const cv::Mat &left,
                                   const cv::Mat &right) override {
        if (step_.method == "clahe" ||
            step_.method == "edge_preserving_denoise")
            return {transform(left, 0, 0), transform(right, 0, 0)};
        cv::Scalar lm, ls, rm, rs;
        cv::meanStdDev(left, lm, ls);
        cv::meanStdDev(right, rm, rs);
        if (step_.method == "pair_normalize") {
            // Flat images have no recoverable contrast: only shift the mean.
            const double gain = ls[0] < step_.epsilon || rs[0] < step_.epsilon
                                    ? 1.0
                                    : ls[0] / rs[0];
            cv::Mat out;
            right.convertTo(out, CV_8U, gain, lm[0] - gain * rm[0]);
            return {left.clone(), std::move(out)};
        }
        // A pooled target for local_normalize; local moments remain per view.
        const double mean     = (lm[0] + rm[0]) * 0.5;
        const double variance = (ls[0] * ls[0] + rs[0] * rs[0]) * 0.5 +
                                (lm[0] - rm[0]) * (lm[0] - rm[0]) * 0.25;
        return {transform(left, mean, std::sqrt(variance)),
                transform(right, mean, std::sqrt(variance))};
    }

  private:
    cv::Mat transform(const cv::Mat &image, double target_mean,
                      double target_stddev) const {
        cv::Mat out;
        if (step_.method == "clahe") {
            auto clahe = cv::createCLAHE(
                step_.clip_limit,
                cv::Size(step_.tile_grid_size, step_.tile_grid_size));
            clahe->apply(image, out);
        } else if (step_.method == "edge_preserving_denoise") {
            cv::bilateralFilter(image, out, step_.diameter, step_.sigma_color,
                                step_.sigma_space);
        } else {
            cv::Mat values, mean;
            image.convertTo(values, CV_32F);
            const cv::Size window(step_.window_size, step_.window_size);
            cv::blur(values, mean, window, cv::Point(-1, -1),
                     cv::BORDER_REFLECT_101);
            cv::Mat result;
            if (step_.method == "local_normalize") {
                cv::Mat square_mean, variance, stddev;
                cv::blur(values.mul(values), square_mean, window,
                         cv::Point(-1, -1), cv::BORDER_REFLECT_101);
                cv::max(square_mean - mean.mul(mean), 0.0, variance);
                cv::sqrt(variance, stddev);
                cv::max(stddev, step_.epsilon, stddev);
                cv::divide(values - mean, stddev, result);
                result = result * target_stddev + target_mean;
            } else {
                // Simple homomorphic-like division by the local illumination;
                // not a Retinex implementation. Shared pair target mean.
                cv::Mat denominator;
                cv::max(mean, step_.epsilon, denominator);
                cv::divide(values, denominator, result);
                result *= target_mean;
            }
            result.convertTo(out, CV_8U); // saturating conversion to [0, 255]
        }
        return out;
    }

    PreprocessingStep step_;
};

} // namespace

StereoPreprocessorChain::StereoPreprocessorChain(
    const PreprocessingConfig &config) {
    validate_preprocessing(config);
    if (config.enabled) {
        for (const auto &step : config.chain) {
            if (step.method == "specular_mask")
                append(std::make_unique<SpecularMaskStage>(step));
            else if (step.method != "none")
                append(std::make_unique<PhotometricStage>(step));
        }
    }
}

void StereoPreprocessorChain::append(
    std::unique_ptr<IStereoPreprocessor> stage) {
    if (!stage)
        throw std::invalid_argument("preprocessing: cannot append null stage");
    stages_.push_back(std::move(stage));
}

PreprocessedStereoPair StereoPreprocessorChain::process(const cv::Mat &left,
                                                        const cv::Mat &right) {
    validate_pair(left, right);
    PreprocessedStereoPair result{left, right};
    for (const auto &stage : stages_) {
        auto next = stage->process(result.left, result.right);
        if (next.left.empty() || next.right.empty() || next.left.dims != 2 ||
            next.right.dims != 2 || next.left.type() != CV_8UC1 ||
            next.right.type() != CV_8UC1 || next.left.size() != left.size() ||
            next.right.size() != right.size())
            throw std::runtime_error(
                "preprocessing: stage changed the stereo pixel grid or type");
        validate_mask(next.left_valid_mask, left.size());
        validate_mask(next.right_valid_mask, right.size());
        next.left_valid_mask =
            merge_masks(result.left_valid_mask, next.left_valid_mask);
        next.right_valid_mask =
            merge_masks(result.right_valid_mask, next.right_valid_mask);
        result = std::move(next);
    }
    return result;
}

cv::Mat apply_preprocessing_masks(const cv::Mat &disparity,
                                  const PreprocessedStereoPair &pair,
                                  cv::Mat *rejected) {
    validate_pair(pair.left, pair.right);
    if (disparity.empty() || disparity.dims != 2 ||
        disparity.size() != pair.left.size() ||
        (disparity.type() != CV_16S && disparity.type() != CV_32F))
        throw std::invalid_argument(
            "preprocessing: disparity must be same-size CV_16S or CV_32F");
    validate_mask(pair.left_valid_mask, disparity.size());
    validate_mask(pair.right_valid_mask, disparity.size());
    if (rejected)
        rejected->release();
    if (pair.left_valid_mask.empty() && pair.right_valid_mask.empty())
        return disparity;
    cv::Mat out = disparity.clone();
    if (rejected)
        *rejected = cv::Mat::zeros(disparity.size(), CV_8U);
    for (int row = 0; row < out.rows; ++row) {
        for (int col = 0; col < out.cols; ++col) {
            const double d = disparity.type() == CV_16S
                                 ? disparity.at<short>(row, col) / 16.0
                                 : disparity.at<float>(row, col);
            if (!(d > 0.0) || !std::isfinite(d))
                continue;
            bool invalid = !pair.left_valid_mask.empty() &&
                           pair.left_valid_mask.at<uchar>(row, col) == 0;
            if (!invalid && !pair.right_valid_mask.empty()) {
                const double x = col - d;
                invalid        = !(x >= 0 && x <= out.cols - 1);
                if (!invalid) {
                    const int lo = static_cast<int>(std::floor(x));
                    const int hi = static_cast<int>(std::ceil(x));
                    invalid = pair.right_valid_mask.at<uchar>(row, lo) == 0 ||
                              pair.right_valid_mask.at<uchar>(row, hi) == 0;
                }
            }
            if (invalid) {
                if (out.type() == CV_16S)
                    out.at<short>(row, col) = 0;
                else
                    out.at<float>(row, col) = 0;
                if (rejected)
                    rejected->at<uchar>(row, col) = 255;
            }
        }
    }
    return out;
}

} // namespace score

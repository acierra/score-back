#pragma once

#include "cloud_point_rpc/preprocessing_config.hpp"
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace score {

/// @brief A rectified CV_8UC1 pair on the unchanged calibration pixel grid.
struct PreprocessedStereoPair {
    cv::Mat left;
    cv::Mat right;
    cv::Mat left_valid_mask{};  ///< Empty = all valid, otherwise CV_8U 0/255
    cv::Mat right_valid_mask{}; ///< Same rectified grid as the images
};

/// @brief Extension point for photometric processing of an entire stereo pair.
/// Implementations must preserve size/type and must not modify input buffers.
/// Photometric stages own their outputs. Identity / mask-only results may alias
/// input; callers must treat them as read-only. Instances are not thread-safe.
class IStereoPreprocessor {
  public:
    virtual ~IStereoPreprocessor() = default;
    [[nodiscard]] virtual PreprocessedStereoPair
    process(const cv::Mat &left, const cv::Mat &right) = 0;
};

/// @brief Ordered chain: every configured stage processes both images.
/// Empty chain / none returns unchanged Mat headers without image copies.
/// Not thread-safe; use one instance per pipeline (as for IStereoMatcher).
class StereoPreprocessorChain final : public IStereoPreprocessor {
  public:
    explicit StereoPreprocessorChain(const PreprocessingConfig &config = {});
    /// @brief Append a custom stage; rejects nullptr.
    void append(std::unique_ptr<IStereoPreprocessor> stage);
    /// @throws std::invalid_argument for empty, mismatched or non-gray inputs.
    /// @throws std::runtime_error if a custom stage changes size or type.
    [[nodiscard]] PreprocessedStereoPair process(const cv::Mat &left,
                                                 const cv::Mat &right) override;

  private:
    std::vector<std::unique_ptr<IStereoPreprocessor>> stages_;
};

/// @brief Reject matches whose left pixel or right correspondence is masked.
/// Right x = left x - d (d/16 for CV_16S, pixel units for CV_32F).
/// Both floor/ceil samples must be valid for a subpixel correspondence.
/// No masks: returns input unchanged. Otherwise returns a copy with rejected
/// disparity = 0. Input images, disparity and masks are never modified.
/// @param rejected Optional CV_8U map, 255 only for newly rejected matches;
///                 empty when there are no masks.
[[nodiscard]] cv::Mat
apply_preprocessing_masks(const cv::Mat &disparity,
                          const PreprocessedStereoPair &pair,
                          cv::Mat *rejected = nullptr);

} // namespace score

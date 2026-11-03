//
// Created by vptyp on 11.03.2026.
//
#pragma once
#include <opencv4/opencv2/opencv.hpp>
namespace score {

class Image {
  public:
    Image();
    explicit Image(const cv::Mat &image);
    ~Image();

    /// @note data_ could be changed through this
    [[nodiscard]] cv::Mat get();

  protected:
    cv::Mat data_;
};

} // namespace score
//
// Created by vptyp on 12.03.2026.
//
#include "cloud_point/image.h"

namespace score {

Image::Image() {
    // no work
}

Image::Image(const cv::Mat &image) { this->data_ = image; }

Image::~Image() = default;

cv::Mat Image::get() { return this->data_; }

} // namespace score
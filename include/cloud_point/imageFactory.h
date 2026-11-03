//
// Created by vptyp on 12.03.2026.
//

#pragma once
#include <cloud_point/image.h>
#include <cloud_point_rpc/imageRpc.h>
namespace score {

class ImageFactory {
  public:
    /**
     * @brief tries to decode available RPC image type to opencv compliant
     * @return opencv compliant image type
     * @throw runtime_error if type is unknown
     */
    static int pixelType(const ImageRPC::Type &type) {
        switch (type) {
        case ImageRPC::Type::BGR:
            return CV_8UC3;
        case ImageRPC::Type::RGBA:
            return CV_8UC4;
        case ImageRPC::Type::DEPTH:
            return CV_64FC1;
        default:
            throw std::runtime_error("Unknown image type");
        }
    }
    /**
     * @brief tries to create Image object from ImageRPC
     * @throw runtime_error if type is unknown or data size does not match
     * dimensions
     */
    static Image create(const ImageRPC &image) {
        const int cv_type     = pixelType(image.type);
        const size_t expected = static_cast<size_t>(image.width) *
                                static_cast<size_t>(image.height) *
                                CV_ELEM_SIZE(cv_type);
        if (image.data.size() != expected)
            throw std::runtime_error("Image data size does not match "
                                     "width*height*channels");
        cv::Mat imageMat(image.height, image.width, cv_type,
                         const_cast<unsigned char *>(image.data.data()));
        return Image{imageMat.clone()};
    }
};
} // namespace score
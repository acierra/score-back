#pragma once

#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include <stdexcept>
namespace score {

class CameraMatrixFactory {
  public:
    /**
     * @param rpc < vector of size Width*Height
     * @throw runtime_error if size is not Width*Height
     */
    template <size_t Width, size_t Height>
    static cv::Mat create(const std::vector<double> &rpc) {
        if (rpc.size() != Width * Height)
            throw std::runtime_error("Vector size is not Width*Height");
        return cv::Mat(Width, Height, CV_64F, const_cast<double *>(rpc.data()))
            .clone();
    }
};

} // namespace score
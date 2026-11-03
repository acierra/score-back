#include "cloud_point/gpu_stereo_matcher.hpp"
#include <stdexcept>
#include <string>

#ifdef HAVE_OPENCV_CUDA
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudastereo.hpp>
#endif

namespace score {

int GpuStereoMatcher::supported_num_disparities(int num_disparities) {
    if (num_disparities <= 64)
        return 64;
    if (num_disparities <= 128)
        return 128;
    if (num_disparities <= 256)
        return 256;
    throw std::invalid_argument(
        "GpuStereoMatcher: cv::cuda::StereoSGM supports at most 256 "
        "disparities, got " +
        std::to_string(num_disparities));
}

GpuStereoMatcher::GpuStereoMatcher(int min_disparity, int num_disparities,
                                   int uniqueness_ratio) {
    const int levels = supported_num_disparities(num_disparities);
#ifdef HAVE_OPENCV_CUDA
    if (cv::cuda::getCudaEnabledDeviceCount() == 0) {
        throw std::runtime_error("No CUDA devices available");
    }
    // P1/P2 follow the cv::cuda::StereoSGM defaults (10/120), which are
    // expressed on the census-transform cost scale rather than SAD.
    sgm_ = cv::cuda::createStereoSGM(min_disparity, levels, /*P1=*/10,
                                     /*P2=*/120, uniqueness_ratio,
                                     cv::cuda::StereoSGM::MODE_HH4);
#else
    (void)min_disparity;
    (void)levels;
    (void)uniqueness_ratio;
    throw std::runtime_error("OpenCV CUDA modules not available in this build");
#endif
}

cv::Mat GpuStereoMatcher::compute(const cv::Mat &left, const cv::Mat &right) {
#ifdef HAVE_OPENCV_CUDA
    cv::cuda::GpuMat d_left(left);
    cv::cuda::GpuMat d_right(right);
    cv::cuda::GpuMat d_disparity;
    sgm_->compute(d_left, d_right, d_disparity);
    cv::Mat disparity;
    d_disparity.download(disparity);
    return disparity;
#else
    (void)left;
    (void)right;
    throw std::runtime_error("OpenCV CUDA modules not available in this build");
#endif
}

} // namespace score

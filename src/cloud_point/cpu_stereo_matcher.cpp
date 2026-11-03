#include "cloud_point/cpu_stereo_matcher.hpp"

#include <algorithm>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

#ifdef CLOUD_POINT_HAVE_OPENCV_XIMGPROC
#include <opencv2/ximgproc/disparity_filter.hpp>
#endif

namespace score {

bool CpuStereoMatcher::wls_available() noexcept {
#ifdef CLOUD_POINT_HAVE_OPENCV_XIMGPROC
    return true;
#else
    return false;
#endif
}

CpuStereoMatcher::CpuStereoMatcher(int min_disparity, int num_disparities,
                                   Params params)
    : median_kernel_(params.median_kernel), wls_lambda_(params.wls_lambda),
      wls_sigma_(params.wls_sigma) {
    if (params.median_kernel != 0 && params.median_kernel != 3 &&
        params.median_kernel != 5) {
        throw std::invalid_argument(
            "CpuStereoMatcher: median_kernel must be 0, 3 or 5, got " +
            std::to_string(params.median_kernel));
    }
    if (params.block_size < 1 || params.block_size % 2 == 0) {
        throw std::invalid_argument(
            "CpuStereoMatcher: block_size must be a positive odd number, got " +
            std::to_string(params.block_size));
    }

    const int bs2 = params.block_size * params.block_size;
    // OpenCV's recommended penalties for a single-channel input.
    const int p1 = 8 * bs2;
    const int p2 = 32 * bs2;

    sgbm_        = cv::StereoSGBM::create(
        min_disparity, num_disparities, params.block_size, p1, p2,
        params.disp12_max_diff, params.pre_filter_cap, params.uniqueness_ratio,
        params.speckle_window_size, params.speckle_range,
        cv::StereoSGBM::MODE_SGBM);

    if (params.wls_filter) {
#ifdef CLOUD_POINT_HAVE_OPENCV_XIMGPROC
        // NB: createDisparityWLSFilter(sgbm_) would silently disable the
        // left matcher's uniqueness/speckle/LRC post-filters to obtain a
        // dense map for its own confidence estimate.  The generic factory
        // leaves our tuned matcher alone; we then keep SGBM's holes invalid.
        right_matcher_ = cv::ximgproc::createRightMatcher(sgbm_);
        auto wls       = cv::ximgproc::createDisparityWLSFilterGeneric(
            /*use_confidence=*/true);
        wls->setLambda(params.wls_lambda);
        wls->setSigmaColor(params.wls_sigma);
        wls->setLRCthresh(24); // 1.5 px in SGBM's 16x fixed point
        wls->setDepthDiscontinuityRadius(
            std::max(1, params.block_size / 2 + 1));
        wls_ = wls;
#else
        LOG(WARNING) << "CpuStereoMatcher: WLS filter requested but OpenCV "
                        "was built without ximgproc; disparity is unfiltered";
#endif
    }
}

cv::Mat CpuStereoMatcher::compute(const cv::Mat &left, const cv::Mat &right) {
    cv::Mat disparity;
    sgbm_->compute(left, right, disparity);

#ifdef CLOUD_POINT_HAVE_OPENCV_XIMGPROC
    // The WLS filter needs a valid ROI to the right of the disparity search
    // range; skip it (raw SGBM output) for images narrower than that.
    const int roi_width = sgbm_->getMinDisparity() +
                          sgbm_->getNumDisparities() + sgbm_->getBlockSize();
    if (wls_ && left.cols > roi_width) {
        // Snapshot SGBM's holes first: the filter may modify its inputs.
        const cv::Mat holes = disparity <= 0;
        cv::Mat right_disparity, filtered;
        right_matcher_->compute(right, left, right_disparity);
        auto wls = wls_.dynamicCast<cv::ximgproc::DisparityWLSFilter>();
        wls->filter(disparity, left, filtered, right_disparity);
        // WLS extrapolates into pixels SGBM rejected; keep those invalid so
        // holes stay holes and coverage is not inflated with guesses.
        filtered.setTo(cv::Scalar(sgbm_->getMinDisparity() * 16 - 16), holes);
        disparity = filtered;
    }
#endif

    if (median_kernel_ > 0) {
        cv::Mat filtered;
        cv::medianBlur(disparity, filtered, median_kernel_);
        disparity = filtered;
    }
    return disparity;
}

} // namespace score

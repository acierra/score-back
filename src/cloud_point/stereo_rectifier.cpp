#include "cloud_point/stereo_rectifier.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "cloud_point/matrixFactory.h"

namespace score {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

void require_mat(const cv::Mat &mat, int rows, int cols, int type,
                 const char *name) {
    if (mat.rows != rows || mat.cols != cols || mat.type() != type) {
        throw std::invalid_argument(
            std::string(name) + " must be " + std::to_string(rows) + "x" +
            std::to_string(cols) + " CV_64F (type " + std::to_string(CV_64F) +
            "), got " + std::to_string(mat.rows) + "x" +
            std::to_string(mat.cols) + " type=" + std::to_string(mat.type()));
    }
}

} // namespace

// ---------------------------------------------------------------------------
// StereoRectifier::Calibration::from_rpc
// ---------------------------------------------------------------------------

StereoRectifier::Calibration
StereoRectifier::Calibration::from_rpc(const StereoCalibrationRPC &rpc) {
    Calibration calib;
    calib.k_left  = CameraMatrixFactory::create<3, 3>(rpc.left.camera_matrix);
    calib.d_left  = CameraMatrixFactory::create<1, 5>(rpc.left.dist_coeffs);
    calib.k_right = CameraMatrixFactory::create<3, 3>(rpc.right.camera_matrix);
    calib.d_right = CameraMatrixFactory::create<1, 5>(rpc.right.dist_coeffs);
    calib.r       = CameraMatrixFactory::create<3, 3>(rpc.rotation);
    calib.t       = CameraMatrixFactory::create<3, 1>(rpc.translation);
    calib.image_size = cv::Size(rpc.width, rpc.height);
    return calib;
}

// ---------------------------------------------------------------------------
// StereoRectifier constructor
// ---------------------------------------------------------------------------

StereoRectifier::StereoRectifier(const Calibration &calib) {
    require_mat(calib.k_left, 3, 3, CV_64F, "k_left");
    require_mat(calib.k_right, 3, 3, CV_64F, "k_right");
    require_mat(calib.d_left, 1, 5, CV_64F, "d_left");
    require_mat(calib.d_right, 1, 5, CV_64F, "d_right");
    require_mat(calib.r, 3, 3, CV_64F, "r");
    require_mat(calib.t, 3, 1, CV_64F, "t");

    cv::Mat R2, P1, P2;
    cv::stereoRectify(calib.k_left, calib.d_left, calib.k_right, calib.d_right,
                      calib.image_size, calib.r, calib.t, r1_, R2, P1, P2, q_,
                      cv::CALIB_ZERO_DISPARITY, /*alpha=*/0);

    cv::initUndistortRectifyMap(calib.k_left, calib.d_left, r1_, P1,
                                calib.image_size, CV_16SC2, map_lx_, map_ly_);
    cv::initUndistortRectifyMap(calib.k_right, calib.d_right, R2, P2,
                                calib.image_size, CV_16SC2, map_rx_, map_ry_);
}

// ---------------------------------------------------------------------------
// StereoRectifier::rectify
// ---------------------------------------------------------------------------

std::pair<cv::Mat, cv::Mat>
StereoRectifier::rectify(const cv::Mat &left, const cv::Mat &right) const {
    cv::Mat rect_left, rect_right;
    cv::remap(left, rect_left, map_lx_, map_ly_, cv::INTER_LINEAR);
    cv::remap(right, rect_right, map_rx_, map_ry_, cv::INTER_LINEAR);
    return {rect_left, rect_right};
}

// ---------------------------------------------------------------------------
// StereoRectifier::rectify_left_point_map
// ---------------------------------------------------------------------------

cv::Mat
StereoRectifier::rectify_left_point_map(const cv::Mat &point_map) const {
    if (point_map.type() != CV_32FC3) {
        throw std::invalid_argument("point_map must be CV_32FC3, got type=" +
                                    std::to_string(point_map.type()));
    }
    if (point_map.size() != map_lx_.size()) {
        throw std::invalid_argument(
            "point_map dimensions must match the calibration image size");
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    cv::Mat rectified;
    cv::remap(point_map, rectified, map_lx_, map_ly_, cv::INTER_NEAREST,
              cv::BORDER_CONSTANT, cv::Scalar(nan, nan, nan));

    cv::Mat rotation;
    r1_.convertTo(rotation, CV_32F);
    for (int row = 0; row < rectified.rows; ++row) {
        for (int column = 0; column < rectified.cols; ++column) {
            auto &point = rectified.at<cv::Vec3f>(row, column);
            if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
                !std::isfinite(point[2])) {
                point = cv::Vec3f(nan, nan, nan);
                continue;
            }

            point = cv::Vec3f(rotation.at<float>(0, 0) * point[0] +
                                  rotation.at<float>(0, 1) * point[1] +
                                  rotation.at<float>(0, 2) * point[2],
                              rotation.at<float>(1, 0) * point[0] +
                                  rotation.at<float>(1, 1) * point[1] +
                                  rotation.at<float>(1, 2) * point[2],
                              rotation.at<float>(2, 0) * point[0] +
                                  rotation.at<float>(2, 1) * point[1] +
                                  rotation.at<float>(2, 2) * point[2]);
        }
    }
    return rectified;
}

// ---------------------------------------------------------------------------
// StereoRectifier::q
// ---------------------------------------------------------------------------

const cv::Mat &StereoRectifier::q() const noexcept { return q_; }

} // namespace score

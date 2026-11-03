#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "cloud_point_rpc/rpc_dto.hpp"

namespace score {

/// @brief Stereo rectifier that computes rectification maps from calibration
///        data and applies them to image pairs.
///
/// Thread safety: const methods (rectify, rectify_left_point_map, q) are safe
/// to call concurrently. The object must not be modified after construction.
class StereoRectifier {
  public:
    /// @brief Calibration parameters for a stereo rig.
    struct Calibration {
        cv::Mat k_left;  ///< 3x3 CV_64F intrinsic matrix, left camera
        cv::Mat d_left;  ///< 1x5 CV_64F distortion coefficients, left camera
        cv::Mat k_right; ///< 3x3 CV_64F intrinsic matrix, right camera
        cv::Mat d_right; ///< 1x5 CV_64F distortion coefficients, right camera
        cv::Mat r;       ///< 3x3 CV_64F rotation from left to right camera
        cv::Mat
            t; ///< 3x1 CV_64F translation from left to right camera (metres)
        cv::Size image_size;

        /// @brief Build calibration from the wire DTO using
        /// CameraMatrixFactory.
        /// @param rpc  Stereo calibration received over JSON-RPC.
        /// @return     Populated Calibration struct.
        static Calibration from_rpc(const StereoCalibrationRPC &rpc);
    };

    /// @brief Construct rectifier from calibration data.
    ///
    /// Calls cv::stereoRectify (CALIB_ZERO_DISPARITY, alpha=0) and
    /// cv::initUndistortRectifyMap (CV_16SC2) for both sides.
    ///
    /// @throws std::invalid_argument if any Mat has the wrong size or type.
    explicit StereoRectifier(const Calibration &calib);

    /// @brief Apply rectification maps to a stereo pair.
    /// @param left   Left input image (any type accepted by cv::remap).
    /// @param right  Right input image.
    /// @return       {rectified_left, rectified_right}.
    [[nodiscard]] std::pair<cv::Mat, cv::Mat>
    rectify(const cv::Mat &left, const cv::Mat &right) const;

    /// @brief Rectify an XYZ point map from the original left-camera frame.
    ///
    /// The map is resampled with nearest-neighbour interpolation to avoid
    /// blending geometry or NaN values, then finite points are rotated into
    /// the rectified-left coordinate frame.
    /// @param point_map CV_32FC3 map with the calibration image dimensions.
    /// @return CV_32FC3 point map in the rectified-left frame.
    /// @throws std::invalid_argument for an invalid type or dimensions.
    [[nodiscard]] cv::Mat
    rectify_left_point_map(const cv::Mat &point_map) const;

    /// @brief Access the 4x4 reprojection matrix Q produced by stereoRectify.
    [[nodiscard]] const cv::Mat &q() const noexcept;

  private:
    cv::Mat map_lx_, map_ly_; ///< Rectification maps for the left image
    cv::Mat map_rx_, map_ry_; ///< Rectification maps for the right image
    cv::Mat r1_;              ///< Rotation into the rectified-left frame
    cv::Mat q_;               ///< 4x4 CV_64F reprojection matrix
};

} // namespace score

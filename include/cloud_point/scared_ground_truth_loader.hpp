#pragma once

#include <cstddef>
#include <string>

#include <opencv2/core.hpp>

namespace score {

/// @brief Loads the semi-dense XYZ point map supplied with a SCARED keyframe.
///
/// The OBJ vertex order is preserved: vertex r * width + c belongs to pixel
/// (r, c). Unknown vertices are represented as NaN in all three coordinates.
/// The returned points are in the original left-camera coordinate frame.
class ScaredGroundTruthLoader {
  public:
    /// @brief Load <keyframe_dir>/point_cloud.obj.
    /// @param keyframe_dir Directory containing the SCARED keyframe files.
    /// @param image_size Expected point-map dimensions.
    /// @param units_to_metres Scale applied to finite OBJ coordinates. SCARED
    ///        ground truth is normally expressed in millimetres.
    /// @throws std::invalid_argument for invalid dimensions or scale.
    /// @throws std::runtime_error if the OBJ cannot be read or does not contain
    ///         exactly image_size.area() vertex records.
    explicit ScaredGroundTruthLoader(const std::string &keyframe_dir,
                                     cv::Size image_size,
                                     float units_to_metres = 0.001f);

    /// @brief Ground-truth XYZ point map, CV_32FC3 in metres.
    [[nodiscard]] const cv::Mat &point_map() const noexcept;

    /// @brief Number of pixels with finite XYZ ground truth.
    [[nodiscard]] std::size_t valid_point_count() const noexcept;

  private:
    cv::Mat point_map_;
    std::size_t valid_point_count_{0};
};

} // namespace score

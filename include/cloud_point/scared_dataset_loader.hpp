#pragma once

#include "cloud_point_rpc/rpc_dto.hpp"
#include <string>

namespace score {

/// @brief Loads stereo calibration and image pair from a SCARED dataset
///        keyframe directory.
///
/// Expected directory layout:
///   <keyframe_dir>/endoscope_calibration.yaml  — OpenCV FileStorage
///   <keyframe_dir>/Left_Image.png              — 1280x1024 RGBA PNG
///   <keyframe_dir>/Right_Image.png             — 1280x1024 RGBA PNG
///
/// The YAML node T is in millimetres; this loader converts to metres before
/// populating StereoCalibrationRPC.translation.
class ScaredDatasetLoader {
  public:
    /// @brief Load calibration and images from @p keyframe_dir.
    /// @throws std::runtime_error if any file cannot be opened or parsed,
    ///         or if the left and right images have different dimensions.
    explicit ScaredDatasetLoader(const std::string &keyframe_dir);

    /// @brief Return the stereo calibration DTO (translation in metres).
    [[nodiscard]] const StereoCalibrationRPC &calibration() const noexcept;

    /// @brief Return an image pair DTO with the given frame index.
    ///        The images are the same for every call (single keyframe).
    [[nodiscard]] ImagePairRPC image_pair(uint64_t frame) const;

  private:
    StereoCalibrationRPC calib_;
    ImageRPC left_image_;
    ImageRPC right_image_;
};

} // namespace score

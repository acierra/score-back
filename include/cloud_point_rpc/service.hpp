#pragma once

#include "cloud_point_rpc/config.hpp"
#include "cloud_point_rpc/rpc_dto.hpp"
#include "export.h"
#include <cstdint>
#include <vector>

namespace score {

class CRPC_EXPORT Service {
  public:
    explicit Service(const TestData &data = {});

    [[nodiscard]] std::vector<double> get_intrinsic_params() const;
    [[nodiscard]] std::vector<double> get_extrinsic_params() const;
    [[nodiscard]] std::vector<std::vector<double>> get_cloud_point() const;

    [[nodiscard]] StereoCalibrationRPC get_stereo_calibration() const;
    [[nodiscard]] ImagePairRPC get_image_pair();

  private:
    TestData data_;
    uint64_t frame_counter_{0};
};

} // namespace score

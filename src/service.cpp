#include "cloud_point_rpc/service.hpp"

namespace score {

Service::Service(const TestData &data) : data_(data), frame_counter_(0) {}

std::vector<double> Service::get_intrinsic_params() const {
    if (data_.intrinsic_params.empty()) {
        // Fallback if no config loaded
        return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    }
    return data_.intrinsic_params;
}

std::vector<double> Service::get_extrinsic_params() const {
    if (data_.extrinsic_params.empty()) {
        return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    }
    return data_.extrinsic_params;
}

std::vector<std::vector<double>> Service::get_cloud_point() const {
    if (data_.cloud_point.empty()) {
        return {{0.1, 0.2, 0.3}, {1.1, 1.2, 1.3}, {2.1, 2.2, 2.3}};
    }
    return data_.cloud_point;
}

StereoCalibrationRPC Service::get_stereo_calibration() const {
    StereoCalibrationRPC calib;
    calib.width  = 640;
    calib.height = 480;

    CameraCalib cam;
    // fx=fy=800, cx=320, cy=240 — identity rotation-scale, principal centre
    cam.camera_matrix = {800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0};
    cam.dist_coeffs   = {0.0, 0.0, 0.0, 0.0, 0.0};

    calib.left        = cam;
    calib.right       = cam;

    // Parallel rig: R = identity
    calib.rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    // T = [-baseline, 0, 0], baseline = 0.06 m
    calib.translation = {-0.06, 0.0, 0.0};

    return calib;
}

ImagePairRPC Service::get_image_pair() {
    constexpr int kWidth  = 640;
    constexpr int kHeight = 480;
    constexpr int kShift  = 8;

    ImagePairRPC pair;
    pair.frame = frame_counter_++;

    // Left image: BGR, each pixel value = (x + y) % 256 for all channels
    pair.left.width  = kWidth;
    pair.left.height = kHeight;
    pair.left.type   = ImageRPC::Type::BGR;
    pair.left.data.resize(static_cast<size_t>(kWidth) * kHeight * 3);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const auto val    = static_cast<unsigned char>((x + y) % 256);
            const size_t base = static_cast<size_t>(y * kWidth + x) * 3;
            pair.left.data[base + 0] = val;
            pair.left.data[base + 1] = val;
            pair.left.data[base + 2] = val;
        }
    }

    // Right image: same pattern shifted 8 px horizontally
    pair.right.width  = kWidth;
    pair.right.height = kHeight;
    pair.right.type   = ImageRPC::Type::BGR;
    pair.right.data.resize(static_cast<size_t>(kWidth) * kHeight * 3);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const auto val = static_cast<unsigned char>((x + kShift + y) % 256);
            const size_t base         = static_cast<size_t>(y * kWidth + x) * 3;
            pair.right.data[base + 0] = val;
            pair.right.data[base + 1] = val;
            pair.right.data[base + 2] = val;
        }
    }

    return pair;
}

} // namespace score

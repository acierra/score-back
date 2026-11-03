#include "cloud_point/scared_dataset_loader.hpp"

#include <cstring>
#include <stdexcept>
#include <vector>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace score {

namespace {

/// Flatten a cv::Mat (row-major) to a std::vector<double>.
std::vector<double> mat_to_vec(const cv::Mat &m) {
    cv::Mat d64;
    m.convertTo(d64, CV_64F);
    std::vector<double> v(static_cast<size_t>(d64.total()));
    std::copy(d64.begin<double>(), d64.end<double>(), v.begin());
    return v;
}

/// Load a PNG from @p path and encode as BGR ImageRPC.
/// Reads with IMREAD_COLOR (→ BGR 8-bit); throws on failure.
ImageRPC load_bgr_image(const std::string &path) {
    const cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {
        throw std::runtime_error(
            "ScaredDatasetLoader: cannot read image: " + path);
    }
    ImageRPC rpc;
    rpc.width       = img.cols;
    rpc.height      = img.rows;
    rpc.type        = ImageRPC::Type::BGR;
    const size_t sz = static_cast<size_t>(img.cols) * img.rows * 3;
    rpc.data.resize(sz);
    std::memcpy(rpc.data.data(), img.data, sz);
    return rpc;
}

} // namespace

ScaredDatasetLoader::ScaredDatasetLoader(const std::string &keyframe_dir) {
    const std::string yaml_path  = keyframe_dir + "/endoscope_calibration.yaml";
    const std::string left_path  = keyframe_dir + "/Left_Image.png";
    const std::string right_path = keyframe_dir + "/Right_Image.png";

    LOG(INFO) << "ScaredDatasetLoader: loading calibration from " << yaml_path;

    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error(
            "ScaredDatasetLoader: cannot open calibration YAML: " + yaml_path);
    }

    cv::Mat M1, D1, M2, D2, R, T;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    fs["R"]  >> R;
    fs["T"]  >> T;
    fs.release();

    if (M1.empty() || D1.empty() || M2.empty() || D2.empty() || R.empty() ||
        T.empty()) {
        throw std::runtime_error(
            "ScaredDatasetLoader: missing node in calibration YAML: " +
            yaml_path);
    }

    // Load images first so we can populate width/height from actual dimensions.
    LOG(INFO) << "ScaredDatasetLoader: loading images";
    left_image_  = load_bgr_image(left_path);
    right_image_ = load_bgr_image(right_path);

    if (left_image_.width != right_image_.width ||
        left_image_.height != right_image_.height) {
        throw std::runtime_error(
            "ScaredDatasetLoader: stereo pair dimension mismatch: left " +
            std::to_string(left_image_.width) + "x" +
            std::to_string(left_image_.height) + " vs right " +
            std::to_string(right_image_.width) + "x" +
            std::to_string(right_image_.height));
    }

    calib_.width  = left_image_.width;
    calib_.height = left_image_.height;

    calib_.left.camera_matrix  = mat_to_vec(M1);
    calib_.left.dist_coeffs    = mat_to_vec(D1);
    calib_.right.camera_matrix = mat_to_vec(M2);
    calib_.right.dist_coeffs   = mat_to_vec(D2);
    calib_.rotation            = mat_to_vec(R);

    // T is stored as 1x3 in millimetres; convert to metres.
    const auto t_mm = mat_to_vec(T);
    calib_.translation.resize(3);
    calib_.translation[0] = t_mm[0] / 1000.0;
    calib_.translation[1] = t_mm[1] / 1000.0;
    calib_.translation[2] = t_mm[2] / 1000.0;

    LOG(INFO) << "ScaredDatasetLoader: T(mm)=["
              << t_mm[0] << "," << t_mm[1] << "," << t_mm[2]
              << "] -> T(m)=["
              << calib_.translation[0] << ","
              << calib_.translation[1] << ","
              << calib_.translation[2] << "]";
    LOG(INFO) << "ScaredDatasetLoader: image size "
              << calib_.width << "x" << calib_.height;
}

const StereoCalibrationRPC &
ScaredDatasetLoader::calibration() const noexcept {
    return calib_;
}

ImagePairRPC ScaredDatasetLoader::image_pair(uint64_t frame) const {
    ImagePairRPC pair;
    pair.frame = frame;
    pair.left  = left_image_;
    pair.right = right_image_;
    return pair;
}

} // namespace score

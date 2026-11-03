#include "cloud_point/scared_ground_truth_loader.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace score {

namespace {

float parse_coordinate(const std::string &token, const std::string &path,
                       std::size_t line_number) {
    try {
        std::size_t parsed = 0;
        const float value  = std::stof(token, &parsed);
        if (parsed != token.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    } catch (const std::exception &) {
        throw std::runtime_error(
            "ScaredGroundTruthLoader: invalid coordinate at " + path + ":" +
            std::to_string(line_number) + ": " + token);
    }
}

} // namespace

ScaredGroundTruthLoader::ScaredGroundTruthLoader(
    const std::string &keyframe_dir, cv::Size image_size,
    float units_to_metres) {
    if (image_size.width <= 0 || image_size.height <= 0) {
        throw std::invalid_argument(
            "ScaredGroundTruthLoader: image dimensions must be positive");
    }
    if (!std::isfinite(units_to_metres) || units_to_metres <= 0.0f) {
        throw std::invalid_argument(
            "ScaredGroundTruthLoader: units_to_metres must be finite and "
            "positive");
    }

    const std::string path = keyframe_dir + "/point_cloud.obj";
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error(
            "ScaredGroundTruthLoader: cannot open point cloud: " + path);
    }

    const auto expected_count = static_cast<std::size_t>(image_size.width) *
                                static_cast<std::size_t>(image_size.height);
    std::vector<cv::Vec3f> vertices;
    vertices.reserve(expected_count);

    std::string line;
    std::size_t line_number = 0;
    const float nan         = std::numeric_limits<float>::quiet_NaN();
    while (std::getline(input, line)) {
        ++line_number;
        std::istringstream stream(line);
        std::string record;
        stream >> record;
        if (record != "v") {
            continue;
        }

        std::string x_token, y_token, z_token;
        if (!(stream >> x_token >> y_token >> z_token)) {
            throw std::runtime_error(
                "ScaredGroundTruthLoader: incomplete vertex at " + path + ":" +
                std::to_string(line_number));
        }

        const float x = parse_coordinate(x_token, path, line_number);
        const float y = parse_coordinate(y_token, path, line_number);
        const float z = parse_coordinate(z_token, path, line_number);
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            vertices.emplace_back(x * units_to_metres, y * units_to_metres,
                                  z * units_to_metres);
            ++valid_point_count_;
        } else {
            vertices.emplace_back(nan, nan, nan);
        }
    }

    if (vertices.size() != expected_count) {
        throw std::runtime_error("ScaredGroundTruthLoader: expected " +
                                 std::to_string(expected_count) +
                                 " vertices in " + path + ", got " +
                                 std::to_string(vertices.size()));
    }

    point_map_ = cv::Mat(image_size, CV_32FC3);
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        point_map_.at<cv::Vec3f>(static_cast<int>(index / image_size.width),
                                 static_cast<int>(index % image_size.width)) =
            vertices[index];
    }
}

const cv::Mat &ScaredGroundTruthLoader::point_map() const noexcept {
    return point_map_;
}

std::size_t ScaredGroundTruthLoader::valid_point_count() const noexcept {
    return valid_point_count_;
}

} // namespace score

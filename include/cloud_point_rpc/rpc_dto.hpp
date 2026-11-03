// Created as part of Phase 1: stereo calibration and image-pair wire protocol.
#pragma once

#include "cloud_point_rpc/imageRpc.h"
#include "cloud_point_rpc/rpc_coder.hpp"
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace score {

// ---------------------------------------------------------------------------
// CameraCalib
// ---------------------------------------------------------------------------

/// @brief Intrinsic parameters for a single camera.
struct CameraCalib {
    std::vector<double> camera_matrix; ///< 9 elements, row-major 3x3
    std::vector<double> dist_coeffs;   ///< 5 elements: k1 k2 p1 p2 k3
};

// ---------------------------------------------------------------------------
// StereoCalibrationRPC
// ---------------------------------------------------------------------------

/// @brief Full stereo rig calibration result.
struct StereoCalibrationRPC {
    CameraCalib left;
    CameraCalib right;
    std::vector<double> rotation;    ///< 9 elements, row-major 3x3
    std::vector<double> translation; ///< 3 elements, metres (OpenCV convention)
    int width{0};
    int height{0};
};

// ---------------------------------------------------------------------------
// ImagePairRPC
// ---------------------------------------------------------------------------

/// @brief A synchronised stereo frame.
struct ImagePairRPC {
    uint64_t frame{0};
    ImageRPC left;
    ImageRPC right;
};

// ---------------------------------------------------------------------------
// nlohmann ADL hooks — ImageRPC
// ---------------------------------------------------------------------------

inline std::string image_type_to_string(ImageRPC::Type t) {
    switch (t) {
    case ImageRPC::Type::BGR:
        return "BGR";
    case ImageRPC::Type::RGBA:
        return "RGBA";
    case ImageRPC::Type::DEPTH:
        return "DEPTH";
    default:
        return "UNKNOWN";
    }
}

inline ImageRPC::Type image_type_from_string(const std::string &s) {
    if (s == "BGR")
        return ImageRPC::Type::BGR;
    if (s == "RGBA")
        return ImageRPC::Type::RGBA;
    if (s == "DEPTH")
        return ImageRPC::Type::DEPTH;
    throw std::runtime_error("Unknown ImageRPC type: " + s);
}

inline void to_json(nlohmann::json &j, const ImageRPC &img) {
    Base64RPCCoder coder;
    // ImageRPC::data is vector<unsigned char>; encode() needs vector<char>
    const std::vector<char> as_char(img.data.begin(), img.data.end());
    j = {{"width", img.width},
         {"height", img.height},
         {"type", image_type_to_string(img.type)},
         {"data", coder.encode(as_char)}};
}

inline void from_json(const nlohmann::json &j, ImageRPC &img) {
    j.at("width").get_to(img.width);
    j.at("height").get_to(img.height);
    img.type = image_type_from_string(j.at("type").get<std::string>());

    Base64RPCCoder coder;
    const auto decoded = coder.decode(j.at("data").get<std::string>());
    img.data.assign(decoded.begin(), decoded.end());
}

// ---------------------------------------------------------------------------
// nlohmann ADL hooks — CameraCalib
// ---------------------------------------------------------------------------

inline void to_json(nlohmann::json &j, const CameraCalib &c) {
    j = {{"camera_matrix", c.camera_matrix}, {"dist_coeffs", c.dist_coeffs}};
}

inline void from_json(const nlohmann::json &j, CameraCalib &c) {
    j.at("camera_matrix").get_to(c.camera_matrix);
    j.at("dist_coeffs").get_to(c.dist_coeffs);
    if (c.camera_matrix.size() != 9) {
        throw std::runtime_error("camera_matrix must have 9 elements, got " +
                                 std::to_string(c.camera_matrix.size()));
    }
    if (c.dist_coeffs.size() != 5) {
        throw std::runtime_error("dist_coeffs must have 5 elements, got " +
                                 std::to_string(c.dist_coeffs.size()));
    }
}

// ---------------------------------------------------------------------------
// nlohmann ADL hooks — StereoCalibrationRPC
// ---------------------------------------------------------------------------

inline void to_json(nlohmann::json &j, const StereoCalibrationRPC &s) {
    j = {{"left", s.left},
         {"right", s.right},
         {"rotation", s.rotation},
         {"translation", s.translation},
         {"image_size", {{"width", s.width}, {"height", s.height}}}};
}

inline void from_json(const nlohmann::json &j, StereoCalibrationRPC &s) {
    j.at("left").get_to(s.left);
    j.at("right").get_to(s.right);
    j.at("rotation").get_to(s.rotation);
    j.at("translation").get_to(s.translation);
    s.width  = j.at("image_size").at("width").get<int>();
    s.height = j.at("image_size").at("height").get<int>();

    if (s.rotation.size() != 9) {
        throw std::runtime_error("rotation must have 9 elements, got " +
                                 std::to_string(s.rotation.size()));
    }
    if (s.translation.size() != 3) {
        throw std::runtime_error("translation must have 3 elements, got " +
                                 std::to_string(s.translation.size()));
    }
}

// ---------------------------------------------------------------------------
// nlohmann ADL hooks — ImagePairRPC
// ---------------------------------------------------------------------------

inline void to_json(nlohmann::json &j, const ImagePairRPC &p) {
    j = {{"frame", p.frame}, {"left", p.left}, {"right", p.right}};
}

inline void from_json(const nlohmann::json &j, ImagePairRPC &p) {
    j.at("frame").get_to(p.frame);
    j.at("left").get_to(p.left);
    j.at("right").get_to(p.right);
}

} // namespace score

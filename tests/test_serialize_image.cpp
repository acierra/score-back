// Unit and in-process integration tests for rpc_dto.hpp serialisation.
// No OpenCV dependency — safe to compile unconditionally.
#include "cloud_point_rpc/rpc_dto.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/service.hpp"
#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>

using namespace score;
using json = nlohmann::json;

class SerializeImageTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("SerializeImageTest");
    }
};

// ---------------------------------------------------------------------------
// ImageRPC round-trip — binary data with zero bytes and 255s
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, ImageRPCRoundTrip) {
    ImageRPC img;
    img.width  = 2;
    img.height = 2;
    img.type   = ImageRPC::Type::BGR;
    // 4 pixels x 3 channels = 12 bytes, including 0s and 255s
    img.data = {0, 255, 0, 255, 0, 255, 0, 0, 0, 255, 255, 255};

    json j;
    to_json(j, img);

    EXPECT_EQ(j["width"].get<int>(), 2);
    EXPECT_EQ(j["height"].get<int>(), 2);
    EXPECT_EQ(j["type"].get<std::string>(), "BGR");
    EXPECT_TRUE(j["data"].is_string());

    ImageRPC decoded;
    from_json(j, decoded);
    EXPECT_EQ(decoded.width, img.width);
    EXPECT_EQ(decoded.height, img.height);
    EXPECT_EQ(decoded.type, img.type);
    EXPECT_EQ(decoded.data, img.data);
}

// ---------------------------------------------------------------------------
// ImageRPC — unknown type string throws
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, ImageRPCUnknownTypeThrows) {
    json j = {{"width", 1}, {"height", 1}, {"type", "XYZ"}, {"data", "AAAA"}};
    ImageRPC img;
    EXPECT_THROW(from_json(j, img), std::runtime_error);
}

// ---------------------------------------------------------------------------
// ImageRPC — RGBA type string survives round-trip
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, ImageRPCRGBARoundTrip) {
    ImageRPC img;
    img.width  = 1;
    img.height = 1;
    img.type   = ImageRPC::Type::RGBA;
    img.data   = {10, 20, 30, 40};

    json j;
    to_json(j, img);
    EXPECT_EQ(j["type"].get<std::string>(), "RGBA");

    ImageRPC out;
    from_json(j, out);
    EXPECT_EQ(out.type, ImageRPC::Type::RGBA);
    EXPECT_EQ(out.data, img.data);
}

// ---------------------------------------------------------------------------
// ImageRPC — DEPTH type string survives round-trip
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, ImageRPCDEPTHRoundTrip) {
    ImageRPC img;
    img.width  = 1;
    img.height = 1;
    img.type   = ImageRPC::Type::DEPTH;
    img.data   = {0x3f, 0x80, 0x00, 0x00}; // 1.0f as little-endian float32

    json j;
    to_json(j, img);
    EXPECT_EQ(j["type"].get<std::string>(), "DEPTH");

    ImageRPC out;
    from_json(j, out);
    EXPECT_EQ(out.type, ImageRPC::Type::DEPTH);
    EXPECT_EQ(out.data, img.data);
}

// ---------------------------------------------------------------------------
// CameraCalib round-trip
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, CameraCalibRoundTrip) {
    CameraCalib c;
    c.camera_matrix = {800, 0, 320, 0, 800, 240, 0, 0, 1};
    c.dist_coeffs   = {0.1, 0.2, 0.0, 0.0, 0.05};

    json j;
    to_json(j, c);

    CameraCalib out;
    from_json(j, out);
    EXPECT_EQ(out.camera_matrix, c.camera_matrix);
    EXPECT_EQ(out.dist_coeffs, c.dist_coeffs);
}

// ---------------------------------------------------------------------------
// CameraCalib — wrong camera_matrix size (8 elements) → throws
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, CameraCalibWrongCameraMatrixSizeThrows) {
    json j = {{"camera_matrix", {1, 2, 3, 4, 5, 6, 7, 8}},
              {"dist_coeffs", {0, 0, 0, 0, 0}}};
    CameraCalib c;
    EXPECT_THROW(from_json(j, c), std::runtime_error);
}

// ---------------------------------------------------------------------------
// CameraCalib — wrong dist_coeffs size → throws
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, CameraCalibWrongDistCoeffsSizeThrows) {
    json j = {{"camera_matrix", {800, 0, 320, 0, 800, 240, 0, 0, 1}},
              {"dist_coeffs", {0, 0, 0}}};
    CameraCalib c;
    EXPECT_THROW(from_json(j, c), std::runtime_error);
}

// ---------------------------------------------------------------------------
// StereoCalibrationRPC round-trip
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, StereoCalibrationRoundTrip) {
    StereoCalibrationRPC calib;
    calib.left.camera_matrix = {800, 0, 320, 0, 800, 240, 0, 0, 1};
    calib.left.dist_coeffs   = {0, 0, 0, 0, 0};
    calib.right              = calib.left;
    calib.rotation           = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    calib.translation        = {-0.06, 0.0, 0.0};
    calib.width              = 640;
    calib.height             = 480;

    json j;
    to_json(j, calib);

    StereoCalibrationRPC out;
    from_json(j, out);

    EXPECT_EQ(out.left.camera_matrix, calib.left.camera_matrix);
    EXPECT_EQ(out.right.dist_coeffs, calib.right.dist_coeffs);
    EXPECT_EQ(out.rotation, calib.rotation);
    EXPECT_DOUBLE_EQ(out.translation[0], -0.06);
    EXPECT_DOUBLE_EQ(out.translation[1], 0.0);
    EXPECT_DOUBLE_EQ(out.translation[2], 0.0);
    EXPECT_EQ(out.width, 640);
    EXPECT_EQ(out.height, 480);
}

// ---------------------------------------------------------------------------
// StereoCalibrationRPC — wrong rotation size → throws
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, StereoCalibWrongRotationSizeThrows) {
    // rotation has only 5 elements — must throw
    json j = {{"left",
               {{"camera_matrix", {800, 0, 320, 0, 800, 240, 0, 0, 1}},
                {"dist_coeffs", {0, 0, 0, 0, 0}}}},
              {"right",
               {{"camera_matrix", {800, 0, 320, 0, 800, 240, 0, 0, 1}},
                {"dist_coeffs", {0, 0, 0, 0, 0}}}},
              {"rotation", {1, 0, 0, 0, 1}},
              {"translation", {-0.06, 0, 0}},
              {"image_size", {{"width", 640}, {"height", 480}}}};
    StereoCalibrationRPC calib;
    EXPECT_THROW(from_json(j, calib), std::runtime_error);
}

// ---------------------------------------------------------------------------
// StereoCalibrationRPC — wrong translation size → throws
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, StereoCalibWrongTranslationSizeThrows) {
    json j = {{"left",
               {{"camera_matrix", {800, 0, 320, 0, 800, 240, 0, 0, 1}},
                {"dist_coeffs", {0, 0, 0, 0, 0}}}},
              {"right",
               {{"camera_matrix", {800, 0, 320, 0, 800, 240, 0, 0, 1}},
                {"dist_coeffs", {0, 0, 0, 0, 0}}}},
              {"rotation", {1, 0, 0, 0, 1, 0, 0, 0, 1}},
              {"translation", {-0.06}}, // only 1 element — wrong
              {"image_size", {{"width", 640}, {"height", 480}}}};
    StereoCalibrationRPC calib;
    EXPECT_THROW(from_json(j, calib), std::runtime_error);
}

// ---------------------------------------------------------------------------
// ImagePairRPC round-trip
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, ImagePairRPCRoundTrip) {
    ImagePairRPC pair;
    pair.frame       = 42;
    pair.left.width  = 2;
    pair.left.height = 1;
    pair.left.type   = ImageRPC::Type::BGR;
    pair.left.data   = {0, 128, 255, 10, 20, 30};
    pair.right       = pair.left;
    pair.right.data  = {5, 6, 7, 8, 9, 10};

    json j;
    to_json(j, pair);

    ImagePairRPC out;
    from_json(j, out);

    EXPECT_EQ(out.frame, 42u);
    EXPECT_EQ(out.left.data, pair.left.data);
    EXPECT_EQ(out.right.data, pair.right.data);
    EXPECT_EQ(out.left.type, ImageRPC::Type::BGR);
}

// ---------------------------------------------------------------------------
// In-process integration: Service -> RpcServer -> process() -> from_json
// ---------------------------------------------------------------------------
TEST_F(SerializeImageTest, ServiceStereoCalibrationViaRpcServer) {
    Service service;
    RpcServer rpc;

    rpc.register_method("get-stereo-calibration", [&](const json &) -> json {
        json j;
        score::to_json(j, service.get_stereo_calibration());
        return j;
    });

    const std::string request =
        R"({"jsonrpc":"2.0","method":"get-stereo-calibration","id":1})";
    const std::string response_str = rpc.process(request);

    auto resp                      = json::parse(response_str);
    ASSERT_TRUE(resp.contains("result")) << "Response was: " << response_str;

    StereoCalibrationRPC calib;
    from_json(resp["result"], calib);

    EXPECT_DOUBLE_EQ(calib.left.camera_matrix[0], 800.0); // fx
    EXPECT_DOUBLE_EQ(calib.left.camera_matrix[4], 800.0); // fy
    EXPECT_DOUBLE_EQ(calib.left.camera_matrix[2], 320.0); // cx
    EXPECT_DOUBLE_EQ(calib.left.camera_matrix[5], 240.0); // cy
    EXPECT_DOUBLE_EQ(calib.translation[0], -0.06);
    EXPECT_DOUBLE_EQ(calib.translation[1], 0.0);
    EXPECT_EQ(calib.width, 640);
    EXPECT_EQ(calib.height, 480);
    // Rotation should be identity
    EXPECT_DOUBLE_EQ(calib.rotation[0], 1.0);
    EXPECT_DOUBLE_EQ(calib.rotation[4], 1.0);
    EXPECT_DOUBLE_EQ(calib.rotation[8], 1.0);
}

TEST_F(SerializeImageTest, ServiceImagePairViaRpcServer) {
    Service service;
    RpcServer rpc;

    rpc.register_method("get-image-pair", [&](const json &) -> json {
        json j;
        score::to_json(j, service.get_image_pair());
        return j;
    });

    const std::string request =
        R"({"jsonrpc":"2.0","method":"get-image-pair","id":1})";
    const std::string response_str = rpc.process(request);

    auto resp                      = json::parse(response_str);
    ASSERT_TRUE(resp.contains("result")) << "Response was: " << response_str;

    ImagePairRPC pair;
    from_json(resp["result"], pair);

    EXPECT_EQ(pair.frame, 0u);
    EXPECT_EQ(pair.left.width, 640);
    EXPECT_EQ(pair.left.height, 480);
    EXPECT_EQ(pair.left.type, ImageRPC::Type::BGR);
    EXPECT_EQ(pair.right.width, 640);
    EXPECT_EQ(pair.right.height, 480);

    // Pixel at (x=0, y=0): left = (0+0)%256 = 0
    EXPECT_EQ(pair.left.data[0], static_cast<unsigned char>(0));
    // Pixel at (x=0, y=0): right = (0+8+0)%256 = 8
    EXPECT_EQ(pair.right.data[0], static_cast<unsigned char>(8));

    // Second call: frame counter should increment
    const std::string request2 =
        R"({"jsonrpc":"2.0","method":"get-image-pair","id":2})";
    const std::string response_str2 = rpc.process(request2);
    auto resp2                      = json::parse(response_str2);
    ImagePairRPC pair2;
    from_json(resp2["result"], pair2);
    EXPECT_EQ(pair2.frame, 1u);
}

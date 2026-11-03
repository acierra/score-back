/// @file test_scared_dataset.cpp
/// @brief E2E test: in-process server backed by SCARED dataset + CloudPointClient.
///
/// Skipped unless env var SCARED_KEYFRAME_DIR is set (CI has no dataset).
/// Run locally:
///   SCARED_KEYFRAME_DIR=/path/to/test_dataset_8/keyframe_0 \
///   ./build/tests/unit_tests --gtest_filter=ScaredDataset*
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "cloud_point/cloud_point_client.hpp"
#include "cloud_point/scared_dataset_loader.hpp"
#include "cloud_point_rpc/rpc_dto.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/tcp_server.hpp"

using namespace score;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Fixture: in-process TcpServer + RpcServer backed by the SCARED loader
// ---------------------------------------------------------------------------

class ScaredDatasetTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestScaredDataset");

        const char *env = std::getenv("SCARED_KEYFRAME_DIR");
        if (!env || std::string(env).empty()) {
            GTEST_SKIP() << "SCARED_KEYFRAME_DIR not set; "
                            "skipping SCARED E2E test";
        }
        keyframe_dir_ = env;
    }

    void TearDown() override {
        if (server_) {
            server_->stop();
        }
    }

    void start_server(int port, std::unique_ptr<RpcServer> rpc) {
        rpc_server_ = std::move(rpc);
        server_     = std::make_unique<TcpServer>(
            "127.0.0.1", port,
            [this](const std::string &req) {
                return rpc_server_->process(req);
            });
        server_->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::string keyframe_dir_;
    std::unique_ptr<RpcServer> rpc_server_;
    std::unique_ptr<TcpServer> server_;
};

// ---------------------------------------------------------------------------
// Test: cloud non-empty and median z within plausible endoscopy range
// ---------------------------------------------------------------------------

TEST_F(ScaredDatasetTest, ComputeCloudFromRealData) {
    constexpr int kPort           = 9301;
    // SCARED rig: fx~1024, B~4.35 mm -> max disparity ~160 needed
    constexpr int kNumDisparities = 160;
    // Expected depth range for endoscopy: 20 mm - 200 mm
    constexpr float kMinExpectedZ = 0.02f;
    constexpr float kMaxExpectedZ = 0.20f;
    // Minimum valid points for a non-trivial cloud
    constexpr size_t kMinValidPts = 50'000;

    ScaredDatasetLoader loader(keyframe_dir_);

    std::atomic<uint64_t> frame_counter{0};
    auto rpc               = std::make_unique<RpcServer>();

    rpc->register_method(
        "get-stereo-calibration", [&](const json &) -> json {
            json j;
            to_json(j, loader.calibration());
            return j;
        });
    rpc->register_method(
        "get-image-pair", [&](const json &) -> json {
            json j;
            to_json(j, loader.image_pair(frame_counter++));
            return j;
        });

    start_server(kPort, std::move(rpc));

    CloudPointClient client("127.0.0.1", kPort, StereoAlgorithmType::CPU,
                            PointCloudBuilder::Options{}, kNumDisparities);
    ASSERT_NO_THROW(client.connect());
    ASSERT_TRUE(client.connected());

    auto result = client.compute_cloud();
    ASSERT_TRUE(result.has_value())
        << "compute_cloud returned Error: " << result.error().message;

    const auto &cloud       = *result;
    const auto valid_points = cloud.valid_points();

    EXPECT_GE(valid_points.size(), kMinValidPts)
        << "Expected >" << kMinValidPts << " valid points, got "
        << valid_points.size();

    // Collect z values and compute median.
    std::vector<float> z_vals;
    z_vals.reserve(valid_points.size());
    for (const auto &pt : valid_points) {
        z_vals.push_back(pt[2]);
    }

    ASSERT_FALSE(z_vals.empty()) << "No valid points in cloud";

    const auto mid =
        z_vals.begin() + static_cast<ptrdiff_t>(z_vals.size() / 2);
    std::nth_element(z_vals.begin(), mid, z_vals.end());
    const float median_z = *mid;

    // Report for the task summary.
    std::cout << "[SCARED] valid_points=" << valid_points.size()
              << " median_z=" << median_z << " m\n";

    EXPECT_GE(median_z, kMinExpectedZ)
        << "Median z " << median_z
        << " m is below minimum expected " << kMinExpectedZ
        << " m (check mm->m conversion)";
    EXPECT_LE(median_z, kMaxExpectedZ)
        << "Median z " << median_z
        << " m exceeds maximum expected " << kMaxExpectedZ
        << " m (check mm->m conversion: T must be divided by 1000)";
}

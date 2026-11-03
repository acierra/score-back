#include <chrono>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

#include "cloud_point_rpc/config.hpp"
#include "cloud_point_rpc/rpc_client.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/service.hpp"
#include "cloud_point_rpc/tcp_server.hpp"

#include <fstream>

using namespace score;

class IntegrationTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestCli");
        // Create a temporary config file for testing
        std::ofstream config_file("config.yaml");
        config_file
            << "server:\n"
            << "  ip: \"127.0.0.1\"\n"
            << "  port: 9095\n"
            << "test_data:\n"
            << "  intrinsic_params: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, "
               "9.0]\n"
            << "  extrinsic_params: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n"
            << "  cloud_point:\n"
            << "    - [0.1, 0.2, 0.3]\n";
        config_file.close();

        // Setup Mock Server
        try {
            config_ = ConfigLoader::load("config.yaml");
        } catch (...) {
            // If config fails, we can't proceed, but we should avoid crashing
            // in TearDown
            throw;
        }

        service_    = std::make_unique<Service>(config_.test_data);
        rpc_server_ = std::make_unique<RpcServer>();

        rpc_server_->register_method(
            "get-intrinsic-params", [&](const nlohmann::json &) {
                return service_->get_intrinsic_params();
            });

        // Start Server Thread
        tcp_server_ = std::make_unique<TcpServer>(
            config_.server.ip, config_.server.port,
            [&](const std::string &req) { return rpc_server_->process(req); });

        server_thread_ = std::thread([&]() { tcp_server_->start(); });

        // Give server time to start
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void TearDown() override {
        if (tcp_server_) {
            tcp_server_->stop();
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        // Clean up
        std::remove("config.yaml");
    }

    Config config_{};
    std::unique_ptr<Service> service_;
    std::unique_ptr<RpcServer> rpc_server_;
    std::unique_ptr<TcpServer> tcp_server_;
    std::thread server_thread_;
};

TEST_F(IntegrationTest, ClientCanConnectAndRetrieveValues) {
    TCPConnector connector(config_.server.ip, config_.server.port);
    RpcClient client(connector);

    // Act: Call Method
    std::vector<double> params;
    EXPECT_NO_THROW(params = client.get_intrinsic_params());

    // Assert: Values match config
    const auto &expected = config_.test_data.intrinsic_params;
    ASSERT_EQ(params.size(), expected.size());
    for (size_t i = 0; i < params.size(); ++i) {
        EXPECT_DOUBLE_EQ(params[i], expected[i]);
    }
}

TEST_F(IntegrationTest, ClientHandlesConnectionError) {
    EXPECT_THROW(TCPConnector connector("127.0.0.1", 9999), std::runtime_error);
}

TEST_F(IntegrationTest, ClientRetrieveRemoteMethods) {
    TCPConnector connector(config_.server.ip, config_.server.port);
    RpcClient client(connector);

    auto res = client.call<std::vector<std::string>>("get-available-methods");
    EXPECT_EQ(res.size(), 2);
}
#include "cloud_point_rpc/serialize.hpp"
#include "cloud_point_rpc/tcp_connector.hpp"
#include "cloud_point_rpc/tcp_server.hpp"
#include <asio.hpp>
#include <cstdint>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>

class TcpTest : public ::testing::Test {
  public:
    void ExpectedResponse(const std::string &expected_response) {
        expected_ = expected_response;
    }

  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestRPC");
        server_ = std::make_unique<score::TcpServer>(
            "127.0.0.1", 12345, [this](const std::string &request) {
                EXPECT_EQ(request, expected_);
                std::string msg = "Echo: " + request;
                auto v          = score::serialize(msg.length());
                std::string res(v.begin(), v.end());
                res += msg;
                return res;
            });
        server_->start();
    }

    void TearDown() override {
        server_->stop();
        server_.reset();
    }

    std::string expected_;
    std::unique_ptr<score::TcpServer> server_;
};

TEST(SerializeTest, Base) {
    uint64_t value{123};
    auto res = score::serialize(value);
    EXPECT_EQ(value, score::deserialize<uint64_t>(res));
}

TEST_F(TcpTest, EchoTest) {
    constexpr std::string_view msg = "Hello, TCP Server!";
    ExpectedResponse(msg.data());
    score::TCPConnector connector("127.0.0.1", 12345);
    auto res = connector.Send(msg.data());
}

TEST_F(TcpTest, HugeBuffer) {
    static constexpr uint64_t w = 1280, h = 720, c = 3;
    std::string data(w * h * c, 77);
    ExpectedResponse(data);
    score::TCPConnector connector("127.0.0.1", 12345);
    auto res = connector.Send(data);
}

#include "cloud_point_rpc/tcp_connector.hpp"
#include "cloud_point_rpc/tcp_server.hpp"
#include <asio.hpp>
#include <chrono>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

using namespace score;

class TcpEdgeCaseTest : public ::testing::Test {
  protected:
    std::unique_ptr<TcpServer> server_;
    std::thread server_thread_;

    void StartServer(int port, TcpServer::RequestProcessor processor) {
        server_ = std::make_unique<TcpServer>("127.0.0.1", port, processor);
        server_thread_ = std::thread([this]() { server_->start(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void StopServer() {
        if (server_) {
            server_->stop();
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestRPC");
    }

    void TearDown() override { StopServer(); }
};

// Empty payload
// NOTE: The server silently ignores empty payloads (payload_length == 0)
// and closes the connection without sending a response.
TEST_F(TcpEdgeCaseTest, EmptyPayloadGetsNoResponse) {
    StartServer(19001, [](const std::string &req) {
        if (req.empty()) {
            return std::string("empty");
        }
        return req;
    });

    TCPConnector connector("127.0.0.1", 19001);
    auto res = connector.Send("");
    EXPECT_EQ(res, "");
}

// Very small payload (1 byte)
TEST_F(TcpEdgeCaseTest, SingleBytePayload) {
    StartServer(19002, [](const std::string &req) { return req; });

    TCPConnector connector("127.0.0.1", 19002);
    auto res = connector.Send("x");
    EXPECT_EQ(res, "x\n");
}

// Multiple sequential connections
TEST_F(TcpEdgeCaseTest, MultipleSequentialConnections) {
    StartServer(19003, [](const std::string &req) { return req; });

    for (int i = 0; i < 10; ++i) {
        TCPConnector connector("127.0.0.1", 19003);
        auto msg = "msg_" + std::to_string(i);
        auto res = connector.Send(msg);
        EXPECT_EQ(res, msg + "\n");
    }
}

// Multiple concurrent connections
TEST_F(TcpEdgeCaseTest, MultipleConcurrentConnections) {
    std::atomic<int> counter{0};
    StartServer(19004, [&counter](const std::string &req) {
        ++counter;
        return req;
    });

    constexpr int N = 10;
    std::vector<std::thread> threads;
    threads.reserve(N);

    for (int i = 0; i < N; ++i) {
        threads.emplace_back([i]() {
            TCPConnector connector("127.0.0.1", 19004);
            auto msg = "concurrent_" + std::to_string(i);
            auto res = connector.Send(msg);
            EXPECT_EQ(res, msg + "\n");
        });
    }

    for (auto &t : threads) {
        t.join();
    }

    EXPECT_EQ(counter, N);
}

// Server stop and restart on same port
TEST_F(TcpEdgeCaseTest, StopRestartSamePort) {
    StartServer(19005, [](const std::string &req) { return req; });

    {
        TCPConnector connector("127.0.0.1", 19005);
        auto res = connector.Send("first");
        EXPECT_EQ(res, "first\n");
    }

    StopServer();

    // Restart on same port
    StartServer(19005, [](const std::string &req) { return req + "_v2"; });

    {
        TCPConnector connector("127.0.0.1", 19005);
        auto res = connector.Send("second");
        EXPECT_EQ(res, "second_v2\n");
    }
}

// Connection to wrong port fails
TEST_F(TcpEdgeCaseTest, ConnectionToWrongPortFails) {
    StartServer(19006, [](const std::string &req) { return req; });

    EXPECT_THROW(TCPConnector connector("127.0.0.1", 19007),
                 std::runtime_error);
}

// Large payload
TEST_F(TcpEdgeCaseTest, LargePayload) {
    std::string large_data(100000, 'L');
    StartServer(19008, [&large_data](const std::string &req) {
        if (req == large_data) {
            return std::string("OK");
        }
        return std::string("MISMATCH");
    });

    TCPConnector connector("127.0.0.1", 19008);
    auto res = connector.Send(large_data);
    EXPECT_EQ(res, "OK\n");
}

// Server processor throws exception
TEST_F(TcpEdgeCaseTest, ProcessorThrowsException) {
    StartServer(19009, [](const std::string &) -> std::string {
        throw std::runtime_error("processor error");
    });

    TCPConnector connector("127.0.0.1", 19009);
    // Should not crash, client may get partial or no response
    EXPECT_NO_THROW(connector.Send("trigger"));
}

// Server start failure (port already in use)
TEST_F(TcpEdgeCaseTest, PortAlreadyInUse) {
    StartServer(19010, [](const std::string &req) { return req; });

    EXPECT_THROW(
        {
            TcpServer duplicate("127.0.0.1", 19010,
                                [](const std::string &req) { return req; });
            duplicate.start();
        },
        std::exception);
}

// Explicit join after stop
TEST_F(TcpEdgeCaseTest, ExplicitJoinAfterStop) {
    StartServer(19011, [](const std::string &req) { return req; });

    server_->stop();
    EXPECT_NO_THROW(server_->join());
}

// Destructor cleanup without explicit stop
TEST_F(TcpEdgeCaseTest, DestructorCleanup) {
    {
        TcpServer local_server("127.0.0.1", 19012,
                               [](const std::string &req) { return req; });
        local_server.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // destructor should clean up without explicit stop
    }
    // If we get here without hanging, destructor works
    EXPECT_TRUE(true);
}

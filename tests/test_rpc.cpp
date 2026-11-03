#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/service.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace score;

class RpcServerTest : public ::testing::Test {
  protected:
    RpcServer server;
    Service service;

    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestCli");
        server.register_method("get-intrinsic-params", [&](const json &) {
            return service.get_intrinsic_params();
        });
    }
};

TEST_F(RpcServerTest, GetIntrinsicParamsReturnsMatrix) {
    std::string request =
        R"({"jsonrpc": "2.0", "method": "get-intrinsic-params", "id": 1})";
    std::string response_str = server.process(request);

    json response            = json::parse(response_str);

    ASSERT_EQ(response["jsonrpc"], "2.0");
    ASSERT_EQ(response["id"], 1);
    ASSERT_TRUE(response.contains("result"));

    auto result = response["result"].get<std::vector<double>>();
    EXPECT_EQ(result.size(), 9);
    // Verify Identity Matrix
    EXPECT_EQ(result[0], 1.0);
    EXPECT_EQ(result[4], 1.0);
    EXPECT_EQ(result[8], 1.0);
}

TEST_F(RpcServerTest, MethodNotFoundReturnsError) {
    std::string request =
        R"({"jsonrpc": "2.0", "method": "unknown-method", "id": 99})";
    std::string response_str = server.process(request);

    json response            = json::parse(response_str);

    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32601);
    EXPECT_EQ(response["error"]["message"], "Method not found");
}

TEST_F(RpcServerTest, InvalidJsonReturnsParseError) {
    std::string request = R"({"jsonrpc": "2.0", "method": "broken-json...)";
    std::string response_str = server.process(request);

    json response            = json::parse(response_str);

    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32700);
}

TEST_F(RpcServerTest, GetMethod) {
    EXPECT_EQ(server.get_count(), 2);
    EXPECT_EQ(server.get_method_name_by_id(1), "get-intrinsic-params");
    EXPECT_EQ(server.get_method_names()[1], "get-intrinsic-params");

    server.register_method("get-test-2", [&](const json &) {
        return std::variant<json, std::string>{std::string("test")};
    });
    EXPECT_EQ(server.get_count(), 3);
    EXPECT_EQ(server.get_method_name_by_id(2), "get-test-2");
    EXPECT_EQ(server.get_method_names()[2], "get-test-2");
}

#include "cloud_point_rpc/rpc_server.hpp"
#include "server_api.h"
#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

using json = nlohmann::json;
using namespace score;

class RpcServerEdgeCaseTest : public ::testing::Test {
  protected:
    RpcServer server;

    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestCli");
        server.register_method(
            "echo", [&](const json &j) { return j.get<std::string>(); });
        server.register_method("thrower", [&](const json &) -> std::string {
            throw std::runtime_error("intentional error");
        });
    }
};

// Empty request string
TEST_F(RpcServerEdgeCaseTest, EmptyRequestReturnsParseError) {
    std::string response_str = server.process("");
    json response            = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32700);
}

// Valid JSON but primitive types (not object)
// NOTE: These currently throw nlohmann::json::type_error instead of returning
// Invalid Request. This documents a known bug in type validation.
TEST_F(RpcServerEdgeCaseTest, JsonArrayThrowsTypeError) {
    EXPECT_THROW(server.process(R"([1, 2, 3])"), nlohmann::json::type_error);
}

TEST_F(RpcServerEdgeCaseTest, JsonStringThrowsTypeError) {
    EXPECT_THROW(server.process(R"("just a string")"),
                 nlohmann::json::type_error);
}

TEST_F(RpcServerEdgeCaseTest, JsonNumberThrowsTypeError) {
    EXPECT_THROW(server.process("42"), nlohmann::json::type_error);
}

TEST_F(RpcServerEdgeCaseTest, JsonNullThrowsTypeError) {
    EXPECT_THROW(server.process("null"), nlohmann::json::type_error);
}

// Missing required fields
TEST_F(RpcServerEdgeCaseTest, MissingJsonrpcField) {
    std::string response_str = server.process(R"({"method": "echo", "id": 1})");
    json response            = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32600);
}

TEST_F(RpcServerEdgeCaseTest, WrongJsonrpcVersion) {
    std::string response_str =
        server.process(R"({"jsonrpc": "1.0", "method": "echo", "id": 1})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32600);
}

TEST_F(RpcServerEdgeCaseTest, MissingMethodField) {
    std::string response_str = server.process(R"({"jsonrpc": "2.0", "id": 1})");
    json response            = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32600);
}

TEST_F(RpcServerEdgeCaseTest, MissingIdField) {
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "echo"})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32600);
}

// Method field type validation
// NOTE: These currently throw nlohmann::json::type_error instead of returning
// Invalid Request. This documents a known bug in type validation.
TEST_F(RpcServerEdgeCaseTest, MethodIsNumberThrowsTypeError) {
    EXPECT_THROW(
        server.process(R"({"jsonrpc": "2.0", "method": 123, "id": 1})"),
        nlohmann::json::type_error);
}

TEST_F(RpcServerEdgeCaseTest, MethodIsNullThrowsTypeError) {
    EXPECT_THROW(
        server.process(R"({"jsonrpc": "2.0", "method": null, "id": 1})"),
        nlohmann::json::type_error);
}

TEST_F(RpcServerEdgeCaseTest, MethodIsArrayThrowsTypeError) {
    EXPECT_THROW(
        server.process(R"({"jsonrpc": "2.0", "method": ["echo"], "id": 1})"),
        nlohmann::json::type_error);
}

TEST_F(RpcServerEdgeCaseTest, MethodIsObjectThrowsTypeError) {
    EXPECT_THROW(
        server.process(
            R"({"jsonrpc": "2.0", "method": {"name": "echo"}, "id": 1})"),
        nlohmann::json::type_error);
}

// Handler exceptions
TEST_F(RpcServerEdgeCaseTest, HandlerThrowsReturnsServerError) {
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "thrower", "id": 42})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32000);
    // Should not leak internal details ideally, but current impl does
    EXPECT_EQ(response["error"]["message"], "intentional error");
}

// Valid request with params
TEST_F(RpcServerEdgeCaseTest, RequestWithParams) {
    server.register_method("add", [&](const json &j) {
        return j.at("a").get<int>() + j.at("b").get<int>();
    });
    std::string response_str = server.process(
        R"({"jsonrpc": "2.0", "method": "add", "id": 1, "params": {"a": 2, "b": 3}})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("result"));
    EXPECT_EQ(response["result"], 5);
}

// Request with empty params object
// The echo handler expects a string but gets an empty object, so it throws.
TEST_F(RpcServerEdgeCaseTest, RequestWithEmptyParamsHandlerThrows) {
    std::string response_str = server.process(
        R"({"jsonrpc": "2.0", "method": "echo", "id": 1, "params": {}})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32000);
}

// C callback edge cases
// NOTE: When the C callback returns nullptr, the wrapper returns {} which
// value-initializes the variant's first alternative (json null). This is a
// bug: it should throw to trigger a proper error response.
TEST_F(RpcServerEdgeCaseTest, CCallbackReturnsNullProducesNullResult) {
    server.register_method(
        "null_cb", [](rpc_string *) -> rpc_string * { return nullptr; });
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "null_cb", "id": 1})");
    json response = json::parse(response_str);
    // Current behavior: returns success with null result due to variant
    // value-initialization bug
    ASSERT_TRUE(response.contains("result"));
    EXPECT_TRUE(response["result"].is_null());
}

TEST_F(RpcServerEdgeCaseTest, CCallbackReturnsNonJsonString) {
    server.register_method("raw_cb", [](rpc_string *) -> rpc_string * {
        return crpc_str_create("hello world", 11);
    });
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "raw_cb", "id": 1})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("result"));
    EXPECT_EQ(response["result"], "hello world");
}

TEST_F(RpcServerEdgeCaseTest, CCallbackReturnsValidJson) {
    server.register_method("json_cb", [](rpc_string *) -> rpc_string * {
        return crpc_str_create(R"({"key": "value"})", 16);
    });
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "json_cb", "id": 1})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("result"));
    EXPECT_EQ(response["result"]["key"], "value");
}

// Thread safety: concurrent register and process
TEST_F(RpcServerEdgeCaseTest, ConcurrentRegisterAndProcess) {
    constexpr int kIterations = 100;
    std::atomic<int> success_count{0};

    std::thread registrar([&]() {
        for (int i = 0; i < kIterations; ++i) {
            server.register_method("dyn_" + std::to_string(i),
                                   [&](const json &j) { return j.get<int>(); });
        }
    });

    std::thread processor([&]() {
        for (int i = 0; i < kIterations; ++i) {
            std::string req = R"({"jsonrpc": "2.0", "method": "echo", "id": )" +
                              std::to_string(i) + "}";
            try {
                auto res = server.process(req);
                if (!res.empty())
                    ++success_count;
            } catch (...) {
                // ignore races
            }
        }
    });

    registrar.join();
    processor.join();

    EXPECT_EQ(success_count, kIterations);
}

// Unicode and special characters in method name
TEST_F(RpcServerEdgeCaseTest, UnicodeMethodNameNotFound) {
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "метод", "id": 1})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32601);
}

TEST_F(RpcServerEdgeCaseTest, MethodWithNewlineNotFound) {
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "echo\n", "id": 1})");
    json response = json::parse(response_str);
    ASSERT_TRUE(response.contains("error"));
    EXPECT_EQ(response["error"]["code"], -32601);
}

// Id edge cases
TEST_F(RpcServerEdgeCaseTest, StringIdPreserved) {
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "echo", "id": "abc"})");
    json response = json::parse(response_str);
    EXPECT_EQ(response["id"], "abc");
}

TEST_F(RpcServerEdgeCaseTest, NullIdPreserved) {
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "echo", "id": null})");
    json response = json::parse(response_str);
    EXPECT_TRUE(response["id"].is_null());
}

TEST_F(RpcServerEdgeCaseTest, ZeroIdPreserved) {
    std::string response_str =
        server.process(R"({"jsonrpc": "2.0", "method": "echo", "id": 0})");
    json response = json::parse(response_str);
    EXPECT_EQ(response["id"], 0);
}

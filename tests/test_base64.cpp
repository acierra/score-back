//
// Created by vptyp on 11.03.2026.
//
#include <chrono>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

#include "cloud_point_rpc/config.hpp"
#include "cloud_point_rpc/rpc_coder.hpp"

class Base64Test : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestCli");
    }
    void TearDown() override {}
};

TEST_F(Base64Test, EncodeDecode) {
    std::vector raw{'H', 'e', 'l', 'l', 'o', 'w', '\0'};
    score::Base64RPCCoder coder;
    auto encoded = coder.encode(raw);
    LOG(INFO) << "encoded: " << encoded;
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(std::ranges::equal(decoded, raw), true);
    LOG(INFO) << "done";
}

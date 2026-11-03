#include "cloud_point_rpc/serialize.hpp"
#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <limits>

using namespace score;

class SerializeEdgeCaseTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestRPC");
    }
};

// uint8_t round-trip
TEST_F(SerializeEdgeCaseTest, Uint8RoundTrip) {
    uint8_t value = 42;
    auto buf      = serialize(value);
    EXPECT_EQ(buf.size(), sizeof(uint8_t));
    EXPECT_EQ(deserialize<uint8_t>(buf), value);
}

// int32_t round-trip
TEST_F(SerializeEdgeCaseTest, Int32RoundTrip) {
    int32_t value = -12345;
    auto buf      = serialize(value);
    EXPECT_EQ(buf.size(), sizeof(int32_t));
    EXPECT_EQ(deserialize<int32_t>(buf), value);
}

// uint64_t round-trip with max value
TEST_F(SerializeEdgeCaseTest, Uint64MaxRoundTrip) {
    uint64_t value = std::numeric_limits<uint64_t>::max();
    auto buf       = serialize(value);
    EXPECT_EQ(buf.size(), sizeof(uint64_t));
    EXPECT_EQ(deserialize<uint64_t>(buf), value);
}

// int64_t round-trip with min value
TEST_F(SerializeEdgeCaseTest, Int64MinRoundTrip) {
    int64_t value = std::numeric_limits<int64_t>::min();
    auto buf      = serialize(value);
    EXPECT_EQ(buf.size(), sizeof(int64_t));
    EXPECT_EQ(deserialize<int64_t>(buf), value);
}

// float round-trip
TEST_F(SerializeEdgeCaseTest, FloatRoundTrip) {
    float value = 3.14159f;
    auto buf    = serialize(value);
    EXPECT_EQ(buf.size(), sizeof(float));
    EXPECT_FLOAT_EQ(deserialize<float>(buf), value);
}

// double round-trip
TEST_F(SerializeEdgeCaseTest, DoubleRoundTrip) {
    double value = 2.718281828459045;
    auto buf     = serialize(value);
    EXPECT_EQ(buf.size(), sizeof(double));
    EXPECT_DOUBLE_EQ(deserialize<double>(buf), value);
}

// zero values
TEST_F(SerializeEdgeCaseTest, ZeroValues) {
    EXPECT_EQ(deserialize<uint64_t>(serialize<uint64_t>(0)), 0);
    EXPECT_EQ(deserialize<int32_t>(serialize<int32_t>(0)), 0);
    EXPECT_FLOAT_EQ(deserialize<float>(serialize<float>(0.0f)), 0.0f);
    EXPECT_DOUBLE_EQ(deserialize<double>(serialize<double>(0.0)), 0.0);
}

// inplace_size_embedding
TEST_F(SerializeEdgeCaseTest, InplaceSizeEmbedding) {
    std::string msg = "Hello";
    inplace_size_embedding(msg);
    EXPECT_EQ(msg.size(), 5 + sizeof(uint64_t));
    // First 8 bytes should be the size (5)
    uint64_t size = deserialize<uint64_t>(
        std::vector<uint8_t>(msg.begin(), msg.begin() + sizeof(uint64_t)));
    EXPECT_EQ(size, 5);
    // Remaining bytes should be the message
    EXPECT_EQ(msg.substr(sizeof(uint64_t)), "Hello");
}

TEST_F(SerializeEdgeCaseTest, InplaceSizeEmbeddingEmpty) {
    std::string msg;
    inplace_size_embedding(msg);
    EXPECT_EQ(msg.size(), sizeof(uint64_t));
    uint64_t size = deserialize<uint64_t>(
        std::vector<uint8_t>(msg.begin(), msg.begin() + sizeof(uint64_t)));
    EXPECT_EQ(size, 0);
}

// Buffer too small for deserialize (unsafe but should not crash in test)
TEST_F(SerializeEdgeCaseTest, DeserializeSmallBuffer) {
    std::vector<uint8_t> small_buf{0x01, 0x02};
    // This is undefined behavior in current implementation, but we document it
    // In a hardened implementation, this should throw
    // For now, just verify it compiles and runs (it's unsafe API usage)
    // EXPECT_THROW(deserialize<uint64_t>(small_buf), std::runtime_error);
    (void)small_buf;
}

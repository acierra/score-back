#include "cloud_point_rpc/rpc_coder.hpp"
#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace score;

class Base64EdgeCaseTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestRPC");
    }
    Base64RPCCoder coder;
};

// Empty input
TEST_F(Base64EdgeCaseTest, EmptyEncode) {
    std::vector<char> empty;
    auto encoded = coder.encode(empty);
    EXPECT_TRUE(encoded.empty());
}

TEST_F(Base64EdgeCaseTest, EmptyDecode) {
    std::string empty;
    auto decoded = coder.decode(empty);
    EXPECT_TRUE(decoded.empty());
}

// 1 byte input
TEST_F(Base64EdgeCaseTest, OneByteEncode) {
    std::vector<char> data{'A'};
    auto encoded = coder.encode(data);
    EXPECT_EQ(encoded, "QQ==");
}

TEST_F(Base64EdgeCaseTest, OneByteRoundTrip) {
    std::vector<char> data{'A'};
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// 2 bytes input
TEST_F(Base64EdgeCaseTest, TwoBytesEncode) {
    std::vector<char> data{'A', 'B'};
    auto encoded = coder.encode(data);
    EXPECT_EQ(encoded, "QUI=");
}

TEST_F(Base64EdgeCaseTest, TwoBytesRoundTrip) {
    std::vector<char> data{'A', 'B'};
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// 3 bytes input (no padding)
TEST_F(Base64EdgeCaseTest, ThreeBytesEncode) {
    std::vector<char> data{'A', 'B', 'C'};
    auto encoded = coder.encode(data);
    EXPECT_EQ(encoded, "QUJD");
}

TEST_F(Base64EdgeCaseTest, ThreeBytesRoundTrip) {
    std::vector<char> data{'A', 'B', 'C'};
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// Standard test vectors
TEST_F(Base64EdgeCaseTest, StandardVectors) {
    struct TestCase {
        std::vector<char> input;
        std::string expected;
    };
    std::vector<TestCase> cases = {
        {{'f'}, "Zg=="},
        {{'f', 'o'}, "Zm8="},
        {{'f', 'o', 'o'}, "Zm9v"},
        {{'f', 'o', 'o', 'b'}, "Zm9vYg=="},
        {{'f', 'o', 'o', 'b', 'a'}, "Zm9vYmE="},
        {{'f', 'o', 'o', 'b', 'a', 'r'}, "Zm9vYmFy"},
    };
    for (const auto &tc : cases) {
        auto encoded = coder.encode(tc.input);
        EXPECT_EQ(encoded, tc.expected);
        auto decoded = coder.decode(encoded);
        EXPECT_EQ(decoded, tc.input);
    }
}

// Binary data with null bytes
TEST_F(Base64EdgeCaseTest, BinaryWithNullBytes) {
    std::vector<char> data{'H', 'e', 'l', 'l', 'o', '\0',
                           'W', 'o', 'r', 'l', 'd'};
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// All byte values 0-255
TEST_F(Base64EdgeCaseTest, AllByteValues) {
    std::vector<char> data(256);
    for (int i = 0; i < 256; ++i) {
        data[i] = static_cast<char>(i);
    }
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// Repeated patterns
TEST_F(Base64EdgeCaseTest, RepeatedPattern) {
    std::vector<char> data(1024, 'A');
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// Invalid base64 characters
TEST_F(Base64EdgeCaseTest, InvalidCharactersDecode) {
    // base64_decode should handle invalid chars gracefully or fail
    std::string invalid = "!!!";
    auto decoded        = coder.decode(invalid);
    // libbase64 may return empty or partial result; just verify no crash
    (void)decoded;
}

TEST_F(Base64EdgeCaseTest, MixedValidInvalid) {
    std::string mixed = "QU!!JD";
    auto decoded      = coder.decode(mixed);
    (void)decoded; // no crash expected
}

// Padding edge cases
TEST_F(Base64EdgeCaseTest, NoPaddingDecode) {
    std::string no_pad = "QUJD"; // "ABC" without explicit padding
    auto decoded       = coder.decode(no_pad);
    std::vector<char> expected{'A', 'B', 'C'};
    EXPECT_EQ(decoded, expected);
}

TEST_F(Base64EdgeCaseTest, ExtraPadding) {
    std::string extra_pad = "QQ===";
    auto decoded          = coder.decode(extra_pad);
    (void)decoded; // no crash expected
}

// Large input
TEST_F(Base64EdgeCaseTest, LargeInputRoundTrip) {
    std::vector<char> data(100000, 'x');
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// Very large input (1MB)
TEST_F(Base64EdgeCaseTest, OneMegabyteRoundTrip) {
    std::vector<char> data(1024 * 1024);
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<char>(i % 256);
    }
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
}

// Whitespace in encoded string
TEST_F(Base64EdgeCaseTest, WhitespaceInEncoded) {
    std::string with_space = "Q U J D";
    auto decoded           = coder.decode(with_space);
    (void)decoded; // libbase64 behavior varies; ensure no crash
}

// Non-ASCII characters in input (UTF-8)
TEST_F(Base64EdgeCaseTest, Utf8RoundTrip) {
    std::string utf8 = "Hello, 世界! 🌍";
    std::vector<char> data(utf8.begin(), utf8.end());
    auto encoded = coder.encode(data);
    auto decoded = coder.decode(encoded);
    EXPECT_EQ(decoded, data);
    std::string decoded_str(decoded.begin(), decoded.end());
    EXPECT_EQ(decoded_str, utf8);
}

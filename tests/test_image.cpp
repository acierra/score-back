//
// Created by vptyp on 12.03.2026.
//
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cloud_point/imageFactory.h>
#include <cloud_point/matrixFactory.h>

class ImageTest : public ::testing::Test {
  protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(ImageTest, DefaultConstructor) {
    score::Image image;
    cv::Mat mat = image.get();
    EXPECT_TRUE(mat.empty());
}

TEST_F(ImageTest, ConstructorWithMat) {
    cv::Mat input = cv::Mat::zeros(5, 5, CV_8UC1);
    score::Image image(input);
    cv::Mat output = image.get();
    EXPECT_EQ(output.rows, 5);
    EXPECT_EQ(output.cols, 5);
    EXPECT_EQ(output.type(), CV_8UC1);
}

TEST_F(ImageTest, PixelTypeMapping) {
    EXPECT_EQ(score::ImageFactory::pixelType(score::ImageRPC::Type::BGR),
              CV_8UC3);
    EXPECT_EQ(score::ImageFactory::pixelType(score::ImageRPC::Type::RGBA),
              CV_8UC4);
    EXPECT_EQ(score::ImageFactory::pixelType(score::ImageRPC::Type::DEPTH),
              CV_64FC1);
    EXPECT_THROW(score::ImageFactory::pixelType(score::ImageRPC::Type::UNKNOWN),
                 std::runtime_error);
}

TEST_F(ImageTest, CreateBGR) {
    score::ImageRPC rpc;
    rpc.width  = 10;
    rpc.height = 20;
    rpc.type   = score::ImageRPC::Type::BGR;
    rpc.data.resize(rpc.width * rpc.height * 3, 128);

    score::Image image = score::ImageFactory::create(rpc);
    cv::Mat mat        = image.get();

    EXPECT_EQ(mat.rows, 20);
    EXPECT_EQ(mat.cols, 10);
    EXPECT_EQ(mat.type(), CV_8UC3);
    EXPECT_EQ(mat.at<cv::Vec3b>(0, 0)[0], 128);
}

TEST_F(ImageTest, CreateRGBA) {
    score::ImageRPC rpc;
    rpc.width  = 15;
    rpc.height = 25;
    rpc.type   = score::ImageRPC::Type::RGBA;
    rpc.data.resize(rpc.width * rpc.height * 4, 255);

    score::Image image = score::ImageFactory::create(rpc);
    cv::Mat mat        = image.get();

    EXPECT_EQ(mat.rows, 25);
    EXPECT_EQ(mat.cols, 15);
    EXPECT_EQ(mat.type(), CV_8UC4);
    EXPECT_EQ(mat.at<cv::Vec4b>(0, 0)[0], 255);
}

TEST_F(ImageTest, CreateDepth) {
    score::ImageRPC rpc;
    rpc.width  = 5;
    rpc.height = 10;
    rpc.type   = score::ImageRPC::Type::DEPTH;
    rpc.data.resize(rpc.width * rpc.height * sizeof(double));
    auto *dataPtr = reinterpret_cast<double *>(rpc.data.data());
    for (int i = 0; i < 50; ++i)
        dataPtr[i] = static_cast<double>(i);

    score::Image image = score::ImageFactory::create(rpc);
    cv::Mat mat        = image.get();

    EXPECT_EQ(mat.rows, 10);
    EXPECT_EQ(mat.cols, 5);
    EXPECT_EQ(mat.type(), CV_64FC1);
    EXPECT_DOUBLE_EQ(mat.at<double>(0, 0), 0.0);
    // row 9, col 4 → flat index 9*5+4 = 49
    EXPECT_DOUBLE_EQ(mat.at<double>(9, 4), 49.0);
}

TEST_F(ImageTest, CreateOwnsDataAfterSourceDestroyed) {
    // Use asymmetric size (width=4, height=2) with known BGR values
    constexpr int kWidth  = 4;
    constexpr int kHeight = 2;
    constexpr int kBytes  = kWidth * kHeight * 3;

    score::Image image;
    {
        score::ImageRPC rpc;
        rpc.width  = kWidth;
        rpc.height = kHeight;
        rpc.type   = score::ImageRPC::Type::BGR;
        rpc.data.resize(kBytes);
        for (int i = 0; i < kBytes; ++i)
            rpc.data[i] = static_cast<unsigned char>(i);
        image = score::ImageFactory::create(rpc);
        // rpc goes out of scope here; image must own its data
    }

    cv::Mat mat = image.get();
    EXPECT_EQ(mat.rows, kHeight);
    EXPECT_EQ(mat.cols, kWidth);
    EXPECT_EQ(mat.type(), CV_8UC3);

    // Verify pixel values match the original source bytes
    for (int r = 0; r < kHeight; ++r) {
        for (int c = 0; c < kWidth; ++c) {
            const int flat = (r * kWidth + c) * 3;
            const auto px  = mat.at<cv::Vec3b>(r, c);
            EXPECT_EQ(px[0], static_cast<unsigned char>(flat));
            EXPECT_EQ(px[1], static_cast<unsigned char>(flat + 1));
            EXPECT_EQ(px[2], static_cast<unsigned char>(flat + 2));
        }
    }
}

TEST_F(ImageTest, CreateThrowsOnWrongDataSize) {
    score::ImageRPC rpc;
    rpc.width  = 4;
    rpc.height = 2;
    rpc.type   = score::ImageRPC::Type::BGR;
    // Intentionally wrong size (1 byte short)
    rpc.data.resize(4 * 2 * 3 - 1, 0);
    EXPECT_THROW(score::ImageFactory::create(rpc), std::runtime_error);
}

TEST_F(ImageTest, CameraMatrixCreateOwnsDataAfterVectorDestroyed) {
    cv::Mat mat;
    {
        std::vector<double> vals = {1, 2, 3, 4, 5, 6, 7, 8, 9};
        mat = score::CameraMatrixFactory::create<3, 3>(vals);
        // vals goes out of scope here; mat must own its data
    }

    ASSERT_EQ(mat.rows, 3);
    ASSERT_EQ(mat.cols, 3);
    for (int i = 0; i < 9; ++i)
        EXPECT_DOUBLE_EQ(mat.at<double>(i / 3, i % 3),
                         static_cast<double>(i + 1));
}

TEST_F(ImageTest, CameraMatrixCreateThrowsOnWrongSize) {
    std::vector<double> vals(8, 0.0); // 8 elements, need 9 for 3x3
    EXPECT_THROW((score::CameraMatrixFactory::create<3, 3>(vals)),
                 std::runtime_error);
}

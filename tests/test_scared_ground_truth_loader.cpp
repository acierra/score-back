#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include "cloud_point/scared_ground_truth_loader.hpp"

namespace {

class ScaredGroundTruthLoaderTest : public ::testing::Test {
  protected:
    void SetUp() override {
        const auto suffix =
            std::chrono::steady_clock::now().time_since_epoch().count();
        directory_ = std::filesystem::temp_directory_path() /
                     ("scared-ground-truth-" + std::to_string(suffix));
        std::filesystem::create_directories(directory_);
    }

    void TearDown() override { std::filesystem::remove_all(directory_); }

    void write_obj(const std::string &contents) const {
        std::ofstream output(directory_ / "point_cloud.obj");
        output << contents;
    }

    std::filesystem::path directory_;
};

} // namespace

TEST_F(ScaredGroundTruthLoaderTest, PreservesOrderNaNsAndConvertsToMetres) {
    write_obj("# two by two point map\n"
              "v 1000 2000 3000\n"
              "v nan nan nan\n"
              "v -500 0 250\n"
              "v 1 2 3\n"
              "f 1 3 4\n");

    const score::ScaredGroundTruthLoader loader(directory_.string(),
                                                cv::Size(2, 2));
    const cv::Mat &points = loader.point_map();

    EXPECT_EQ(points.type(), CV_32FC3);
    EXPECT_EQ(points.size(), cv::Size(2, 2));
    EXPECT_EQ(loader.valid_point_count(), 3u);

    const cv::Vec3f first = points.at<cv::Vec3f>(0, 0);
    EXPECT_FLOAT_EQ(first[0], 1.0f);
    EXPECT_FLOAT_EQ(first[1], 2.0f);
    EXPECT_FLOAT_EQ(first[2], 3.0f);

    const cv::Vec3f missing = points.at<cv::Vec3f>(0, 1);
    EXPECT_TRUE(std::isnan(missing[0]));
    EXPECT_TRUE(std::isnan(missing[1]));
    EXPECT_TRUE(std::isnan(missing[2]));

    const cv::Vec3f third = points.at<cv::Vec3f>(1, 0);
    EXPECT_FLOAT_EQ(third[0], -0.5f);
    EXPECT_FLOAT_EQ(third[1], 0.0f);
    EXPECT_FLOAT_EQ(third[2], 0.25f);
}

TEST_F(ScaredGroundTruthLoaderTest, RejectsIncorrectVertexCount) {
    write_obj("v 1 2 3\n");
    EXPECT_THROW(
        score::ScaredGroundTruthLoader(directory_.string(), cv::Size(2, 2)),
        std::runtime_error);
}

TEST_F(ScaredGroundTruthLoaderTest, RejectsInvalidScale) {
    write_obj("v 1 2 3\n");
    EXPECT_THROW(score::ScaredGroundTruthLoader(directory_.string(),
                                                cv::Size(1, 1), 0.0f),
                 std::invalid_argument);
}

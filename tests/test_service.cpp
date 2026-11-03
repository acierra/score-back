#include "cloud_point_rpc/service.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace score;

class ServiceEdgeCaseTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestRPC");
    }
};

// Default constructor (no data)
TEST_F(ServiceEdgeCaseTest, DefaultConstructorFallbacks) {
    Service service;

    auto intrinsic = service.get_intrinsic_params();
    EXPECT_EQ(intrinsic.size(), 9);
    EXPECT_EQ(intrinsic[0], 1.0);
    EXPECT_EQ(intrinsic[4], 1.0);
    EXPECT_EQ(intrinsic[8], 1.0);

    auto extrinsic = service.get_extrinsic_params();
    EXPECT_EQ(extrinsic.size(), 16);
    EXPECT_EQ(extrinsic[0], 1.0);
    EXPECT_EQ(extrinsic[5], 1.0);
    EXPECT_EQ(extrinsic[10], 1.0);
    EXPECT_EQ(extrinsic[15], 1.0);

    auto cloud = service.get_cloud_point();
    EXPECT_EQ(cloud.size(), 3);
    EXPECT_EQ(cloud[0], std::vector<double>({0.1, 0.2, 0.3}));
}

// Empty TestData explicitly
TEST_F(ServiceEdgeCaseTest, ExplicitEmptyData) {
    TestData empty_data;
    Service service(empty_data);

    auto intrinsic = service.get_intrinsic_params();
    EXPECT_EQ(intrinsic.size(), 9);

    auto extrinsic = service.get_extrinsic_params();
    EXPECT_EQ(extrinsic.size(), 16);

    auto cloud = service.get_cloud_point();
    EXPECT_EQ(cloud.size(), 3);
}

// Custom intrinsic params
TEST_F(ServiceEdgeCaseTest, CustomIntrinsicParams) {
    TestData data;
    data.intrinsic_params = {100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0};
    Service service(data);

    auto intrinsic = service.get_intrinsic_params();
    EXPECT_EQ(intrinsic, data.intrinsic_params);
}

// Custom extrinsic params
TEST_F(ServiceEdgeCaseTest, CustomExtrinsicParams) {
    TestData data;
    data.extrinsic_params = {1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1};
    Service service(data);

    auto extrinsic = service.get_extrinsic_params();
    EXPECT_EQ(extrinsic, data.extrinsic_params);
}

// Custom cloud point
TEST_F(ServiceEdgeCaseTest, CustomCloudPoint) {
    TestData data;
    data.cloud_point = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    Service service(data);

    auto cloud = service.get_cloud_point();
    EXPECT_EQ(cloud.size(), 2);
    EXPECT_EQ(cloud[0], std::vector<double>({1.0, 2.0, 3.0}));
    EXPECT_EQ(cloud[1], std::vector<double>({4.0, 5.0, 6.0}));
}

// Large point cloud
TEST_F(ServiceEdgeCaseTest, LargePointCloud) {
    TestData data;
    for (int i = 0; i < 10000; ++i) {
        data.cloud_point.push_back({static_cast<double>(i),
                                    static_cast<double>(i + 1),
                                    static_cast<double>(i + 2)});
    }
    Service service(data);

    auto cloud = service.get_cloud_point();
    EXPECT_EQ(cloud.size(), 10000);
    EXPECT_EQ(cloud[9999], std::vector<double>({9999.0, 10000.0, 10001.0}));
}

// Single point cloud
TEST_F(ServiceEdgeCaseTest, SinglePointCloud) {
    TestData data;
    data.cloud_point = {{0.0, 0.0, 0.0}};
    Service service(data);

    auto cloud = service.get_cloud_point();
    EXPECT_EQ(cloud.size(), 1);
    EXPECT_EQ(cloud[0], std::vector<double>({0.0, 0.0, 0.0}));
}

// Negative values
TEST_F(ServiceEdgeCaseTest, NegativeValues) {
    TestData data;
    data.intrinsic_params = {-100.0, 0.0, -50.0, 0.0, -100.0,
                             -50.0,  0.0, 0.0,   -1.0};
    Service service(data);

    auto intrinsic = service.get_intrinsic_params();
    EXPECT_EQ(intrinsic[0], -100.0);
    EXPECT_EQ(intrinsic[8], -1.0);
}

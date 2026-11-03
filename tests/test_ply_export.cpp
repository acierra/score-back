#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>

#include "cloud_point/cloud_point_client.hpp"

using namespace score;

namespace {

/// 3x3 organised cloud on a plane z = 1 m with 1 cm pixel spacing.
PointCloud make_grid_cloud() {
    PointCloud cloud;
    cloud.width  = 3;
    cloud.height = 3;
    cloud.data.resize(27);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const size_t i    = static_cast<size_t>(r * 3 + c) * 3;
            cloud.data[i]     = 0.01f * static_cast<float>(c);
            cloud.data[i + 1] = 0.01f * static_cast<float>(r);
            cloud.data[i + 2] = 1.0f;
        }
    }
    return cloud;
}

void set_nan(PointCloud &cloud, int r, int c) {
    const size_t i  = static_cast<size_t>(r * cloud.width + c) * 3;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    cloud.data[i] = cloud.data[i + 1] = cloud.data[i + 2] = nan;
}

struct TempFile {
    std::string path{"test_ply_export.ply"};
    ~TempFile() { std::remove(path.c_str()); }
    std::string read() const {
        std::ifstream in(path);
        std::stringstream ss;
        ss << in.rdbuf();
        return ss.str();
    }
};

} // namespace

TEST(PlyExportTest, PointsOnlyHasNoFaceElement) {
    TempFile file;
    const auto faces =
        write_ply(make_grid_cloud(), file.path, PlyOptions{false, 0.05f});
    EXPECT_EQ(faces, 0u);
    const auto text = file.read();
    EXPECT_NE(text.find("element vertex 9\n"), std::string::npos);
    EXPECT_EQ(text.find("element face"), std::string::npos);
}

TEST(PlyExportTest, FullGridProducesTwoTrianglesPerCell) {
    TempFile file;
    const auto faces =
        write_ply(make_grid_cloud(), file.path, PlyOptions{true, 0.05f, false});
    EXPECT_EQ(faces, 8u); // 2x2 cells * 2 triangles
    EXPECT_NE(file.read().find("format ascii 1.0\n"), std::string::npos);
    const auto text = file.read();
    EXPECT_NE(text.find("element face 8\n"), std::string::npos);
    EXPECT_NE(text.find("property list uchar int vertex_indices\n"),
              std::string::npos);
    // First cell, first triangle: (0,0) -> (1,0) -> (1,1) = indices 0,3,4
    EXPECT_NE(text.find("\n3 0 3 4\n"), std::string::npos);
}

TEST(PlyExportTest, MissingCornerKeepsSingleTriangle) {
    auto cloud = make_grid_cloud();
    set_nan(cloud, 0, 0); // top-left cell has 3 valid corners
    TempFile file;
    const auto faces =
        write_ply(cloud, file.path, PlyOptions{true, 0.05f, false});
    EXPECT_EQ(faces, 7u);
    const auto text = file.read();
    EXPECT_NE(text.find("element vertex 8\n"), std::string::npos);
}

TEST(PlyExportTest, DepthJumpBreaksMesh) {
    auto cloud = make_grid_cloud();
    // Push the centre column 20 cm away: every triangle touching it now has
    // an edge longer than 5% of its mean depth and must be dropped.
    for (int r = 0; r < 3; ++r) {
        cloud.data[static_cast<size_t>(r * 3 + 1) * 3 + 2] = 1.2f;
    }
    TempFile file;
    EXPECT_EQ(write_ply(cloud, file.path), 0u);
}

TEST(PlyExportTest, StrideAveragesBlocks) {
    // 3x3 grid with stride 2 -> 2x2 blocks: one full 2x2 block, two 2x1 /
    // 1x2 edge blocks and the single corner pixel -> 4 vertices, 2 faces.
    TempFile file;
    const auto faces = write_ply(make_grid_cloud(), file.path,
                                 PlyOptions{true, 0.05f, false, 2});
    EXPECT_EQ(faces, 2u);
    const auto text = file.read();
    EXPECT_NE(text.find("element vertex 4\n"), std::string::npos);
    // First vertex is the mean of pixels (0,0),(0,1),(1,0),(1,1).
    EXPECT_NE(text.find("\n0.005 0.005 1\n"), std::string::npos);
    // Last vertex is the lone corner pixel (2,2).
    EXPECT_NE(text.find("\n0.02 0.02 1\n"), std::string::npos);
}

TEST(PlyExportTest, StrideDropsMostlyInvalidBlocks) {
    auto cloud = make_grid_cloud();
    // Invalidate 3 of the 4 pixels of the top-left 2x2 block.
    set_nan(cloud, 0, 0);
    set_nan(cloud, 0, 1);
    set_nan(cloud, 1, 0);
    TempFile file;
    write_ply(cloud, file.path, PlyOptions{true, 0.05f, false, 2});
    EXPECT_NE(file.read().find("element vertex 3\n"), std::string::npos);
}

TEST(PlyExportTest, InvalidStrideThrows) {
    TempFile file;
    EXPECT_THROW(std::ignore = write_ply(make_grid_cloud(), file.path,
                                         PlyOptions{true, 0.05f, false, 0}),
                 std::invalid_argument);
}

TEST(PlyExportTest, BinaryLayoutMatchesHeader) {
    TempFile file;
    const auto faces = write_ply(make_grid_cloud(), file.path); // binary
    ASSERT_EQ(faces, 8u);
    const auto text = file.read();
    EXPECT_NE(text.find("format binary_little_endian 1.0\n"),
              std::string::npos);
    const auto header_end = text.find("end_header\n") + 11;
    ASSERT_NE(header_end, std::string::npos + 11);
    // 9 vertices * 12 bytes + 8 faces * (1 + 12) bytes
    EXPECT_EQ(text.size() - header_end, 9u * 12u + 8u * 13u);
    // First vertex is (0, 0, 1)
    float v[3];
    std::memcpy(v, text.data() + header_end, sizeof(v));
    EXPECT_FLOAT_EQ(v[0], 0.0f);
    EXPECT_FLOAT_EQ(v[1], 0.0f);
    EXPECT_FLOAT_EQ(v[2], 1.0f);
    // First face: count byte 3 then indices 0,3,4
    const char *f = text.data() + header_end + 9 * 12;
    EXPECT_EQ(static_cast<unsigned char>(f[0]), 3u);
    int idx[3];
    std::memcpy(idx, f + 1, sizeof(idx));
    EXPECT_EQ(idx[0], 0);
    EXPECT_EQ(idx[1], 3);
    EXPECT_EQ(idx[2], 4);
}

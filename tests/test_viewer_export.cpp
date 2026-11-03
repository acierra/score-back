// Tests for the CPV1 viewer frame export (encode_viewer_frame /
// write_viewer_frame_js).  The base64 payload is decoded with an independent
// decoder here so that the test checks the wire format the JS viewer reads.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/core.hpp>

#include "cloud_point/cloud_point_client.hpp"
#include "cloud_point/point_cloud_evaluator.hpp"

using namespace score;

namespace {

constexpr float kUnitM = 2e-5f; // 0.02 mm per int16 step

/// Standard-alphabet, padded base64 decoder (independent of the writer).
std::vector<uint8_t> base64_decode_std(const std::string &text) {
    const auto value = [](char ch) -> int {
        if (ch >= 'A' && ch <= 'Z')
            return ch - 'A';
        if (ch >= 'a' && ch <= 'z')
            return ch - 'a' + 26;
        if (ch >= '0' && ch <= '9')
            return ch - '0' + 52;
        if (ch == '+')
            return 62;
        if (ch == '/')
            return 63;
        return -1;
    };
    std::vector<uint8_t> out;
    uint32_t acc = 0;
    int bits     = 0;
    for (const char ch : text) {
        if (ch == '=')
            break;
        const int v = value(ch);
        if (v < 0)
            throw std::runtime_error(std::string("bad base64 char: ") + ch);
        acc = (acc << 6) | static_cast<uint32_t>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<uint8_t>((acc >> bits) & 0xffu));
        }
    }
    return out;
}

/// Parsed CPV1 payload.
struct Frame {
    int width{0}, height{0}, stride{0};
    bool has_gt{false}, has_rgb{false};
    std::vector<int16_t> xyz, gt;
    std::vector<uint8_t> rgb;
};

Frame parse_cpv1(const std::string &b64) {
    // Base64 must be padded, single-line.
    EXPECT_EQ(b64.size() % 4u, 0u);
    EXPECT_EQ(b64.find('\n'), std::string::npos);
    const auto bytes = base64_decode_std(b64);
    EXPECT_GE(bytes.size(), 12u);
    EXPECT_EQ(std::memcmp(bytes.data(), "CPV1", 4), 0);
    const auto u16 = [&](size_t off) {
        return static_cast<int>(bytes[off]) |
               (static_cast<int>(bytes[off + 1]) << 8);
    };
    Frame f;
    f.width   = u16(4);
    f.height  = u16(6);
    f.stride  = u16(8);
    f.has_gt  = (bytes[10] & 1u) != 0;
    f.has_rgb = (bytes[10] & 2u) != 0;
    EXPECT_EQ(bytes[11], 0u) << "reserved byte";
    const size_t n      = static_cast<size_t>(f.width) * f.height * 3u;
    size_t off          = 12;
    const auto read_i16 = [&](std::vector<int16_t> &dst) {
        dst.resize(n);
        for (size_t i = 0; i < n; ++i, off += 2) {
            const uint16_t u = static_cast<uint16_t>(
                bytes[off] | (static_cast<uint16_t>(bytes[off + 1]) << 8));
            int16_t v;
            std::memcpy(&v, &u, 2);
            dst[i] = v;
        }
    };
    read_i16(f.xyz);
    if (f.has_gt)
        read_i16(f.gt);
    if (f.has_rgb) {
        f.rgb.assign(bytes.begin() + static_cast<ptrdiff_t>(off),
                     bytes.begin() + static_cast<ptrdiff_t>(off + n));
        off += n;
    }
    EXPECT_EQ(off, bytes.size()) << "payload has trailing bytes";
    return f;
}

/// Organised w x h cloud where pixel (r, c) = (c * 1 mm, r * 1 mm, 50 mm
/// + (r * w + c) * 0.001 mm) so every sample is distinct.
PointCloud make_cloud(int w, int h) {
    PointCloud cloud;
    cloud.width  = w;
    cloud.height = h;
    cloud.data.resize(static_cast<size_t>(w) * h * 3u);
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const size_t i    = static_cast<size_t>(r * w + c) * 3u;
            cloud.data[i]     = 0.001f * static_cast<float>(c);
            cloud.data[i + 1] = 0.001f * static_cast<float>(r);
            cloud.data[i + 2] = 0.05f + 1e-6f * static_cast<float>(r * w + c);
        }
    }
    return cloud;
}

void set_nan(PointCloud &cloud, int r, int c) {
    const size_t i  = static_cast<size_t>(r * cloud.width + c) * 3u;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    cloud.data[i] = cloud.data[i + 1] = cloud.data[i + 2] = nan;
}

/// Ground truth = cloud shifted by a per-pixel offset (metres) on x and z.
cv::Mat make_gt(const PointCloud &cloud, float dx_m, float dz_m) {
    cv::Mat gt(cloud.height, cloud.width, CV_32FC3);
    for (int r = 0; r < cloud.height; ++r) {
        for (int c = 0; c < cloud.width; ++c) {
            const size_t i = static_cast<size_t>(r * cloud.width + c) * 3u;
            gt.at<cv::Vec3f>(r, c) =
                cv::Vec3f(cloud.data[i] + dx_m, cloud.data[i + 1],
                          cloud.data[i + 2] + dz_m);
        }
    }
    return gt;
}

cv::Mat make_bgr(int w, int h) {
    cv::Mat bgr(h, w, CV_8UC3);
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            // B, G, R distinct per pixel
            bgr.at<cv::Vec3b>(r, c) =
                cv::Vec3b(static_cast<uchar>(c), static_cast<uchar>(r + 100),
                          static_cast<uchar>(r * w + c + 200));
        }
    }
    return bgr;
}

struct TempFile {
    std::string path{"test_viewer_export.js"};
    ~TempFile() { std::remove(path.c_str()); }
    std::string read() const {
        std::ifstream in(path);
        std::stringstream ss;
        ss << in.rdbuf();
        return ss.str();
    }
};

} // namespace

TEST(ViewerExportTest, HeaderFieldsAndFlags) {
    const auto cloud = make_cloud(9, 6);
    const auto frame = parse_cpv1(
        encode_viewer_frame(cloud, nullptr, nullptr, ViewerFrameOptions{2}));
    EXPECT_EQ(frame.width, 4); // floor(9 / 2)
    EXPECT_EQ(frame.height, 3);
    EXPECT_EQ(frame.stride, 2);
    EXPECT_FALSE(frame.has_gt);
    EXPECT_FALSE(frame.has_rgb);
    EXPECT_EQ(frame.xyz.size(), 4u * 3u * 3u);
    EXPECT_TRUE(frame.gt.empty());
    EXPECT_TRUE(frame.rgb.empty());
}

TEST(ViewerExportTest, DefaultStrideIsFour) {
    const auto cloud = make_cloud(16, 8);
    const auto frame = parse_cpv1(encode_viewer_frame(cloud, nullptr, nullptr));
    EXPECT_EQ(frame.stride, 4);
    EXPECT_EQ(frame.width, 4);
    EXPECT_EQ(frame.height, 2);
}

TEST(ViewerExportTest, QuantisationWithin0p01mm) {
    const auto cloud = make_cloud(5, 4);
    const auto frame = parse_cpv1(
        encode_viewer_frame(cloud, nullptr, nullptr, ViewerFrameOptions{1}));
    ASSERT_EQ(frame.xyz.size(), cloud.data.size());
    for (size_t i = 0; i < cloud.data.size(); ++i) {
        const double decoded = static_cast<double>(frame.xyz[i]) * kUnitM;
        EXPECT_NEAR(decoded, cloud.data[i], 0.01e-3) << "index " << i;
    }
}

TEST(ViewerExportTest, StrideSamplesRawPixels) {
    const auto cloud = make_cloud(8, 8);
    const int stride = 3;
    const auto frame = parse_cpv1(encode_viewer_frame(
        cloud, nullptr, nullptr, ViewerFrameOptions{stride}));
    ASSERT_EQ(frame.width, 2);
    ASSERT_EQ(frame.height, 2);
    for (int r = 0; r < frame.height; ++r) {
        for (int c = 0; c < frame.width; ++c) {
            const size_t src =
                static_cast<size_t>(r * stride * cloud.width + c * stride) * 3u;
            const size_t dst = static_cast<size_t>(r * frame.width + c) * 3u;
            for (int k = 0; k < 3; ++k) {
                EXPECT_EQ(frame.xyz[dst + k],
                          static_cast<int16_t>(
                              std::lround(cloud.data[src + k] / kUnitM)))
                    << "sample (" << r << "," << c << ") comp " << k;
            }
        }
    }
}

TEST(ViewerExportTest, InvalidPixelIsInt16Min) {
    auto cloud = make_cloud(4, 4);
    set_nan(cloud, 1, 2);
    // A single non-finite component also invalidates the whole pixel.
    cloud.data[static_cast<size_t>(3 * 4 + 0) * 3u + 2] =
        std::numeric_limits<float>::infinity();
    const auto frame = parse_cpv1(
        encode_viewer_frame(cloud, nullptr, nullptr, ViewerFrameOptions{1}));
    const auto at = [&](int r, int c, int k) {
        return frame.xyz[static_cast<size_t>(r * 4 + c) * 3u + k];
    };
    for (int k = 0; k < 3; ++k) {
        EXPECT_EQ(at(1, 2, k), std::numeric_limits<int16_t>::min());
        EXPECT_EQ(at(3, 0, k), std::numeric_limits<int16_t>::min());
    }
    EXPECT_NE(at(0, 0, 2), std::numeric_limits<int16_t>::min());
}

TEST(ViewerExportTest, OutOfRangeIsClamped) {
    auto cloud       = make_cloud(1, 1);
    cloud.data[2]    = 1.0f; // 1 m = 50000 units > 32767
    cloud.data[0]    = -1.0f;
    const auto frame = parse_cpv1(
        encode_viewer_frame(cloud, nullptr, nullptr, ViewerFrameOptions{1}));
    EXPECT_EQ(frame.xyz[2], 32767);
    EXPECT_EQ(frame.xyz[0], -32767); // never INT16_MIN for a finite value
}

TEST(ViewerExportTest, GroundTruthChannel) {
    auto cloud                = make_cloud(6, 4);
    cv::Mat gt                = make_gt(cloud, 0.0005f, -0.001f);
    const float nan           = std::numeric_limits<float>::quiet_NaN();
    gt.at<cv::Vec3f>(2, 4)    = cv::Vec3f(nan, nan, nan);
    gt.at<cv::Vec3f>(0, 2)[1] = nan; // partially unknown
    const auto frame          = parse_cpv1(
        encode_viewer_frame(cloud, &gt, nullptr, ViewerFrameOptions{2}));
    EXPECT_TRUE(frame.has_gt);
    EXPECT_FALSE(frame.has_rgb);
    ASSERT_EQ(frame.gt.size(), 3u * 2u * 3u);
    const auto at = [&](int r, int c, int k) {
        return frame.gt[static_cast<size_t>(r * frame.width + c) * 3u + k];
    };
    // Sample (1, 2) = pixel (2, 4): all-NaN -> invalid.
    for (int k = 0; k < 3; ++k)
        EXPECT_EQ(at(1, 2, k), std::numeric_limits<int16_t>::min());
    // Sample (0, 1) = pixel (0, 2): one NaN component -> invalid.
    for (int k = 0; k < 3; ++k)
        EXPECT_EQ(at(0, 1, k), std::numeric_limits<int16_t>::min());
    // Sample (0, 0) = pixel (0, 0): shifted cloud value.
    EXPECT_NEAR(at(0, 0, 0) * kUnitM, cloud.data[0] + 0.0005f, 0.01e-3);
    EXPECT_NEAR(at(0, 0, 2) * kUnitM, cloud.data[2] - 0.001f, 0.01e-3);
}

TEST(ViewerExportTest, RgbChannelIsStoredAsRgbFromBgr) {
    const auto cloud = make_cloud(6, 4);
    const auto bgr   = make_bgr(6, 4);
    const auto frame = parse_cpv1(
        encode_viewer_frame(cloud, nullptr, &bgr, ViewerFrameOptions{2}));
    EXPECT_FALSE(frame.has_gt);
    EXPECT_TRUE(frame.has_rgb);
    ASSERT_EQ(frame.rgb.size(), 3u * 2u * 3u);
    for (int r = 0; r < frame.height; ++r) {
        for (int c = 0; c < frame.width; ++c) {
            const auto px  = bgr.at<cv::Vec3b>(r * 2, c * 2);
            const size_t i = static_cast<size_t>(r * frame.width + c) * 3u;
            EXPECT_EQ(frame.rgb[i], px[2]) << "R at " << r << "," << c;
            EXPECT_EQ(frame.rgb[i + 1], px[1]) << "G at " << r << "," << c;
            EXPECT_EQ(frame.rgb[i + 2], px[0]) << "B at " << r << "," << c;
        }
    }
}

TEST(ViewerExportTest, AllChannelsLayout) {
    const auto cloud = make_cloud(8, 8);
    const auto gt    = make_gt(cloud, 0.001f, 0.0f);
    const auto bgr   = make_bgr(8, 8);
    const auto b64   = encode_viewer_frame(cloud, &gt, &bgr);
    const auto bytes = base64_decode_std(b64);
    // stride 4 -> 2x2 samples: 12 + 4*6 + 4*6 + 4*3 bytes
    EXPECT_EQ(bytes.size(), 12u + 24u + 24u + 12u);
    EXPECT_EQ(bytes[10], 3u);
    const auto frame = parse_cpv1(b64);
    EXPECT_TRUE(frame.has_gt);
    EXPECT_TRUE(frame.has_rgb);
}

TEST(ViewerExportTest, EmptyOptionalInputsMeanAbsent) {
    const auto cloud = make_cloud(4, 4);
    const cv::Mat empty;
    const auto frame = parse_cpv1(
        encode_viewer_frame(cloud, &empty, &empty, ViewerFrameOptions{1}));
    EXPECT_FALSE(frame.has_gt);
    EXPECT_FALSE(frame.has_rgb);
}

TEST(ViewerExportTest, RoundTripMaeMatchesDirectMae) {
    // Cloud with holes and a GT with holes; MAE recomputed from the decoded
    // int16 samples must match PointCloudEvaluator on the same pixels.
    auto cloud = make_cloud(40, 30);
    for (int r = 0; r < cloud.height; r += 7)
        for (int c = 0; c < cloud.width; c += 5)
            set_nan(cloud, r, c);
    // Error grows across the image (0 .. ~1.7 mm) to exercise many values.
    cv::Mat gt(cloud.height, cloud.width, CV_32FC3);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int r = 0; r < cloud.height; ++r) {
        for (int c = 0; c < cloud.width; ++c) {
            const size_t i = static_cast<size_t>(r * cloud.width + c) * 3u;
            if ((r * 3 + c) % 11 == 0) {
                gt.at<cv::Vec3f>(r, c) = cv::Vec3f(nan, nan, nan);
                continue;
            }
            gt.at<cv::Vec3f>(r, c) = cv::Vec3f(
                cloud.data[i] + 1e-3f * static_cast<float>(c) / 40.f,
                cloud.data[i + 1] - 0.3e-3f,
                cloud.data[i + 2] + 1.3e-3f * static_cast<float>(r) / 30.f);
        }
    }
    const auto direct = PointCloudEvaluator::evaluate(cloud, gt);
    ASSERT_GT(direct.matched_points, 0u);

    const auto frame = parse_cpv1(
        encode_viewer_frame(cloud, &gt, nullptr, ViewerFrameOptions{1}));
    ASSERT_TRUE(frame.has_gt);
    double sum        = 0.0;
    size_t matched    = 0;
    const int16_t bad = std::numeric_limits<int16_t>::min();
    for (size_t i = 0; i + 2 < frame.xyz.size(); i += 3) {
        if (frame.xyz[i] == bad || frame.gt[i] == bad)
            continue;
        double e2 = 0.0;
        for (size_t k = 0; k < 3; ++k) {
            const double d =
                (static_cast<double>(frame.xyz[i + k]) - frame.gt[i + k]) *
                kUnitM;
            e2 += d * d;
        }
        sum += std::sqrt(e2);
        ++matched;
    }
    EXPECT_EQ(matched, direct.matched_points);
    EXPECT_NEAR(sum / static_cast<double>(matched), direct.mae_3d_m, 0.02e-3);
}

TEST(ViewerExportTest, InvalidInputsThrow) {
    const auto cloud = make_cloud(8, 8);
    EXPECT_THROW(std::ignore = encode_viewer_frame(cloud, nullptr, nullptr,
                                                   ViewerFrameOptions{0}),
                 std::invalid_argument);
    // Stride larger than the cloud leaves no samples.
    EXPECT_THROW(std::ignore = encode_viewer_frame(cloud, nullptr, nullptr,
                                                   ViewerFrameOptions{9}),
                 std::invalid_argument);
    // Wrong GT size / type.
    const cv::Mat gt_small(4, 4, CV_32FC3, cv::Scalar(0));
    EXPECT_THROW(std::ignore = encode_viewer_frame(cloud, &gt_small, nullptr),
                 std::invalid_argument);
    const cv::Mat gt_type(8, 8, CV_64FC3, cv::Scalar(0));
    EXPECT_THROW(std::ignore = encode_viewer_frame(cloud, &gt_type, nullptr),
                 std::invalid_argument);
    // Wrong RGB size / type.
    const cv::Mat rgb_small(8, 4, CV_8UC3, cv::Scalar(0));
    EXPECT_THROW(std::ignore = encode_viewer_frame(cloud, nullptr, &rgb_small),
                 std::invalid_argument);
    const cv::Mat rgb_gray(8, 8, CV_8UC1, cv::Scalar(0));
    EXPECT_THROW(std::ignore = encode_viewer_frame(cloud, nullptr, &rgb_gray),
                 std::invalid_argument);
    // Malformed cloud.
    PointCloud bad = cloud;
    bad.data.pop_back();
    EXPECT_THROW(std::ignore = encode_viewer_frame(bad, nullptr, nullptr,
                                                   ViewerFrameOptions{1}),
                 std::invalid_argument);
    PointCloud empty;
    EXPECT_THROW(std::ignore = encode_viewer_frame(empty, nullptr, nullptr),
                 std::invalid_argument);
}

TEST(ViewerExportTest, WriteFrameJsWrapsPayload) {
    TempFile file;
    write_viewer_frame_js(file.path, "kf-1.a_b", "QUJD");
    EXPECT_EQ(file.read(), "CloudViewer.addFrame(\"kf-1.a_b\", \"QUJD\");\n");
}

TEST(ViewerExportTest, WriteFrameJsRejectsBadId) {
    TempFile file;
    EXPECT_THROW(write_viewer_frame_js(file.path, "", "QUJD"),
                 std::invalid_argument);
    EXPECT_THROW(write_viewer_frame_js(file.path, "kf 1", "QUJD"),
                 std::invalid_argument);
    EXPECT_THROW(write_viewer_frame_js(file.path, "kf\"1", "QUJD"),
                 std::invalid_argument);
    EXPECT_THROW(write_viewer_frame_js(file.path, "a/b", "QUJD"),
                 std::invalid_argument);
}

TEST(ViewerExportTest, WriteFrameJsUnwritablePathThrows) {
    EXPECT_THROW(
        write_viewer_frame_js("/nonexistent-dir/x/kf.js", "kf", "QUJD"),
        std::runtime_error);
}

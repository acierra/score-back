// E2E tests for CloudPointClient: in-process TcpServer + RpcServer with a
// synthetic scene providing known ground-truth depth.
//
// Calibration: fx=fy=800, cx=320, cy=240, 640×480, baseline=0.06 m,
//              R=I, T=[-0.06,0,0].  Disparity = 32 px → z = 1.5 m.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "cloud_point/cloud_point_client.hpp"
#include "cloud_point_rpc/cli.hpp"
#include "cloud_point_rpc/rpc_dto.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/tcp_server.hpp"

using namespace score;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Shared synthetic calibration helpers
// ---------------------------------------------------------------------------
namespace {

constexpr double kFx       = 800.0;
constexpr double kFy       = 800.0;
constexpr double kCx       = 320.0;
constexpr double kCy       = 240.0;
constexpr int kWidth       = 640;
constexpr int kHeight      = 480;
constexpr double kBaseline = 0.06;
constexpr double kTx       = -kBaseline; // OpenCV T convention

/// Standard stereo calibration DTO (parallel rig, no distortion).
StereoCalibrationRPC make_stereo_calib() {
    StereoCalibrationRPC c;
    c.width              = kWidth;
    c.height             = kHeight;
    c.left.camera_matrix = {kFx, 0, kCx, 0, kFy, kCy, 0, 0, 1};
    c.left.dist_coeffs   = {0, 0, 0, 0, 0};
    c.right              = c.left;
    c.rotation           = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    c.translation        = {kTx, 0.0, 0.0};
    return c;
}

/// Synthetic image pair: random left (seed 42), right = left shifted +32 px.
/// Convention: right(x, y) = left(x+d, y) → disparity d = 32 for positive
/// depth (matches service.cpp and OpenCV SGBM sign convention with Tx < 0).
ImagePairRPC make_image_pair(int disparity = 32) {
    // Generate textured left grayscale image with fixed seed.
    cv::theRNG().state = 42;
    cv::Mat left_gray(kHeight, kWidth, CV_8UC1);
    cv::randu(left_gray, 0, 256);

    // Build right by shifting: right(x) = left(x+d), 0 at right border.
    cv::Mat right_gray = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth - disparity; ++x) {
            right_gray.at<uchar>(y, x) = left_gray.at<uchar>(y, x + disparity);
        }
    }

    // Convert to BGR for the wire protocol.
    cv::Mat left_bgr, right_bgr;
    cv::cvtColor(left_gray, left_bgr, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right_gray, right_bgr, cv::COLOR_GRAY2BGR);

    auto mat_to_rpc = [](const cv::Mat &img, int w, int h) -> ImageRPC {
        ImageRPC rpc;
        rpc.width       = w;
        rpc.height      = h;
        rpc.type        = ImageRPC::Type::BGR;
        const size_t sz = static_cast<size_t>(w) * h * 3;
        rpc.data.resize(sz);
        std::memcpy(rpc.data.data(), img.data, sz);
        return rpc;
    };

    ImagePairRPC pair;
    pair.frame = 0;
    pair.left  = mat_to_rpc(left_bgr, kWidth, kHeight);
    pair.right = mat_to_rpc(right_bgr, kWidth, kHeight);
    return pair;
}

} // namespace

// ---------------------------------------------------------------------------
// Test fixture: in-process TcpServer + RpcServer on port 9201
// ---------------------------------------------------------------------------

class CloudPointClientTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestCloudPointClient");
    }
    void TearDown() override {
        if (server_) {
            server_->stop();
        }
    }

    /// Start an in-process server with the given RpcServer and wait for it.
    void start_server(int port, std::unique_ptr<RpcServer> rpc) {
        rpc_server_ = std::move(rpc);
        server_     = std::make_unique<TcpServer>(
            "127.0.0.1", port, [this](const std::string &req) {
                return rpc_server_->process(req);
            });
        server_->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::unique_ptr<RpcServer> rpc_server_;
    std::unique_ptr<TcpServer> server_;
};

// ---------------------------------------------------------------------------
// Test 1: connect succeeds; compute_cloud() returns a cloud with median z ≈ 1.5
// m
// ---------------------------------------------------------------------------

TEST_F(CloudPointClientTest, PreprocessingMatchesDirectPipelineAndKeepsColour) {
    constexpr int port = 9229;
    auto rpc           = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [](const json &) -> json {
        json j;
        to_json(j, make_image_pair());
        return j;
    });
    start_server(port, std::move(rpc));

    // Non-identity processing must actually reach the matcher, while the
    // viewer continues to receive the original rectified colour image.
    PreprocessingStep step;
    step.method     = "clahe";
    step.statistics = "independent";
    const PreprocessingConfig config{true, {step}};
    CloudPointClient client("127.0.0.1", port, StereoAlgorithmType::CPU, {},
                            128, {}, config);
    client.connect();
    auto actual = client.compute_frame();
    ASSERT_TRUE(actual.has_value()) << actual.error().message;

    const auto pair = make_image_pair();
    cv::Mat left(kHeight, kWidth, CV_8UC3,
                 const_cast<uint8_t *>(pair.left.data.data()));
    cv::Mat right(kHeight, kWidth, CV_8UC3,
                  const_cast<uint8_t *>(pair.right.data.data()));
    cv::Mat gray_l, gray_r;
    cv::cvtColor(left, gray_l, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, gray_r, cv::COLOR_BGR2GRAY);
    StereoRectifier rectifier(
        StereoRectifier::Calibration::from_rpc(make_stereo_calib()));
    const auto [l, r] = rectifier.rectify(gray_l, gray_r);
    StereoPreprocessorChain chain(config);
    const auto processed = chain.process(l, r);
    CpuStereoMatcher matcher(0, 128);
    PointCloudBuilder builder(rectifier.q());
    const auto expected =
        builder.build(matcher.compute(processed.left, processed.right));
    ASSERT_EQ(actual->cloud.data.size(), expected.data.size());
    for (std::size_t i = 0; i < expected.data.size(); ++i) {
        if (std::isnan(expected.data[i]))
            ASSERT_TRUE(std::isnan(actual->cloud.data[i]));
        else
            ASSERT_FLOAT_EQ(actual->cloud.data[i], expected.data[i]);
    }
    EXPECT_EQ(cv::norm(actual->rectified_left_bgr,
                       rectifier.rectify(left, right).first, cv::NORM_INF),
              0);
}

TEST_F(CloudPointClientTest, ExplicitNonePreservesExistingCloudExactly) {
    constexpr int port = 9230;
    auto rpc           = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [](const json &) -> json {
        json j;
        to_json(j, make_image_pair());
        return j;
    });
    start_server(port, std::move(rpc));
    CloudPointClient old_client("127.0.0.1", port, StereoAlgorithmType::CPU);
    old_client.connect();
    const auto old = old_client.compute_cloud();
    ASSERT_TRUE(old.has_value());
    PreprocessingConfig config{true, {PreprocessingStep{}}};
    CloudPointClient none_client("127.0.0.1", port, StereoAlgorithmType::CPU,
                                 {}, 128, {}, config);
    none_client.connect();
    const auto none = none_client.compute_cloud();
    ASSERT_TRUE(none.has_value());
    ASSERT_EQ(none->data.size(), old->data.size());
    EXPECT_EQ(std::memcmp(none->data.data(), old->data.data(),
                          old->data.size() * sizeof(float)),
              0);
}

TEST_F(CloudPointClientTest, SpecularMaskExcludesUnreliablePoints) {
    constexpr int port = 9231;
    auto rpc           = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [](const json &) -> json {
        json j;
        to_json(j, make_image_pair());
        return j;
    });
    start_server(port, std::move(rpc));
    PreprocessingStep step;
    step.method               = "specular_mask";
    step.brightness_threshold = 1;
    step.dilation_radius =
        15; // intentionally masks this entire synthetic scene
    CloudPointClient client("127.0.0.1", port, StereoAlgorithmType::CPU, {},
                            128, {}, PreprocessingConfig{true, {step}});
    client.connect();
    const auto result = client.compute_cloud();
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_TRUE(result->valid_points().empty());
}

TEST_F(CloudPointClientTest, ComputeCloudReturnsCorrectDepth) {
    constexpr int kPort = 9201;

    // Build RPC server with known stereo calibration and image pair.
    auto rpc = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [](const json &) -> json {
        json j;
        to_json(j, make_image_pair(32));
        return j;
    });
    start_server(kPort, std::move(rpc));

    CloudPointClient client("127.0.0.1", kPort, StereoAlgorithmType::CPU);
    ASSERT_NO_THROW(client.connect());
    ASSERT_TRUE(client.connected());

    auto result = client.compute_cloud();
    ASSERT_TRUE(result.has_value())
        << "compute_cloud returned Error: " << result.error().message;

    const auto &cloud = *result;
    EXPECT_EQ(cloud.width, kWidth);
    EXPECT_EQ(cloud.height, kHeight);
    EXPECT_FALSE(cloud.valid_points().empty())
        << "Expected at least some valid points";

    // Collect z values in the central region (avoid SGBM borders).
    // Horizontal: [150, 490) to skip left border (SGBM invalid) and
    //             right border where right image has no data (last 32 cols).
    // Vertical: [50, 430).
    constexpr int kXMin = 150;
    constexpr int kXMax = 490;
    constexpr int kYMin = 50;
    constexpr int kYMax = 430;

    std::vector<float> z_vals;
    z_vals.reserve(static_cast<size_t>((kXMax - kXMin) * (kYMax - kYMin)));

    for (int y = kYMin; y < kYMax; ++y) {
        for (int x = kXMin; x < kXMax; ++x) {
            const float z =
                cloud.data[static_cast<size_t>(y * kWidth + x) * 3 + 2];
            if (!std::isnan(z))
                z_vals.push_back(z);
        }
    }

    ASSERT_FALSE(z_vals.empty()) << "No valid points in central region";

    // Compute median z.
    const auto mid = z_vals.begin() + static_cast<ptrdiff_t>(z_vals.size() / 2);
    std::nth_element(z_vals.begin(), mid, z_vals.end());
    const float median_z        = *mid;

    constexpr float kExpectedZ  = static_cast<float>(kFx * kBaseline / 32.0);
    constexpr float kToleranceZ = kExpectedZ * 0.05f; // 5%

    EXPECT_NEAR(median_z, kExpectedZ, kToleranceZ)
        << "Median z in central region should be ~" << kExpectedZ
        << " m (expected=" << kExpectedZ << ", got=" << median_z << ")";
}

// ---------------------------------------------------------------------------
// Test 2: server returns garbage image data → compute_cloud() returns Error
// ---------------------------------------------------------------------------

TEST_F(CloudPointClientTest, GarbageImageDataReturnsError) {
    constexpr int kPort = 9202;

    auto rpc            = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    // Return an image pair whose data size doesn't match width*height*channels.
    rpc->register_method("get-image-pair", [](const json &) -> json {
        ImagePairRPC bad_pair;
        bad_pair.frame       = 0;
        bad_pair.left.width  = kWidth;
        bad_pair.left.height = kHeight;
        bad_pair.left.type   = ImageRPC::Type::BGR;
        bad_pair.left.data = {0x01, 0x02}; // wrong size: 2 bytes, not 640*480*3
        bad_pair.right     = bad_pair.left;
        json j;
        to_json(j, bad_pair);
        return j;
    });
    start_server(kPort, std::move(rpc));

    CloudPointClient client("127.0.0.1", kPort, StereoAlgorithmType::CPU);
    ASSERT_NO_THROW(client.connect());

    auto result = client.compute_cloud();
    EXPECT_FALSE(result.has_value())
        << "Expected Error for garbage image data, got a cloud instead";
    if (!result.has_value()) {
        EXPECT_FALSE(result.error().message.empty());
    }
}

// ---------------------------------------------------------------------------
// Test 3: connect() to a closed port throws
// ---------------------------------------------------------------------------

TEST_F(CloudPointClientTest, ConnectToClosedPortThrows) {
    // Port 9203 has no server running.
    CloudPointClient client("127.0.0.1", 9203, StereoAlgorithmType::CPU);
    EXPECT_THROW(client.connect(), std::runtime_error);
    EXPECT_FALSE(client.connected());
}

// ---------------------------------------------------------------------------
// Test 3b: compute_frame() returns the rectified left colour image on the
// same grid as the cloud, and the same cloud as compute_cloud()
// ---------------------------------------------------------------------------

TEST_F(CloudPointClientTest, ComputeFrameReturnsColourImageOfCloudSize) {
    constexpr int kPort = 9205;

    auto rpc            = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [](const json &) -> json {
        json j;
        to_json(j, make_image_pair(32));
        return j;
    });
    start_server(kPort, std::move(rpc));

    CloudPointClient client("127.0.0.1", kPort, StereoAlgorithmType::CPU);
    ASSERT_NO_THROW(client.connect());

    auto frame = client.compute_frame();
    ASSERT_TRUE(frame.has_value())
        << "compute_frame returned Error: " << frame.error().message;

    EXPECT_EQ(frame->cloud.width, kWidth);
    EXPECT_EQ(frame->cloud.height, kHeight);
    EXPECT_EQ(frame->rectified_left_bgr.type(), CV_8UC3);
    EXPECT_EQ(frame->rectified_left_bgr.cols, frame->cloud.width);
    EXPECT_EQ(frame->rectified_left_bgr.rows, frame->cloud.height);
    // Identity rectification of a textured image must not be blank.
    EXPECT_GT(cv::countNonZero(frame->rectified_left_bgr.reshape(1)), 0);

    // compute_cloud() is the same pipeline without the colour image.
    auto cloud = client.compute_cloud();
    ASSERT_TRUE(cloud.has_value());
    ASSERT_EQ(cloud->data.size(), frame->cloud.data.size());
    for (size_t i = 0; i < cloud->data.size(); ++i) {
        const float a = cloud->data[i], b = frame->cloud.data[i];
        ASSERT_TRUE((std::isnan(a) && std::isnan(b)) || a == b)
            << "cloud differs at " << i;
    }
}

// ---------------------------------------------------------------------------
// Test 3c: "Same reconstruction everywhere" — the client's cloud is
// byte-identical to a PointCloudBuilder driven directly with the same Q,
// matcher settings, disparity offset and outlier filter on the same pair.
// ---------------------------------------------------------------------------

namespace {

/// NaN-aware element-wise equality of two clouds.
void expect_identical_clouds(const PointCloud &a, const PointCloud &b) {
    ASSERT_EQ(a.width, b.width);
    ASSERT_EQ(a.height, b.height);
    ASSERT_EQ(a.data.size(), b.data.size());
    for (size_t i = 0; i < a.data.size(); ++i) {
        const float x = a.data[i], y = b.data[i];
        ASSERT_TRUE((std::isnan(x) && std::isnan(y)) || x == y)
            << "cloud differs at " << i << ": " << x << " vs " << y;
    }
}

bool clouds_equal(const PointCloud &a, const PointCloud &b) {
    if (a.width != b.width || a.height != b.height ||
        a.data.size() != b.data.size())
        return false;
    for (size_t i = 0; i < a.data.size(); ++i) {
        const float x = a.data[i], y = b.data[i];
        if (!((std::isnan(x) && std::isnan(y)) || x == y))
            return false;
    }
    return true;
}

} // namespace

TEST_F(CloudPointClientTest, ClientMatchesDirectBuilderWithOffsetAndFilter) {
    constexpr int kPort           = 9210;
    constexpr int kNumDisparities = 128;

    auto rpc                      = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [](const json &) -> json {
        json j;
        to_json(j, make_image_pair(32));
        return j;
    });
    start_server(kPort, std::move(rpc));

    // Non-default reconstruction settings: a fractional offset and the
    // outlier filter switched on with a tight window.
    PointCloudBuilder::Options opts{0.5f, 5.0f};
    opts.disparity_offset_px            = 0.5;
    opts.outlier_filter.enabled         = true;
    opts.outlier_filter.window          = 5;
    opts.outlier_filter.abs_threshold_m = 0.002;
    opts.outlier_filter.rel_threshold   = 0.01;
    opts.outlier_filter.min_neighbours  = 8;
    CpuStereoMatcher::Params matcher_params;

    // Through the client.
    CloudPointClient client("127.0.0.1", kPort, StereoAlgorithmType::CPU, opts,
                            kNumDisparities, matcher_params);
    ASSERT_NO_THROW(client.connect());
    auto via_client = client.compute_cloud();
    ASSERT_TRUE(via_client.has_value())
        << "compute_cloud returned Error: " << via_client.error().message;

    // Directly: same calibration, matcher and builder options on the same
    // pair (the benchmark's path).
    const auto pair    = make_image_pair(32);
    const auto to_gray = [](const ImageRPC &img) {
        cv::Mat bgr(img.height, img.width, CV_8UC3,
                    const_cast<uint8_t *>(img.data.data()));
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        return gray;
    };
    const StereoRectifier rectifier(
        StereoRectifier::Calibration::from_rpc(make_stereo_calib()));
    const auto [rect_left, rect_right] =
        rectifier.rectify(to_gray(pair.left), to_gray(pair.right));
    const auto matcher = StereoMatcherFactory::create(
        StereoAlgorithmType::CPU, kNumDisparities, matcher_params);
    const cv::Mat disparity = matcher->compute(rect_left, rect_right);
    const PointCloudBuilder builder(rectifier.q(), opts);
    const PointCloud direct = builder.build(disparity);

    expect_identical_clouds(*via_client, direct);
    EXPECT_FALSE(direct.valid_points().empty());

    // The settings really reached the client's builder: with the offset off
    // and the filter disabled the same disparity gives a different cloud.
    PointCloudBuilder::Options plain{0.5f, 5.0f};
    const PointCloud untuned =
        PointCloudBuilder(rectifier.q(), plain).build(disparity);
    EXPECT_FALSE(clouds_equal(*via_client, untuned))
        << "offset / outlier filter had no effect on the client's cloud";
}

// ---------------------------------------------------------------------------
// Test 4: CLI smoke — option 4 against the mock server
// ---------------------------------------------------------------------------

TEST_F(CloudPointClientTest, CliOption4ComputeCloud) {
    constexpr int kPort = 9204;

    auto rpc            = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [](const json &) -> json {
        json j;
        to_json(j, make_image_pair(32));
        return j;
    });
    // The CLI also uses the legacy RPC methods; register stubs so it can
    // connect.
    rpc->register_method("get-intrinsic-params",
                         [](const json &) { return std::vector<double>{1.0}; });
    rpc->register_method("get-extrinsic-params",
                         [](const json &) { return std::vector<double>{1.0}; });
    rpc->register_method("get-cloud-point", [](const json &) {
        return std::vector<std::vector<double>>{};
    });
    start_server(kPort, std::move(rpc));

    std::istringstream input("4\n0\n");
    std::ostringstream output;

    const int rc = run_cli(input, output, "127.0.0.1", kPort);
    EXPECT_EQ(rc, 0);

    const std::string out = output.str();
    // Expect either a point count line ("valid_pts=") or an error message.
    EXPECT_THAT(out, ::testing::AnyOf(::testing::HasSubstr("valid_pts="),
                                      ::testing::HasSubstr("Error")))
        << "CLI output was:\n"
        << out;
}

// ---------------------------------------------------------------------------
// CLI option 6: compute-cloud + open viewer
// ---------------------------------------------------------------------------
namespace {

namespace fs = std::filesystem;

/// Registers the stereo methods plus the legacy stubs the CLI needs.
std::unique_ptr<RpcServer> make_stereo_rpc(bool garbage_images = false) {
    auto rpc = std::make_unique<RpcServer>();
    rpc->register_method("get-stereo-calibration", [](const json &) -> json {
        json j;
        to_json(j, make_stereo_calib());
        return j;
    });
    rpc->register_method("get-image-pair", [garbage_images](const json &) {
        json j;
        if (garbage_images) {
            ImagePairRPC bad;
            bad.left.width  = kWidth;
            bad.left.height = kHeight;
            bad.left.type   = ImageRPC::Type::BGR;
            bad.left.data   = {0x01, 0x02};
            bad.right       = bad.left;
            to_json(j, bad);
        } else {
            to_json(j, make_image_pair(32));
        }
        return j;
    });
    rpc->register_method("get-intrinsic-params",
                         [](const json &) { return std::vector<double>{1.0}; });
    return rpc;
}

/// Fresh scratch directory with a minimal viewer template inside.
struct ViewerScratch {
    fs::path root;
    fs::path template_dir;
    fs::path site_dir;
    explicit ViewerScratch(const std::string &name)
        : root(fs::temp_directory_path() /
               ("cprpc_viewer_" + name + "_" + std::to_string(::getpid()))),
          template_dir(root / "template"), site_dir(root / "site") {
        fs::remove_all(root);
        fs::create_directories(template_dir / "vendor");
        std::ofstream(template_dir / "index.html") << "<html>viewer</html>\n";
        std::ofstream(template_dir / "viewer.js") << "// viewer\n";
        std::ofstream(template_dir / "vendor" / "three.min.js") << "// three\n";
    }
    ~ViewerScratch() { fs::remove_all(root); }

    CliStereoOptions options(const std::string &open_command) const {
        CliStereoOptions o;
        o.use_gpu             = false;
        o.viewer.output_dir   = site_dir.string();
        o.viewer.template_dir = template_dir.string();
        o.viewer.open_command = open_command;
        o.viewer.stride       = 8;
        return o;
    }

    std::vector<std::string> frame_files() const {
        std::vector<std::string> names;
        if (!fs::is_directory(site_dir / "frames"))
            return names;
        for (const auto &e : fs::directory_iterator(site_dir / "frames")) {
            const auto n = e.path().filename().string();
            if (n.rfind("cli_", 0) == 0 && e.path().extension() == ".js")
                names.push_back(n);
        }
        return names;
    }
};

std::string read_file(const fs::path &p) {
    std::ifstream in(p);
    return std::string(std::istreambuf_iterator<char>(in), {});
}

std::size_t count_occurrences(const std::string &text, const std::string &s) {
    std::size_t n = 0;
    for (auto pos = text.find(s); pos != std::string::npos;
         pos      = text.find(s, pos + s.size()))
        ++n;
    return n;
}

} // namespace

TEST_F(CloudPointClientTest, CliOption6WritesSiteAndAccumulatesFrames) {
    constexpr int kPort = 9206;
    start_server(kPort, make_stereo_rpc());
    ViewerScratch scratch("accumulate");
    const auto opts = scratch.options("true");

    {
        std::istringstream input("6\n0\n");
        std::ostringstream output;
        ASSERT_EQ(run_cli(input, output, "127.0.0.1", kPort, opts), 0);
        const std::string out = output.str();
        EXPECT_THAT(out, ::testing::HasSubstr("valid_pts=")) << out;
        EXPECT_THAT(out, ::testing::HasSubstr("Saved viewer frame cli_"));
        EXPECT_THAT(
            out, ::testing::HasSubstr(
                     "Viewer: " + (scratch.site_dir / "index.html").string()));
        EXPECT_THAT(out, ::testing::Not(
                             ::testing::HasSubstr("could not open a browser")));
    }
    // Template copied (including subdirectories) and one frame written.
    EXPECT_TRUE(fs::exists(scratch.site_dir / "index.html"));
    EXPECT_TRUE(fs::exists(scratch.site_dir / "viewer.js"));
    EXPECT_TRUE(fs::exists(scratch.site_dir / "vendor" / "three.min.js"));
    ASSERT_EQ(scratch.frame_files().size(), 1u);
    {
        const auto manifest =
            read_file(scratch.site_dir / "frames" / "manifest.js");
        EXPECT_THAT(manifest,
                    ::testing::StartsWith("CloudViewer.setManifest("));
        EXPECT_THAT(manifest, ::testing::EndsWith(");\n"));
        EXPECT_THAT(manifest,
                    ::testing::HasSubstr("cloudPointRPC CLI captures"));
        EXPECT_EQ(count_occurrences(manifest, "\"id\": \"cli_"), 1u);
        // The JSON inside the call parses and carries the no-GT metrics.
        const auto body = manifest.substr(
            std::string("CloudViewer.setManifest(").size(),
            manifest.size() - std::string("CloudViewer.setManifest(").size() -
                std::string(");\n").size());
        const auto j = json::parse(body);
        ASSERT_EQ(j["frames"].size(), 1u);
        EXPECT_FALSE(j["frames"][0]["has_gt"].get<bool>());
        EXPECT_THAT(j["frames"][0]["label"].get<std::string>(),
                    ::testing::HasSubstr("127.0.0.1:9206"));
        const auto &m = j["frames"][0]["metrics"];
        EXPECT_GT(m["valid_fraction"].get<double>(), 0.0);
        EXPECT_NEAR(m["z_median_m"].get<double>(), kFx * kBaseline / 32.0, 0.1);
        EXPECT_LE(m["z_p05_m"].get<double>(), m["z_p95_m"].get<double>());
        const auto frame_js =
            read_file(scratch.site_dir / "frames" /
                      (j["frames"][0]["id"].get<std::string>() + ".js"));
        EXPECT_THAT(frame_js, ::testing::StartsWith("CloudViewer.addFrame("));
    }

    // A second capture (separate CLI run) extends the same site.
    {
        std::istringstream input("6\n0\n");
        std::ostringstream output;
        ASSERT_EQ(run_cli(input, output, "127.0.0.1", kPort, opts), 0);
        EXPECT_THAT(output.str(), ::testing::HasSubstr("2 frames in site"));
    }
    EXPECT_EQ(scratch.frame_files().size(), 2u);
    const auto manifest =
        read_file(scratch.site_dir / "frames" / "manifest.js");
    EXPECT_EQ(count_occurrences(manifest, "\"id\": \"cli_"), 2u);
}

TEST_F(CloudPointClientTest, CliOption6ReportsUnopenableBrowserAndContinues) {
    constexpr int kPort = 9207;
    start_server(kPort, make_stereo_rpc());
    ViewerScratch scratch("noopen");

    // Spawn failure: the command does not exist.
    {
        std::istringstream input("6\n0\n");
        std::ostringstream output;
        EXPECT_EQ(run_cli(input, output, "127.0.0.1", kPort,
                          scratch.options("/nonexistent/cprpc-open")),
                  0);
        const std::string out   = output.str();
        const std::string index = (scratch.site_dir / "index.html").string();
        EXPECT_THAT(out, ::testing::HasSubstr("Viewer: " + index)) << out;
        EXPECT_THAT(out, ::testing::HasSubstr("Viewer written to " + index +
                                              " (could not open a browser: "));
        EXPECT_THAT(out, ::testing::Not(::testing::HasSubstr("Error")));
        EXPECT_EQ(count_occurrences(out, "Select an option:"), 2u);
    }
    // Command exists but fails immediately.
    {
        std::istringstream input("6\n0\n");
        std::ostringstream output;
        EXPECT_EQ(run_cli(input, output, "127.0.0.1", kPort,
                          scratch.options("false")),
                  0);
        EXPECT_THAT(output.str(),
                    ::testing::HasSubstr("could not open a browser: false "
                                         "exited with status 1"));
    }
    // Empty command: site written, nothing launched, no note about failure.
    {
        std::istringstream input("6\n0\n");
        std::ostringstream output;
        EXPECT_EQ(
            run_cli(input, output, "127.0.0.1", kPort, scratch.options("")), 0);
        EXPECT_THAT(output.str(), ::testing::HasSubstr("Viewer: "));
        EXPECT_THAT(output.str(), ::testing::Not(::testing::HasSubstr(
                                      "could not open a browser")));
    }
    EXPECT_EQ(scratch.frame_files().size(), 3u);
}

TEST_F(CloudPointClientTest, CliOption6ComputeErrorWritesNothing) {
    constexpr int kPort = 9208;
    start_server(kPort, make_stereo_rpc(/*garbage_images=*/true));
    ViewerScratch scratch("compute_error");

    std::istringstream input("6\n0\n");
    std::ostringstream output;
    EXPECT_EQ(
        run_cli(input, output, "127.0.0.1", kPort, scratch.options("true")), 0);
    const std::string out = output.str();
    EXPECT_THAT(out, ::testing::HasSubstr("Error: ")) << out;
    EXPECT_THAT(out, ::testing::Not(::testing::HasSubstr("Viewer:")));
    EXPECT_FALSE(fs::exists(scratch.site_dir));
    EXPECT_EQ(count_occurrences(out, "Select an option:"), 2u);
}

TEST_F(CloudPointClientTest, CliOption6MissingTemplateIsReported) {
    constexpr int kPort = 9209;
    start_server(kPort, make_stereo_rpc());
    ViewerScratch scratch("no_template");
    auto opts                = scratch.options("true");
    opts.viewer.template_dir = (scratch.root / "missing").string();

    std::istringstream input("6\n0\n");
    std::ostringstream output;
    EXPECT_EQ(run_cli(input, output, "127.0.0.1", kPort, opts), 0);
    const std::string out = output.str();
    EXPECT_THAT(out, ::testing::HasSubstr("valid_pts=")) << out;
    EXPECT_THAT(out,
                ::testing::HasSubstr("Error: viewer template not found at " +
                                     (scratch.root / "missing").string() +
                                     " (set viewer.template_dir)"));
    EXPECT_FALSE(fs::exists(scratch.site_dir));
    EXPECT_EQ(count_occurrences(out, "Select an option:"), 2u);
}

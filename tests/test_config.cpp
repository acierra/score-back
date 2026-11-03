#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <tuple>

#include "cloud_point_rpc/config.hpp"

namespace {

struct TempConfig {
    std::string path;
    explicit TempConfig(const std::string &yaml)
        : path("test_config_" +
               std::to_string(reinterpret_cast<uintptr_t>(this)) + ".yaml") {
        std::ofstream(path) << yaml;
    }
    ~TempConfig() { std::remove(path.c_str()); }
};

} // namespace

TEST(ConfigLoaderTest, CloudPointSectionDefaultsWhenAbsent) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_EQ(c.cloud_point.algorithm, "gpu");
    EXPECT_EQ(c.cloud_point.num_disparities, 128);
    EXPECT_DOUBLE_EQ(c.cloud_point.min_depth_m, 0.01);
    EXPECT_DOUBLE_EQ(c.cloud_point.max_depth_m, 10.0);
}

TEST(ConfigLoaderTest, CloudPointSectionIsParsed) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "cloud_point:\n  algorithm: cpu\n  num_disparities: 160\n"
                   "  min_depth_m: 0.02\n  max_depth_m: 0.3\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_EQ(c.cloud_point.algorithm, "cpu");
    EXPECT_EQ(c.cloud_point.num_disparities, 160);
    EXPECT_DOUBLE_EQ(c.cloud_point.min_depth_m, 0.02);
    EXPECT_DOUBLE_EQ(c.cloud_point.max_depth_m, 0.3);
}

TEST(ConfigLoaderTest, CloudPointExportAndFilterKeys) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "cloud_point:\n  ply_stride: 4\n  wls_filter: true\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_EQ(c.cloud_point.ply_stride, 4);
    EXPECT_TRUE(c.cloud_point.wls_filter);
    TempConfig bad("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "cloud_point:\n  ply_stride: 0\n");
    EXPECT_THROW(std::ignore = score::ConfigLoader::load(bad.path),
                 std::runtime_error);
}

TEST(ConfigLoaderTest, CloudPointPartialSectionKeepsDefaults) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "cloud_point:\n  max_depth_m: 0.5\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_EQ(c.cloud_point.algorithm, "gpu");
    EXPECT_EQ(c.cloud_point.num_disparities, 128);
    EXPECT_DOUBLE_EQ(c.cloud_point.max_depth_m, 0.5);
}

TEST(ConfigLoaderTest, CloudPointRejectsBadAlgorithm) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "cloud_point:\n  algorithm: fpga\n");
    EXPECT_THROW(std::ignore = score::ConfigLoader::load(cfg.path),
                 std::runtime_error);
}

TEST(ConfigLoaderTest, CloudPointRejectsInvertedDepthRange) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "cloud_point:\n  min_depth_m: 1.0\n  max_depth_m: 0.5\n");
    EXPECT_THROW(std::ignore = score::ConfigLoader::load(cfg.path),
                 std::runtime_error);
}

TEST(ConfigLoaderTest, ViewerSectionDefaultsWhenAbsent) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_EQ(c.viewer.output_dir, "out/viewer-cli");
    EXPECT_TRUE(c.viewer.template_dir.empty());
#ifdef __APPLE__
    EXPECT_EQ(c.viewer.open_command, "open");
#else
    EXPECT_EQ(c.viewer.open_command, "xdg-open");
#endif
    EXPECT_EQ(c.viewer.stride, 4);
}

TEST(ConfigLoaderTest, ViewerSectionOverrides) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "viewer:\n  output_dir: /tmp/site\n"
                   "  template_dir: /opt/tpl\n  open_command: firefox\n"
                   "  stride: 2\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_EQ(c.viewer.output_dir, "/tmp/site");
    EXPECT_EQ(c.viewer.template_dir, "/opt/tpl");
    EXPECT_EQ(c.viewer.open_command, "firefox");
    EXPECT_EQ(c.viewer.stride, 2);
}

TEST(ConfigLoaderTest, ViewerPartialSectionKeepsDefaults) {
    TempConfig cfg("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                   "viewer:\n  open_command: \"\"\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_EQ(c.viewer.output_dir, "out/viewer-cli");
    EXPECT_TRUE(c.viewer.open_command.empty());
    EXPECT_EQ(c.viewer.stride, 4);
}

TEST(ConfigLoaderTest, ViewerRejectsBadValues) {
    TempConfig bad_stride("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                          "viewer:\n  stride: 0\n");
    EXPECT_THROW(std::ignore = score::ConfigLoader::load(bad_stride.path),
                 std::runtime_error);
    TempConfig bad_dir("server:\n  ip: \"127.0.0.1\"\n  port: 8080\n"
                       "viewer:\n  output_dir: \"\"\n");
    EXPECT_THROW(std::ignore = score::ConfigLoader::load(bad_dir.path),
                 std::runtime_error);
}

// ---------------------------------------------------------------------------
// cloud_point.disparity_offset_px and cloud_point.outlier_filter.*
// ---------------------------------------------------------------------------

namespace {

constexpr const char *kServerYaml =
    "server:\n  ip: \"127.0.0.1\"\n  port: 8080\n";

/// Loads @p yaml and returns the message of the std::runtime_error it throws
/// (fails the test when it does not throw).
std::string load_error(const std::string &yaml) {
    TempConfig cfg(yaml);
    try {
        std::ignore = score::ConfigLoader::load(cfg.path);
    } catch (const std::runtime_error &e) {
        return e.what();
    }
    ADD_FAILURE() << "expected std::runtime_error for:\n" << yaml;
    return {};
}

} // namespace

TEST(ConfigLoaderTest, ReconstructionKeysDefaultWhenOmitted) {
    // A cloud_point section without the new keys: offset 0, filter off with
    // the builder's defaults.
    TempConfig cfg(std::string(kServerYaml) +
                   "cloud_point:\n  algorithm: cpu\n  num_disparities: 160\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_DOUBLE_EQ(c.cloud_point.disparity_offset_px, 0.0);
    EXPECT_FALSE(c.cloud_point.outlier_filter.enabled);
    EXPECT_EQ(c.cloud_point.outlier_filter.window, 5);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.abs_threshold_m, 0.002);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.rel_threshold, 0.03);
    EXPECT_EQ(c.cloud_point.outlier_filter.min_neighbours, 6);

    // Same when the whole cloud_point section is absent.
    TempConfig none(kServerYaml);
    const auto n = score::ConfigLoader::load(none.path);
    EXPECT_DOUBLE_EQ(n.cloud_point.disparity_offset_px, 0.0);
    EXPECT_FALSE(n.cloud_point.outlier_filter.enabled);
    EXPECT_EQ(n.cloud_point.outlier_filter.window, 5);
}

TEST(ConfigLoaderTest, ReconstructionKeysPartialFilterKeepsDefaults) {
    TempConfig cfg(std::string(kServerYaml) +
                   "cloud_point:\n  outlier_filter:\n    enabled: true\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_TRUE(c.cloud_point.outlier_filter.enabled);
    EXPECT_EQ(c.cloud_point.outlier_filter.window, 5);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.abs_threshold_m, 0.002);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.rel_threshold, 0.03);
    EXPECT_EQ(c.cloud_point.outlier_filter.min_neighbours, 6);
}

TEST(ConfigLoaderTest, ReconstructionKeysOverridesAreRead) {
    TempConfig cfg(std::string(kServerYaml) + "cloud_point:\n"
                                              "  disparity_offset_px: -0.125\n"
                                              "  outlier_filter:\n"
                                              "    enabled: true\n"
                                              "    window: 7\n"
                                              "    abs_threshold_m: 0.004\n"
                                              "    rel_threshold: 0.05\n"
                                              "    min_neighbours: 12\n");
    const auto c = score::ConfigLoader::load(cfg.path);
    EXPECT_DOUBLE_EQ(c.cloud_point.disparity_offset_px, -0.125);
    EXPECT_TRUE(c.cloud_point.outlier_filter.enabled);
    EXPECT_EQ(c.cloud_point.outlier_filter.window, 7);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.abs_threshold_m, 0.004);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.rel_threshold, 0.05);
    EXPECT_EQ(c.cloud_point.outlier_filter.min_neighbours, 12);
}

TEST(ConfigLoaderTest, ReconstructionRejectsEvenOrSmallWindowByName) {
    const std::string even = load_error(std::string(kServerYaml) +
                                        "cloud_point:\n  outlier_filter:\n"
                                        "    window: 4\n");
    EXPECT_NE(even.find("outlier_filter.window"), std::string::npos) << even;

    const std::string small = load_error(std::string(kServerYaml) +
                                         "cloud_point:\n  outlier_filter:\n"
                                         "    window: 1\n");
    EXPECT_NE(small.find("outlier_filter.window"), std::string::npos) << small;
}

TEST(ConfigLoaderTest, ReconstructionValidatesFilterEvenWhenDisabled) {
    // The spec requires unconditional validation: a bad window is rejected
    // even though the filter is off.
    const std::string msg = load_error(std::string(kServerYaml) +
                                       "cloud_point:\n  outlier_filter:\n"
                                       "    enabled: false\n    window: 6\n");
    EXPECT_NE(msg.find("outlier_filter.window"), std::string::npos) << msg;
}

TEST(ConfigLoaderTest, ReconstructionRejectsNegativeThresholdsByName) {
    const std::string abs = load_error(std::string(kServerYaml) +
                                       "cloud_point:\n  outlier_filter:\n"
                                       "    abs_threshold_m: -0.001\n");
    EXPECT_NE(abs.find("outlier_filter.abs_threshold_m"), std::string::npos)
        << abs;

    const std::string rel = load_error(std::string(kServerYaml) +
                                       "cloud_point:\n  outlier_filter:\n"
                                       "    rel_threshold: -0.5\n");
    EXPECT_NE(rel.find("outlier_filter.rel_threshold"), std::string::npos)
        << rel;
}

TEST(ConfigLoaderTest, ReconstructionRejectsNonFiniteOffsetByName) {
    const std::string nan =
        load_error(std::string(kServerYaml) +
                   "cloud_point:\n  disparity_offset_px: .nan\n");
    EXPECT_NE(nan.find("disparity_offset_px"), std::string::npos) << nan;

    const std::string inf =
        load_error(std::string(kServerYaml) +
                   "cloud_point:\n  disparity_offset_px: .inf\n");
    EXPECT_NE(inf.find("disparity_offset_px"), std::string::npos) << inf;
}

TEST(ConfigLoaderTest, ReconstructionRejectsBadMinNeighboursByName) {
    // window^2 = 25: 25 is one more than the most neighbours a window holds.
    const std::string big = load_error(
        std::string(kServerYaml) + "cloud_point:\n  outlier_filter:\n"
                                   "    window: 5\n    min_neighbours: 25\n");
    EXPECT_NE(big.find("outlier_filter.min_neighbours"), std::string::npos)
        << big;

    const std::string neg = load_error(std::string(kServerYaml) +
                                       "cloud_point:\n  outlier_filter:\n"
                                       "    min_neighbours: -1\n");
    EXPECT_NE(neg.find("outlier_filter.min_neighbours"), std::string::npos)
        << neg;

    // 24 is the largest allowed value for a 5x5 window.
    TempConfig ok(std::string(kServerYaml) + "cloud_point:\n  outlier_filter:\n"
                                             "    window: 5\n"
                                             "    min_neighbours: 24\n");
    EXPECT_EQ(score::ConfigLoader::load(ok.path)
                  .cloud_point.outlier_filter.min_neighbours,
              24);
}

// ---------------------------------------------------------------------------
// The checked-in configuration files still load with the documented values.
// ---------------------------------------------------------------------------

#ifdef CPRPC_SOURCE_ROOT
TEST(ConfigLoaderTest, CheckedInConfigYamlExampleLoads) {
    const auto c = score::ConfigLoader::load(std::string(CPRPC_SOURCE_ROOT) +
                                             "/config.yaml.example");
    EXPECT_EQ(c.server.ip, "127.0.0.1");
    EXPECT_EQ(c.server.port, 8080);
    // The cloud_point section is commented out: defaults throughout.
    EXPECT_EQ(c.cloud_point.algorithm, "gpu");
    EXPECT_DOUBLE_EQ(c.cloud_point.disparity_offset_px, 0.0);
    EXPECT_FALSE(c.cloud_point.outlier_filter.enabled);
}

TEST(ConfigLoaderTest, CheckedInConfigScaredLoads) {
    const auto c = score::ConfigLoader::load(std::string(CPRPC_SOURCE_ROOT) +
                                             "/config.scared.yml");
    EXPECT_EQ(c.cloud_point.algorithm, "cpu");
    EXPECT_EQ(c.cloud_point.num_disparities, 160);
    EXPECT_DOUBLE_EQ(c.cloud_point.min_depth_m, 0.02);
    EXPECT_DOUBLE_EQ(c.cloud_point.max_depth_m, 0.30);
    // Effective values unchanged by the commented examples of the new keys.
    EXPECT_DOUBLE_EQ(c.cloud_point.disparity_offset_px, 0.0);
    EXPECT_FALSE(c.cloud_point.outlier_filter.enabled);
    EXPECT_EQ(c.cloud_point.outlier_filter.window, 5);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.abs_threshold_m, 0.002);
    EXPECT_DOUBLE_EQ(c.cloud_point.outlier_filter.rel_threshold, 0.03);
    EXPECT_EQ(c.cloud_point.outlier_filter.min_neighbours, 6);
}
#endif

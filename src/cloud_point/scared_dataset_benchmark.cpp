/// @file scared_dataset_benchmark.cpp
/// @brief Reconstruct a SCARED keyframe with the CPU stereo pipeline, report
/// depth statistics and, when point_cloud.obj is present, accuracy against
/// the XYZ ground truth plus the reconstruction diagnostics (depth-binned
/// bias, calibration-error fits, error tail, coverage-loss attribution).
/// Every accuracy-relevant setting can be overridden with `--key=value`
/// flags and is echoed under `settings` in the JSON output.
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cloud_point/cloud_point_client.hpp"
#include "cloud_point/cpu_stereo_matcher.hpp"
#include "cloud_point/imageFactory.h"
#include "cloud_point/point_cloud_builder.hpp"
#include "cloud_point/point_cloud_evaluator.hpp"
#include "cloud_point/reconstruction_diagnostics.hpp"
#include "cloud_point/scared_dataset_loader.hpp"
#include "cloud_point/scared_ground_truth_loader.hpp"
#include "cloud_point/stereo_rectifier.hpp"

namespace {

cv::Mat to_gray(const score::ImageRPC &rpc) {
    auto image = score::ImageFactory::create(rpc);
    cv::Mat gray;
    switch (rpc.type) {
    case score::ImageRPC::Type::BGR:
        cv::cvtColor(image.get(), gray, cv::COLOR_BGR2GRAY);
        break;
    case score::ImageRPC::Type::RGBA:
        cv::cvtColor(image.get(), gray, cv::COLOR_RGBA2GRAY);
        break;
    default:
        throw std::invalid_argument(
            "scared_dataset_benchmark: expected a colour stereo image");
    }
    return gray;
}

/// Decode to a BGR image (the viewer stores colour as RGB; see
/// encode_viewer_frame).
cv::Mat to_bgr(const score::ImageRPC &rpc) {
    auto image = score::ImageFactory::create(rpc);
    cv::Mat bgr;
    switch (rpc.type) {
    case score::ImageRPC::Type::BGR:
        bgr = image.get().clone();
        break;
    case score::ImageRPC::Type::RGBA:
        cv::cvtColor(image.get(), bgr, cv::COLOR_RGBA2BGR);
        break;
    default:
        throw std::invalid_argument(
            "scared_dataset_benchmark: expected a colour stereo image");
    }
    return bgr;
}

/// Optional `--viewer-*` flags, accepted anywhere on the command line.
struct ViewerArgs {
    std::string frame_path; ///< empty = no export
    int stride{4};
};

/// Every optional `--key[=value]` flag.  The reconstruction settings default
/// to the same `Params()` / `Options()` constructors the config loader uses,
/// so a run without flags equals the CLI defaults (design D6).
struct BenchmarkOptions {
    ViewerArgs viewer;
    score::CpuStereoMatcher::Params matcher;
    score::PointCloudBuilder::Options builder; ///< offset + outlier filter
};

/// Strict integer parse: the whole value must be consumed.
int parse_int(std::string_view flag, std::string_view value) {
    int out{};
    const auto *end = value.data() + value.size();
    const auto res  = std::from_chars(value.data(), end, out);
    if (res.ec != std::errc{} || res.ptr != end || value.empty()) {
        throw std::invalid_argument(std::string(flag) +
                                    " expects an integer, got '" +
                                    std::string(value) + "'");
    }
    return out;
}

/// Strict floating-point parse: the whole value must be consumed and finite.
double parse_double(std::string_view flag, std::string_view value) {
    const std::string text(value);
    std::size_t consumed = 0;
    double out{};
    try {
        out = std::stod(text, &consumed);
    } catch (const std::exception &) {
        consumed = 0;
    }
    if (text.empty() || consumed != text.size() || !std::isfinite(out)) {
        throw std::invalid_argument(std::string(flag) +
                                    " expects a finite number, got '" +
                                    text + "'");
    }
    return out;
}

/// Strip every `--flag[=value]` out of argv, leaving the positional
/// arguments exactly as before these flags existed.
/// @throws std::invalid_argument on an unknown or malformed flag.
BenchmarkOptions extract_options(int argc, char *argv[],
                                 std::vector<std::string> &positional) {
    BenchmarkOptions opts;
    auto &m = opts.matcher;
    auto &b = opts.builder;
    auto &f = opts.builder.outlier_filter;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.rfind("--", 0) != 0) {
            positional.push_back(arg);
            continue;
        }
        const auto eq = arg.find('=');
        const std::string key =
            eq == std::string::npos ? arg : arg.substr(0, eq);
        const bool has_value = eq != std::string::npos;
        const std::string value = has_value ? arg.substr(eq + 1) : "";

        auto need_value = [&]() -> const std::string & {
            if (!has_value)
                throw std::invalid_argument(key + " needs a value (" + key +
                                            "=...)");
            return value;
        };
        auto no_value = [&]() {
            if (has_value)
                throw std::invalid_argument(key + " does not take a value");
        };

        if (key == "--viewer-frame") {
            opts.viewer.frame_path = need_value();
            if (opts.viewer.frame_path.empty())
                throw std::invalid_argument("--viewer-frame needs a path");
        } else if (key == "--viewer-stride") {
            opts.viewer.stride = parse_int(key, need_value());
            if (opts.viewer.stride < 1)
                throw std::invalid_argument("--viewer-stride must be >= 1");
        } else if (key == "--block-size") {
            m.block_size = parse_int(key, need_value());
        } else if (key == "--uniqueness") {
            m.uniqueness_ratio = parse_int(key, need_value());
        } else if (key == "--speckle-window") {
            m.speckle_window_size = parse_int(key, need_value());
        } else if (key == "--speckle-range") {
            m.speckle_range = parse_int(key, need_value());
        } else if (key == "--disp12-max-diff") {
            m.disp12_max_diff = parse_int(key, need_value());
        } else if (key == "--pre-filter-cap") {
            m.pre_filter_cap = parse_int(key, need_value());
        } else if (key == "--median-kernel") {
            m.median_kernel = parse_int(key, need_value());
        } else if (key == "--wls") {
            no_value();
            m.wls_filter = true;
        } else if (key == "--no-wls") {
            no_value();
            m.wls_filter = false;
        } else if (key == "--disparity-offset") {
            b.disparity_offset_px = parse_double(key, need_value());
        } else if (key == "--outlier-filter") {
            no_value();
            f.enabled = true;
        } else if (key == "--no-outlier-filter") {
            no_value();
            f.enabled = false;
        } else if (key == "--outlier-window") {
            f.window = parse_int(key, need_value());
        } else if (key == "--outlier-abs-m") {
            f.abs_threshold_m = parse_double(key, need_value());
        } else if (key == "--outlier-rel") {
            f.rel_threshold = parse_double(key, need_value());
        } else if (key == "--outlier-min-neighbours") {
            f.min_neighbours = parse_int(key, need_value());
        } else {
            throw std::invalid_argument("unknown option " + arg);
        }
    }
    return opts;
}

/// Validate the reconstruction settings before the dataset is touched, by
/// running the same constructors the pipeline will use (with a placeholder
/// Q; the real one needs the calibration).
/// @throws std::invalid_argument naming the offending setting.
void validate_options(const BenchmarkOptions &opts, int num_disparities) {
    if (num_disparities <= 0 || num_disparities % 16 != 0) {
        throw std::invalid_argument(
            "num_disparities must be a positive multiple of 16, got " +
            std::to_string(num_disparities));
    }
    if (opts.matcher.uniqueness_ratio < 0 || opts.matcher.speckle_window_size < 0 ||
        opts.matcher.speckle_range < 0 || opts.matcher.pre_filter_cap < 1) {
        throw std::invalid_argument(
            "--uniqueness, --speckle-window and --speckle-range must be >= 0 "
            "and --pre-filter-cap >= 1");
    }
    score::CpuStereoMatcher probe(0, num_disparities, opts.matcher);
    (void)probe;
    score::PointCloudBuilder builder(cv::Mat::eye(4, 4, CV_64F), opts.builder);
    (void)builder;
}

void print_usage(const char *program) {
    std::cerr
        << "Usage: " << program
        << " [flags] <keyframe_dir> [num_disparities] [depth_png|-] "
           "[min_depth_m max_depth_m]\n"
        << "  Accuracy metrics and diagnostics are reported when "
           "<keyframe_dir>/point_cloud.obj exists;\n"
        << "  depth statistics are always reported.\n"
        << "Viewer flags:\n"
        << "  --viewer-frame=PATH        write a web-viewer frame script "
           "(CloudViewer.addFrame; id = file stem)\n"
        << "  --viewer-stride=N          sample every N-th pixel for the "
           "viewer frame (default 4)\n"
        << "Matcher flags (defaults = CpuStereoMatcher::Params):\n"
        << "  --block-size=N --uniqueness=N --speckle-window=N "
           "--speckle-range=N\n"
        << "  --disp12-max-diff=N --pre-filter-cap=N --median-kernel=N "
           "--wls | --no-wls\n"
        << "Reconstruction flags (defaults = PointCloudBuilder::Options):\n"
        << "  --disparity-offset=PX      added to every valid disparity\n"
        << "  --outlier-filter | --no-outlier-filter\n"
        << "  --outlier-window=N --outlier-abs-m=M --outlier-rel=R "
           "--outlier-min-neighbours=N\n";
}

/// `settings`: every effective reconstruction setting (spec: tuning flags).
nlohmann::json settings_json(const BenchmarkOptions &opts, int min_disparity,
                             int num_disparities) {
    const auto &m = opts.matcher;
    const auto &b = opts.builder;
    const auto &f = b.outlier_filter;
    return {
        {"min_disparity", min_disparity},
        {"num_disparities", num_disparities},
        {"block_size", m.block_size},
        {"uniqueness_ratio", m.uniqueness_ratio},
        {"speckle_window_size", m.speckle_window_size},
        {"speckle_range", m.speckle_range},
        {"disp12_max_diff", m.disp12_max_diff},
        {"pre_filter_cap", m.pre_filter_cap},
        {"median_kernel", m.median_kernel},
        {"wls_filter", m.wls_filter},
        {"wls_lambda", m.wls_lambda},
        {"wls_sigma", m.wls_sigma},
        {"min_depth_m", b.min_depth_m},
        {"max_depth_m", b.max_depth_m},
        {"disparity_offset_px", b.disparity_offset_px},
        {"outlier_filter",
         {
             {"enabled", f.enabled},
             {"window", f.window},
             {"abs_threshold_m", f.abs_threshold_m},
             {"rel_threshold", f.rel_threshold},
             {"min_neighbours", f.min_neighbours},
         }},
    };
}

/// Diagnostics keys (spec reconstruction-diagnostics), merged into the
/// top-level JSON next to the evaluator's metrics.
void append_diagnostics(nlohmann::json &output,
                        const score::ReconstructionDiagnostics &d) {
    nlohmann::json bins = nlohmann::json::array();
    for (const auto &bin : d.depth_bias) {
        bins.push_back({
            {"lo_m", bin.lo_m},
            {"hi_m", bin.hi_m},
            {"count", bin.count},
            {"bias_z_median_m", bin.bias_z_median_m},
            {"abs_3d_median_m", bin.abs_3d_median_m},
        });
    }
    output["depth_bias"]                      = std::move(bins);
    output["bias_z_median_m"]                 = d.bias_z_median_m;
    output["fit_disparity_offset_px"]         = d.fit_disparity_offset_px;
    output["fit_disparity_offset_residual_m"] = d.fit_disparity_offset_residual_m;
    output["fit_depth_scale"]                 = d.fit_depth_scale;
    output["fit_depth_scale_residual_m"]      = d.fit_depth_scale_residual_m;
    output["tail_over_5mm"]                   = d.tail_over_5mm;
    output["tail_over_20mm"]                  = d.tail_over_20mm;
    output["tail_over_50mm"]                  = d.tail_over_50mm;
    output["max_3d_m"]                        = d.max_3d_m;
    output["coverage_loss"]                   = {
        {"out_of_search_range", d.coverage_loss.out_of_search_range},
        {"image_border", d.coverage_loss.image_border},
        {"matcher_rejected", d.coverage_loss.matcher_rejected},
        {"depth_range", d.coverage_loss.depth_range},
        {"outlier_filter", d.coverage_loss.outlier_filter},
    };
    output["unscored_fraction"] = d.unscored_fraction;
}

/// Depth percentiles (metres) over the valid points of a cloud.
struct DepthStats {
    std::size_t valid_points{0};
    double valid_fraction{0.0};
    double z_min_m{0.0}, z_p05_m{0.0}, z_median_m{0.0}, z_p95_m{0.0},
        z_max_m{0.0};
};

DepthStats depth_stats(const score::PointCloud &cloud) {
    std::vector<float> z;
    z.reserve(static_cast<std::size_t>(cloud.width) *
              static_cast<std::size_t>(cloud.height));
    for (const auto &pt : cloud.valid_points())
        z.push_back(pt[2]);
    DepthStats stats;
    stats.valid_points = z.size();
    stats.valid_fraction =
        static_cast<double>(z.size()) /
        (static_cast<double>(cloud.width) * static_cast<double>(cloud.height));
    if (z.empty())
        return stats;
    std::sort(z.begin(), z.end());
    const auto at = [&](double q) {
        return static_cast<double>(
            z[std::min(z.size() - 1, static_cast<std::size_t>(
                                         q * static_cast<double>(z.size())))]);
    };
    stats.z_min_m    = z.front();
    stats.z_p05_m    = at(0.05);
    stats.z_median_m = at(0.5);
    stats.z_p95_m    = at(0.95);
    stats.z_max_m    = z.back();
    return stats;
}

/// Colour-mapped depth image (invalid pixels black) for visual inspection.
void write_depth_png(const score::PointCloud &cloud, const std::string &path,
                     double z_lo, double z_hi) {
    cv::Mat depth8(cloud.height, cloud.width, CV_8UC1, cv::Scalar(0));
    cv::Mat valid(cloud.height, cloud.width, CV_8UC1, cv::Scalar(0));
    const double span = std::max(z_hi - z_lo, 1e-6);
    for (int r = 0; r < cloud.height; ++r) {
        for (int c = 0; c < cloud.width; ++c) {
            const std::size_t idx = (static_cast<std::size_t>(r) *
                                         static_cast<std::size_t>(cloud.width) +
                                     static_cast<std::size_t>(c)) *
                                    3u;
            const float z         = cloud.data[idx + 2];
            if (std::isnan(z))
                continue;
            const double t         = std::clamp((z - z_lo) / span, 0.0, 1.0);
            depth8.at<uchar>(r, c) = static_cast<uchar>(255.0 * (1.0 - t));
            valid.at<uchar>(r, c)  = 255;
        }
    }
    cv::Mat colour;
    cv::applyColorMap(depth8, colour, cv::COLORMAP_TURBO);
    colour.setTo(cv::Scalar(0, 0, 0), valid == 0);
    if (!cv::imwrite(path, colour))
        throw std::runtime_error("failed to write " + path);
}

} // namespace

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    // Positional arguments after the --viewer-* flags are removed; args[0]
    // is a placeholder so that the indices match the historical argv layout.
    std::vector<std::string> args{argv[0]};
    BenchmarkOptions options;
    std::string keyframe_dir;
    int num_disparities = 160;
    std::string depth_png;
    try {
        options = extract_options(argc, argv, args);
        const int n_args = static_cast<int>(args.size());
        if (n_args < 2 || n_args > 6 || n_args == 5) {
            throw std::invalid_argument("expected 1 to 5 positional arguments");
        }
        keyframe_dir    = args[1];
        num_disparities = n_args >= 3 ? parse_int("num_disparities", args[2])
                                      : 160;
        depth_png       = n_args >= 4 ? args[3] : "";
        if (depth_png == "-")
            depth_png.clear();
        if (n_args == 6) {
            options.builder.min_depth_m =
                static_cast<float>(parse_double("min_depth_m", args[4]));
            options.builder.max_depth_m =
                static_cast<float>(parse_double("max_depth_m", args[5]));
        }
        validate_options(options, num_disparities);
    } catch (const std::exception &error) {
        std::cerr << "Invalid option: " << error.what() << '\n';
        print_usage(argv[0]);
        return 1;
    }
    const ViewerArgs &viewer = options.viewer;
    const score::PointCloudBuilder::Options &depth_range = options.builder;
    constexpr int min_disparity = 0; // as StereoMatcherFactory

    try {
        score::ScaredDatasetLoader dataset(keyframe_dir);
        const auto &calibration_rpc = dataset.calibration();
        const cv::Size image_size(calibration_rpc.width,
                                  calibration_rpc.height);
        std::optional<score::ScaredGroundTruthLoader> ground_truth;
        if (std::filesystem::exists(keyframe_dir + "/point_cloud.obj")) {
            ground_truth.emplace(keyframe_dir, image_size);
        } else {
            LOG(WARNING) << "No point_cloud.obj in " << keyframe_dir
                         << "; reporting depth statistics only";
        }

        const auto calibration =
            score::StereoRectifier::Calibration::from_rpc(calibration_rpc);
        score::StereoRectifier rectifier(calibration);
        score::CpuStereoMatcher matcher(min_disparity, num_disparities,
                                        options.matcher);
        score::PointCloudBuilder builder(rectifier.q(), depth_range);

        const auto pair          = dataset.image_pair(0);
        const cv::Mat left_gray  = to_gray(pair.left);
        const cv::Mat right_gray = to_gray(pair.right);
        auto [rectified_left, rectified_right] =
            rectifier.rectify(left_gray, right_gray);

        const auto start = std::chrono::steady_clock::now();
        const cv::Mat disparity =
            matcher.compute(rectified_left, rectified_right);
        const auto matching_end = std::chrono::steady_clock::now();
        cv::Mat reason;
        const score::PointCloud cloud = builder.build(disparity, &reason);
        const auto reconstruction_end = std::chrono::steady_clock::now();

        const auto stats              = depth_stats(cloud);
        if (!depth_png.empty()) {
            write_depth_png(cloud, depth_png, stats.z_p05_m, stats.z_p95_m);
        }

        const auto matching_ms =
            std::chrono::duration<double, std::milli>(matching_end - start)
                .count();
        const auto reconstruction_ms =
            std::chrono::duration<double, std::milli>(reconstruction_end -
                                                      matching_end)
                .count();

        nlohmann::json output = {
            {"keyframe_dir", keyframe_dir},
            {"algorithm", "StereoSGBM"},
            {"num_disparities", num_disparities},
            {"min_depth_m", depth_range.min_depth_m},
            {"max_depth_m", depth_range.max_depth_m},
            {"image_width", cloud.width},
            {"image_height", cloud.height},
            {"valid_points", stats.valid_points},
            {"valid_fraction", stats.valid_fraction},
            {"z_min_m", stats.z_min_m},
            {"z_p05_m", stats.z_p05_m},
            {"z_median_m", stats.z_median_m},
            {"z_p95_m", stats.z_p95_m},
            {"z_max_m", stats.z_max_m},
            {"matching_ms", matching_ms},
            {"reconstruction_ms", reconstruction_ms},
            {"has_ground_truth", ground_truth.has_value()},
            {"settings",
             settings_json(options, min_disparity, num_disparities)},
        };
        if (!depth_png.empty())
            output["depth_png"] = depth_png;

        cv::Mat rectified_ground_truth;
        if (ground_truth) {
            rectified_ground_truth =
                rectifier.rectify_left_point_map(ground_truth->point_map());
        }

        if (!viewer.frame_path.empty()) {
            // Rectify the colour left image on the same grid as the cloud.
            const cv::Mat left_bgr = to_bgr(pair.left);
            const cv::Mat rectified_left_bgr =
                rectifier.rectify(left_bgr, left_bgr).first;
            const std::string frame_id =
                std::filesystem::path(viewer.frame_path).stem().string();
            score::write_viewer_frame_js(
                viewer.frame_path, frame_id,
                score::encode_viewer_frame(
                    cloud, &rectified_ground_truth, &rectified_left_bgr,
                    score::ViewerFrameOptions{viewer.stride}));
            output["viewer_frame"] = viewer.frame_path;
        }

        if (ground_truth) {
            const auto metrics = score::PointCloudEvaluator::evaluate(
                cloud, rectified_ground_truth);
            output["ground_truth_points"] = metrics.ground_truth_points;
            output["matched_points"]      = metrics.matched_points;
            output["coverage"]            = metrics.coverage;
            output["mae_x_m"]             = metrics.mae_x_m;
            output["mae_y_m"]             = metrics.mae_y_m;
            output["mae_z_m"]             = metrics.mae_z_m;
            output["mae_3d_m"]            = metrics.mae_3d_m;
            output["rmse_3d_m"]           = metrics.rmse_3d_m;
            output["median_3d_m"]         = metrics.median_3d_m;
            output["within_1mm"]          = metrics.within_1mm;
            output["within_2mm"]          = metrics.within_2mm;
            output["within_5mm"]          = metrics.within_5mm;

            score::ReconstructionDiagnosticsInputs inputs;
            inputs.q                   = rectifier.q();
            inputs.reason              = reason;
            inputs.min_disparity       = min_disparity;
            inputs.num_disparities     = num_disparities;
            inputs.disparity_offset_px = depth_range.disparity_offset_px;
            inputs.min_depth_m         = depth_range.min_depth_m;
            inputs.max_depth_m         = depth_range.max_depth_m;
            append_diagnostics(output,
                               score::diagnose_reconstruction(
                                   cloud, rectified_ground_truth, inputs));
        }
        std::cout << output.dump(2) << '\n';
    } catch (const std::exception &error) {
        std::cerr << "Benchmark failed: " << error.what() << '\n';
        return 1;
    }

    return 0;
}

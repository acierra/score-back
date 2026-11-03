#include "cloud_point_rpc/cli.hpp"
#include "cloud_point_rpc/rpc_client.hpp"
#include <glog/logging.h>
#include <string>

#ifdef HAVE_CLOUD_POINT_COMPUTE
#include "cloud_point/cloud_point_client.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <expected>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <thread>
#include <vector>
#ifdef _WIN32
#include <process.h>
#else
#include <spawn.h>
#include <sys/wait.h>
extern char **environ;
#endif
#ifndef CPRPC_VIEWER_TEMPLATE_DIR
#define CPRPC_VIEWER_TEMPLATE_DIR "scripts/cloud_viewer/site"
#endif
#endif

namespace score {

void print_menu(std::ostream &output) {
    output << "\n=== Cloud Point RPC CLI ===" << std::endl;
    output << "1. get-intrinsic-params" << std::endl;
    output << "2. get-extrinsic-params" << std::endl;
    output << "3. get-cloud-point" << std::endl;
    output << "4. compute-cloud" << std::endl;
    output << "5. compute-cloud + save PLY" << std::endl;
    output << "6. compute-cloud + open viewer" << std::endl;
    output << "0. Exit" << std::endl;
    output << "Select an option: ";
}

template <typename T> std::string vector_to_string(const std::vector<T> &v) {
    std::string result;
    for (auto &el : v) {
        result += std::to_string(el) + " ";
    }
    return result;
}

template <typename T>
std::string vector_to_string(const std::vector<std::vector<T>> &v) {
    std::string result;
    for (auto &el : v) {
        result += vector_to_string(el) + "\n";
    }
    return result;
}

#ifdef HAVE_CLOUD_POINT_COMPUTE
namespace {

namespace fs = std::filesystem;

/// Print the "Cloud: WxH valid_pts=N" summary shared by options 4, 5 and 6.
void print_cloud_summary(std::ostream &output, const PointCloud &cloud,
                         const std::vector<std::array<float, 3>> &valid) {
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();

    for (const auto &pt : valid) {
        x_min = std::min(x_min, pt[0]);
        x_max = std::max(x_max, pt[0]);
        y_min = std::min(y_min, pt[1]);
        y_max = std::max(y_max, pt[1]);
        z_min = std::min(z_min, pt[2]);
        z_max = std::max(z_max, pt[2]);
    }

    output << "Cloud: " << cloud.width << "x" << cloud.height
           << "  valid_pts=" << valid.size() << "\n";
    if (!valid.empty()) {
        output << "  bbox x=[" << x_min << "," << x_max << "]"
               << " y=[" << y_min << "," << y_max << "]"
               << " z=[" << z_min << "," << z_max << "]\n";
    }
}

/// Depth percentiles of the valid points (same definition as the SCARED
/// benchmark's depth_stats): manifest metrics for a frame without GT.
nlohmann::json frame_metrics(const PointCloud &cloud,
                             const std::vector<std::array<float, 3>> &valid) {
    nlohmann::json m;
    const double total =
        static_cast<double>(cloud.width) * static_cast<double>(cloud.height);
    m["valid_fraction"] =
        total > 0.0 ? static_cast<double>(valid.size()) / total : 0.0;
    if (valid.empty())
        return m;
    std::vector<float> z;
    z.reserve(valid.size());
    for (const auto &pt : valid)
        z.push_back(pt[2]);
    std::sort(z.begin(), z.end());
    const auto at = [&](double q) {
        return static_cast<double>(
            z[std::min(z.size() - 1, static_cast<std::size_t>(
                                         q * static_cast<double>(z.size())))]);
    };
    m["z_p05_m"]    = at(0.05);
    m["z_median_m"] = at(0.5);
    m["z_p95_m"]    = at(0.95);
    return m;
}

struct LocalTime {
    std::tm tm{};
    int millis{0};
    long utc_offset_s{0};
};

LocalTime local_now() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto t   = system_clock::to_time_t(now);
    LocalTime lt;
#ifdef _WIN32
    localtime_s(&lt.tm, &t);
#else
    localtime_r(&t, &lt.tm);
    lt.utc_offset_s = lt.tm.tm_gmtoff;
#endif
    lt.millis = static_cast<int>(
        duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
    return lt;
}

/// "cli_YYYYMMDD_HHMMSS_mmm": sortable, unique per capture, and a valid
/// viewer frame id ([A-Za-z0-9_.-]+).
std::string frame_id_for(const LocalTime &lt) {
    char buf[32];
    std::strftime(buf, sizeof buf, "cli_%Y%m%d_%H%M%S", &lt.tm);
    char ms[8];
    std::snprintf(ms, sizeof ms, "_%03d", lt.millis);
    return std::string(buf) + ms;
}

std::string iso8601(const LocalTime &lt) {
    char buf[40];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%S", &lt.tm);
    const long off  = lt.utc_offset_s;
    const long aoff = off < 0 ? -off : off;
    char zone[8];
    std::snprintf(zone, sizeof zone, "%c%02d:%02d", off < 0 ? '-' : '+',
                  static_cast<int>(aoff / 3600) % 100,
                  static_cast<int>((aoff % 3600) / 60));
    return std::string(buf) + zone;
}

std::string time_label(const LocalTime &lt) {
    char buf[16];
    std::strftime(buf, sizeof buf, "%H:%M:%S", &lt.tm);
    return buf;
}

/// Frame list sidecar (frames/index.json): what the CLI remembers about the
/// frames it wrote, so that the manifest can be rebuilt from all of them.
nlohmann::json read_frame_index(const fs::path &index_path) {
    std::ifstream in(index_path);
    if (!in)
        return nlohmann::json::array();
    try {
        auto j = nlohmann::json::parse(in);
        if (j.is_array())
            return j;
    } catch (const nlohmann::json::exception &e) {
        LOG(WARNING) << "Ignoring unreadable " << index_path.string() << ": "
                     << e.what();
    }
    return nlohmann::json::array();
}

/// Adds one capture to the site at `site_dir` (copying the template on first
/// use) and rewrites frames/manifest.js from every frame still present.
/// @return the number of frames listed in the manifest.
std::size_t add_viewer_frame(const fs::path &site_dir,
                             const fs::path &template_dir,
                             const std::string &id, const std::string &label,
                             const std::string &generated,
                             const std::string &payload_b64,
                             nlohmann::json metrics) {
    if (!fs::exists(site_dir / "index.html")) {
        fs::create_directories(site_dir);
        fs::copy(template_dir, site_dir,
                 fs::copy_options::recursive |
                     fs::copy_options::overwrite_existing);
    }
    const fs::path frames_dir = site_dir / "frames";
    fs::create_directories(frames_dir);

    write_viewer_frame_js((frames_dir / (id + ".js")).string(), id,
                          payload_b64);

    const fs::path index_path = frames_dir / "index.json";
    nlohmann::json index      = read_frame_index(index_path);
    // Drop entries whose frame file disappeared (manual cleanup).
    nlohmann::json kept = nlohmann::json::array();
    for (const auto &entry : index) {
        if (entry.is_object() && entry.contains("id") &&
            entry["id"].is_string() && entry["id"] != id &&
            fs::exists(frames_dir / (entry["id"].get<std::string>() + ".js"))) {
            kept.push_back(entry);
        }
    }
    kept.push_back({{"id", id},
                    {"label", label},
                    {"has_gt", false},
                    {"metrics", std::move(metrics)}});
    {
        std::ofstream out(index_path);
        if (!out)
            throw std::runtime_error("cannot write " + index_path.string());
        out << kept.dump(2) << "\n";
    }

    const nlohmann::json manifest = {{"title", "cloudPointRPC CLI captures"},
                                     {"generated", generated},
                                     {"frames", kept}};
    const fs::path manifest_path  = frames_dir / "manifest.js";
    std::ofstream out(manifest_path);
    if (!out)
        throw std::runtime_error("cannot write " + manifest_path.string());
    out << "CloudViewer.setManifest(" << manifest.dump(2) << ");\n";
    return kept.size();
}

/// Launch `command <arg>` without a shell and without blocking the menu.
///
/// The child is given a short grace period so that an immediate failure
/// (command exits non-zero right away) can still be reported; after that it
/// is handed to a detached reaper thread.
/// @return empty string on success, otherwise the reason it did not open.
std::string spawn_opener(const std::string &command, const std::string &arg) {
#ifdef _WIN32
    const auto rc = _spawnlp(_P_NOWAIT, command.c_str(), command.c_str(),
                             arg.c_str(), nullptr);
    if (rc == -1)
        return command + ": " + std::strerror(errno);
    return {};
#else
    std::vector<char *> argv{const_cast<char *>(command.c_str()),
                             const_cast<char *>(arg.c_str()), nullptr};
    pid_t pid     = 0;
    const int err = posix_spawnp(&pid, command.c_str(), nullptr, nullptr,
                                 argv.data(), environ);
    if (err != 0)
        return command + ": " + std::strerror(err);

    using namespace std::chrono;
    const auto deadline = steady_clock::now() + milliseconds(300);
    while (steady_clock::now() < deadline) {
        int status         = 0;
        const pid_t waited = waitpid(pid, &status, WNOHANG);
        if (waited == pid) {
            if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
                return command + " exited with status " +
                       std::to_string(WEXITSTATUS(status));
            if (WIFSIGNALED(status))
                return command + " was killed by signal " +
                       std::to_string(WTERMSIG(status));
            return {};
        }
        if (waited < 0)
            return {}; // Already reaped elsewhere; nothing to report.
        std::this_thread::sleep_for(milliseconds(10));
    }
    // Still running (a browser started in the foreground, for example):
    // reap it in the background so it never lingers as a zombie.
    std::thread([pid] {
        int status = 0;
        waitpid(pid, &status, 0);
    }).detach();
    return {};
#endif
}

} // namespace
#endif

int run_cli(std::istream &input, std::ostream &output, const std::string &ip,
            int port, [[maybe_unused]] const CliStereoOptions &stereo) {
    try {
        TCPConnector connector(ip, port);
        RpcClient client(connector);

        output << "Connected to " << ip << ":" << port << std::endl;

        std::string choice;
        while (true) {
            print_menu(output);
            if (!(input >> choice))
                break;

            if (choice == "0")
                break;

            if (choice == "1" || choice == "2" || choice == "3") {
                std::string method;
                if (choice == "1") {
                    method = "get-intrinsic-params";
                } else if (choice == "2") {
                    method = "get-extrinsic-params";
                } else {
                    method = "get-cloud-point";
                }

                try {
                    if (method == "get-intrinsic-params") {
                        auto response = client.get_intrinsic_params();
                        output << vector_to_string(response);
                    }
                    if (method == "get-extrinsic-params") {
                        auto response = client.get_extrinsic_params();
                        output << vector_to_string(response);
                    }
                    if (method == "get-cloud-point") {
                        auto response = client.get_cloud_point();
                        output << vector_to_string(response);
                    }
                } catch (const std::exception &e) {
                    output << "\nRPC Error: " << e.what() << std::endl;
                }
            } else if (choice == "4" || choice == "5" || choice == "6") {
#ifdef HAVE_CLOUD_POINT_COMPUTE
                try {
                    const auto algo = stereo.use_gpu ? StereoAlgorithmType::GPU
                                                     : StereoAlgorithmType::CPU;
                    CpuStereoMatcher::Params matcher;
                    matcher.wls_filter = stereo.wls_filter;
                    // Options 4, 5 and 6 share this builder configuration,
                    // so the PLY export and the viewer see the same cloud.
                    PointCloudBuilder::Options builder{stereo.min_depth_m,
                                                       stereo.max_depth_m};
                    builder.disparity_offset_px = stereo.disparity_offset_px;
                    builder.outlier_filter.enabled =
                        stereo.outlier_filter.enabled;
                    builder.outlier_filter.window =
                        stereo.outlier_filter.window;
                    builder.outlier_filter.abs_threshold_m =
                        stereo.outlier_filter.abs_threshold_m;
                    builder.outlier_filter.rel_threshold =
                        stereo.outlier_filter.rel_threshold;
                    builder.outlier_filter.min_neighbours =
                        stereo.outlier_filter.min_neighbours;
                    CloudPointClient cpc(ip, port, algo, builder,
                                         stereo.num_disparities, matcher);
                    cpc.connect();

                    // Option 6 needs the rectified colour image as well.
                    std::expected<CloudPointClient::Frame,
                                  CloudPointClient::Error>
                        result;
                    if (choice == "6") {
                        result = cpc.compute_frame();
                    } else if (auto cloud = cpc.compute_cloud()) {
                        result = CloudPointClient::Frame{std::move(*cloud), {}};
                    } else {
                        result = std::unexpected(cloud.error());
                    }

                    if (!result) {
                        output << "Error: " << result.error().message
                               << std::endl;
                    } else {
                        const auto &cloud = result->cloud;
                        const auto valid  = cloud.valid_points();
                        print_cloud_summary(output, cloud, valid);

                        if (choice == "5") {
                            output << "PLY output path: ";
                            std::string path;
                            if (input >> path) {
                                PlyOptions ply;
                                ply.stride       = stereo.ply_stride;
                                const auto faces = write_ply(cloud, path, ply);
                                output << "Saved mesh (stride " << ply.stride
                                       << ", " << faces << " faces) from "
                                       << valid.size() << " valid points to "
                                       << path << "\n";
                            }
                        } else if (choice == "6") {
                            const auto &vo = stereo.viewer;
                            const fs::path template_dir =
                                vo.template_dir.empty()
                                    ? fs::path(CPRPC_VIEWER_TEMPLATE_DIR)
                                    : fs::path(vo.template_dir);
                            const fs::path site_dir =
                                fs::absolute(vo.output_dir);
                            const fs::path index_html = site_dir / "index.html";

                            if (!fs::exists(index_html) &&
                                !fs::is_directory(template_dir)) {
                                output << "Error: viewer template not found "
                                          "at "
                                       << template_dir.string()
                                       << " (set viewer.template_dir)"
                                       << std::endl;
                                continue;
                            }

                            const auto now    = local_now();
                            std::string id    = frame_id_for(now);
                            const auto frames = site_dir / "frames";
                            for (int n = 2; fs::exists(frames / (id + ".js"));
                                 ++n) {
                                id =
                                    frame_id_for(now) + "_" + std::to_string(n);
                            }
                            const std::string label =
                                ip + ":" + std::to_string(port) + " \xC2\xB7 " +
                                time_label(now);

                            const auto payload = encode_viewer_frame(
                                cloud, nullptr, &result->rectified_left_bgr,
                                ViewerFrameOptions{vo.stride});
                            const auto n_frames = add_viewer_frame(
                                site_dir, template_dir, id, label, iso8601(now),
                                payload, frame_metrics(cloud, valid));
                            output << "Saved viewer frame " << id << " (stride "
                                   << vo.stride << ", " << n_frames << " frame"
                                   << (n_frames == 1 ? "" : "s")
                                   << " in site)\n";
                            output << "Viewer: " << index_html.string()
                                   << std::endl;

                            if (vo.open_command.empty()) {
                                output << "No viewer.open_command set; open "
                                          "the page manually."
                                       << std::endl;
                            } else if (const auto why =
                                           spawn_opener(vo.open_command,
                                                        index_html.string());
                                       !why.empty()) {
                                output << "Viewer written to "
                                       << index_html.string()
                                       << " (could not open a browser: " << why
                                       << ")" << std::endl;
                            }
                        }
                    }
                } catch (const std::exception &e) {
                    output << "Error: " << e.what() << std::endl;
                }
#else
                output << "OpenCV support not built" << std::endl;
#endif
            } else {
                output << "Invalid option: " << choice << std::endl;
            }
        }
    } catch (const std::exception &e) {
        LOG(ERROR) << "CLI Error: " << e.what();
        output << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

} // namespace score

#include "cloud_point/cloud_point_client.hpp"

#include "cloud_point/imageFactory.h"
#include "cloud_point_rpc/rpc_client.hpp"
#include "cloud_point_rpc/tcp_connector.hpp"
#include "libbase64.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <jsonrpccxx/common.hpp>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

namespace score {

CloudPointClient::CloudPointClient(std::string ip, int port,
                                   StereoAlgorithmType algo,
                                   PointCloudBuilder::Options opts,
                                   int num_disparities,
                                   CpuStereoMatcher::Params matcher_params)
    : ip_(std::move(ip)), port_(port), algo_(algo), opts_(opts),
      num_disparities_(num_disparities), matcher_params_(matcher_params) {}

CloudPointClient::~CloudPointClient() = default;

void CloudPointClient::connect() {
    connector_           = std::make_unique<TCPConnector>(ip_, port_);
    client_              = std::make_unique<RpcClient>(*connector_);

    const auto calib_rpc = client_->get_stereo_calibration();
    const auto calib     = StereoRectifier::Calibration::from_rpc(calib_rpc);
    rectifier_           = std::make_unique<StereoRectifier>(calib);
    matcher_ = StereoMatcherFactory::create(algo_, num_disparities_,
                                            matcher_params_);
    builder_ = std::make_unique<PointCloudBuilder>(rectifier_->q(), opts_);
}

bool CloudPointClient::connected() const noexcept {
    return connector_ != nullptr && client_ != nullptr &&
           rectifier_ != nullptr && matcher_ != nullptr && builder_ != nullptr;
}

std::expected<PointCloud, CloudPointClient::Error>
CloudPointClient::compute_cloud() {
    auto frame = compute(/*with_colour=*/false);
    if (!frame)
        return std::unexpected(frame.error());
    return std::move(frame->cloud);
}

std::expected<CloudPointClient::Frame, CloudPointClient::Error>
CloudPointClient::compute_frame() {
    return compute(/*with_colour=*/true);
}

std::expected<CloudPointClient::Frame, CloudPointClient::Error>
CloudPointClient::compute(bool with_colour) {
    if (!connected()) {
        return std::unexpected(Error{"not connected — call connect() first"});
    }

    try {
        // Fetch image pair.
        const auto pair = client_->get_image_pair();

        // Decode raw bytes into cv::Mat via ImageFactory.
        auto left_img  = ImageFactory::create(pair.left);
        auto right_img = ImageFactory::create(pair.right);

        // Convert to grayscale according to image type.
        auto to_gray = [](ImageRPC::Type type,
                          cv::Mat mat) -> std::expected<cv::Mat, Error> {
            cv::Mat gray;
            switch (type) {
            case ImageRPC::Type::BGR:
                cv::cvtColor(mat, gray, cv::COLOR_BGR2GRAY);
                return gray;
            case ImageRPC::Type::RGBA:
                cv::cvtColor(mat, gray, cv::COLOR_RGBA2GRAY);
                return gray;
            case ImageRPC::Type::DEPTH:
                return std::unexpected(
                    Error{"depth images not supported by stereo pipeline"});
            default:
                return std::unexpected(Error{"unknown image type"});
            }
        };

        auto left_gray_r = to_gray(pair.left.type, left_img.get());
        if (!left_gray_r)
            return std::unexpected(left_gray_r.error());

        auto right_gray_r = to_gray(pair.right.type, right_img.get());
        if (!right_gray_r)
            return std::unexpected(right_gray_r.error());

        const cv::Mat &left_gray  = *left_gray_r;
        const cv::Mat &right_gray = *right_gray_r;

        // Rectify, compute disparity, reproject.
        auto [rect_left, rect_right] =
            rectifier_->rectify(left_gray, right_gray);
        const auto disparity = matcher_->compute(rect_left, rect_right);
        Frame frame;
        frame.cloud = builder_->build(disparity);

        if (with_colour) {
            // Only compute_frame() pays for the colour rectification.  The
            // type switch above already rejected anything but BGR / RGBA.
            cv::Mat left_bgr;
            if (pair.left.type == ImageRPC::Type::RGBA) {
                cv::cvtColor(left_img.get(), left_bgr, cv::COLOR_RGBA2BGR);
            } else {
                left_bgr = left_img.get();
            }
            frame.rectified_left_bgr =
                rectifier_->rectify(left_bgr, left_bgr).first;
        }
        return frame;

    } catch (const jsonrpccxx::JsonRpcException &e) {
        return std::unexpected(Error{e.what()});
    } catch (const std::exception &e) {
        return std::unexpected(Error{e.what()});
    }
}

// ---------------------------------------------------------------------------
// PLY helper
// ---------------------------------------------------------------------------

namespace {

struct Vertex {
    float x, y, z;
};

/// Longest edge of a triangle must not exceed max_ratio * mean depth.
bool triangle_ok(const Vertex &a, const Vertex &b, const Vertex &c,
                 float max_ratio) {
    const auto dist2 = [](const Vertex &p, const Vertex &q) {
        const float dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
        return dx * dx + dy * dy + dz * dz;
    };
    const float longest2 = std::max({dist2(a, b), dist2(b, c), dist2(c, a)});
    const float limit    = max_ratio * (a.z + b.z + c.z) / 3.0f;
    return longest2 <= limit * limit;
}

} // namespace

size_t write_ply(const PointCloud &cloud, const std::string &path,
                 const PlyOptions &opts) {
    if (opts.stride < 1) {
        throw std::invalid_argument("write_ply: stride must be >= 1, got " +
                                    std::to_string(opts.stride));
    }
    // Decimated grid dimensions; pixel (r, c) of the sub-grid maps to
    // (r * stride, c * stride) of the source cloud.
    const int grid_w = (cloud.width + opts.stride - 1) / opts.stride;
    const int grid_h = (cloud.height + opts.stride - 1) / opts.stride;

    // Map every valid sub-grid cell to its index in the vertex list.  With
    // stride > 1 each vertex is the mean of the valid pixels in its
    // stride x stride block, which divides the per-pixel disparity noise by
    // roughly the stride and yields a far smoother mesh than sub-sampling.
    const size_t n_px =
        static_cast<size_t>(grid_w) * static_cast<size_t>(grid_h);
    std::vector<int> index(n_px, -1);
    std::vector<Vertex> vertices;
    vertices.reserve(n_px);
    for (int gr = 0; gr < grid_h; ++gr) {
        for (int gc = 0; gc < grid_w; ++gc) {
            double sx = 0.0, sy = 0.0, sz = 0.0;
            int count           = 0;
            const int row_begin = gr * opts.stride;
            const int col_begin = gc * opts.stride;
            const int row_end = std::min(row_begin + opts.stride, cloud.height);
            const int col_end = std::min(col_begin + opts.stride, cloud.width);
            for (int r = row_begin; r < row_end; ++r) {
                for (int c = col_begin; c < col_end; ++c) {
                    const size_t src = (static_cast<size_t>(r) *
                                            static_cast<size_t>(cloud.width) +
                                        static_cast<size_t>(c)) *
                                       3u;
                    const float x    = cloud.data[src];
                    const float y    = cloud.data[src + 1];
                    const float z    = cloud.data[src + 2];
                    if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                        sx += x;
                        sy += y;
                        sz += z;
                        ++count;
                    }
                }
            }
            // Require at least half the block to be valid so that a lone
            // pixel cannot fabricate a vertex inside a hole.
            const int block = (row_end - row_begin) * (col_end - col_begin);
            if (count * 2 >= block && count > 0) {
                index[static_cast<size_t>(gr) * static_cast<size_t>(grid_w) +
                      static_cast<size_t>(gc)] =
                    static_cast<int>(vertices.size());
                vertices.push_back({static_cast<float>(sx / count),
                                    static_cast<float>(sy / count),
                                    static_cast<float>(sz / count)});
            }
        }
    }

    // Grid triangulation: each 2x2 cell yields up to two triangles, wound so
    // the normal faces the camera (-z in OpenCV coordinates).
    std::vector<std::array<int, 3>> faces;
    if (opts.triangulate) {
        faces.reserve(2 * n_px);
        const auto at = [&](int r, int c) {
            return index[static_cast<size_t>(r) * static_cast<size_t>(grid_w) +
                         static_cast<size_t>(c)];
        };
        const auto emit = [&](int i0, int i1, int i2) {
            if (i0 < 0 || i1 < 0 || i2 < 0)
                return;
            if (triangle_ok(vertices[static_cast<size_t>(i0)],
                            vertices[static_cast<size_t>(i1)],
                            vertices[static_cast<size_t>(i2)],
                            opts.max_edge_depth_ratio)) {
                faces.push_back({i0, i1, i2});
            }
        };
        for (int r = 0; r + 1 < grid_h; ++r) {
            for (int c = 0; c + 1 < grid_w; ++c) {
                const int i00 = at(r, c), i01 = at(r, c + 1);
                const int i10 = at(r + 1, c), i11 = at(r + 1, c + 1);
                const int valid =
                    (i00 >= 0) + (i01 >= 0) + (i10 >= 0) + (i11 >= 0);
                if (valid == 4) {
                    emit(i00, i10, i11);
                    emit(i00, i11, i01);
                } else if (valid == 3) {
                    // One missing corner: keep the single remaining triangle.
                    if (i00 < 0)
                        emit(i10, i11, i01);
                    else if (i01 < 0)
                        emit(i00, i10, i11);
                    else if (i10 < 0)
                        emit(i00, i11, i01);
                    else
                        emit(i00, i10, i01);
                }
            }
        }
    }

    std::ofstream out(path, std::ios::binary);
    out << "ply\n"
        << (opts.binary ? "format binary_little_endian 1.0\n"
                        : "format ascii 1.0\n")
        << "element vertex " << vertices.size() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n";
    if (opts.triangulate) {
        out << "element face " << faces.size() << "\n"
            << "property list uchar int vertex_indices\n";
    }
    out << "end_header\n";

    if (opts.binary) {
        static_assert(std::endian::native == std::endian::little,
                      "binary PLY writer assumes a little-endian host");
        static_assert(sizeof(Vertex) == 3 * sizeof(float));
        out.write(
            reinterpret_cast<const char *>(vertices.data()),
            static_cast<std::streamsize>(vertices.size() * sizeof(Vertex)));
        for (const auto &f : faces) {
            const unsigned char count = 3;
            out.write(reinterpret_cast<const char *>(&count), 1);
            out.write(reinterpret_cast<const char *>(f.data()),
                      static_cast<std::streamsize>(3 * sizeof(int)));
        }
    } else {
        for (const auto &v : vertices) {
            out << v.x << " " << v.y << " " << v.z << "\n";
        }
        for (const auto &f : faces) {
            out << "3 " << f[0] << " " << f[1] << " " << f[2] << "\n";
        }
    }
    return faces.size();
}

// ---------------------------------------------------------------------------
// Viewer frame export (CPV1)
// ---------------------------------------------------------------------------

namespace {

/// Quantise metres to 0.02 mm units; non-finite -> INT16_MIN.
constexpr float kViewerUnitM     = 2e-5f;
constexpr int16_t kViewerInvalid = std::numeric_limits<int16_t>::min();

int16_t quantise_viewer(float value_m) {
    if (!std::isfinite(value_m))
        return kViewerInvalid;
    const long units = std::lround(value_m / kViewerUnitM);
    return static_cast<int16_t>(std::clamp(units, -32767L, 32767L));
}

void append_u16(std::vector<char> &buf, uint16_t v) {
    buf.push_back(static_cast<char>(v & 0xffu));
    buf.push_back(static_cast<char>((v >> 8) & 0xffu));
}

void append_i16(std::vector<char> &buf, int16_t v) {
    append_u16(buf, static_cast<uint16_t>(v));
}

/// Validate an optional CV_32FC3 / CV_8UC3 input against the cloud size.
/// Returns nullptr when the input is absent (null or empty).
const cv::Mat *check_viewer_input(const cv::Mat *mat, int type,
                                  const char *name, const PointCloud &cloud) {
    if (mat == nullptr || mat->empty())
        return nullptr;
    if (mat->type() != type) {
        throw std::invalid_argument(
            std::string("encode_viewer_frame: ") + name + " must be " +
            (type == CV_32FC3 ? "CV_32FC3" : "CV_8UC3") + ", got type " +
            std::to_string(mat->type()));
    }
    if (mat->cols != cloud.width || mat->rows != cloud.height) {
        throw std::invalid_argument(
            std::string("encode_viewer_frame: ") + name + " size " +
            std::to_string(mat->cols) + "x" + std::to_string(mat->rows) +
            " does not match cloud " + std::to_string(cloud.width) + "x" +
            std::to_string(cloud.height));
    }
    return mat;
}

} // namespace

std::string encode_viewer_frame(const PointCloud &cloud,
                                const cv::Mat *gt_rectified,
                                const cv::Mat *rgb_rectified_bgr,
                                const ViewerFrameOptions &opts) {
    if (opts.stride < 1) {
        throw std::invalid_argument(
            "encode_viewer_frame: stride must be >= 1, got " +
            std::to_string(opts.stride));
    }
    if (cloud.width <= 0 || cloud.height <= 0) {
        throw std::invalid_argument("encode_viewer_frame: empty cloud");
    }
    if (cloud.data.size() != static_cast<size_t>(cloud.width) *
                                 static_cast<size_t>(cloud.height) * 3u) {
        throw std::invalid_argument(
            "encode_viewer_frame: cloud data size does not match width * "
            "height * 3");
    }
    const int grid_w = cloud.width / opts.stride;
    const int grid_h = cloud.height / opts.stride;
    if (grid_w < 1 || grid_h < 1) {
        throw std::invalid_argument(
            "encode_viewer_frame: stride " + std::to_string(opts.stride) +
            " leaves no samples in a " + std::to_string(cloud.width) + "x" +
            std::to_string(cloud.height) + " cloud");
    }
    if (grid_w > 65535 || grid_h > 65535) {
        throw std::invalid_argument(
            "encode_viewer_frame: sampled grid exceeds 65535 in one direction");
    }
    const cv::Mat *gt =
        check_viewer_input(gt_rectified, CV_32FC3, "gt_rectified", cloud);
    const cv::Mat *rgb = check_viewer_input(rgb_rectified_bgr, CV_8UC3,
                                            "rgb_rectified_bgr", cloud);

    const size_t n_samples =
        static_cast<size_t>(grid_w) * static_cast<size_t>(grid_h);
    std::vector<char> buf;
    buf.reserve(12u + n_samples * 3u * (2u + (gt ? 2u : 0u) + (rgb ? 1u : 0u)));

    // Header.
    buf.insert(buf.end(), {'C', 'P', 'V', '1'});
    append_u16(buf, static_cast<uint16_t>(grid_w));
    append_u16(buf, static_cast<uint16_t>(grid_h));
    append_u16(buf, static_cast<uint16_t>(opts.stride));
    buf.push_back(static_cast<char>((gt ? 1u : 0u) | (rgb ? 2u : 0u)));
    buf.push_back(0); // reserved

    // A pixel with any non-finite component is invalid on all three.
    const auto append_xyz = [&](float x, float y, float z) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            append_i16(buf, kViewerInvalid);
            append_i16(buf, kViewerInvalid);
            append_i16(buf, kViewerInvalid);
            return;
        }
        append_i16(buf, quantise_viewer(x));
        append_i16(buf, quantise_viewer(y));
        append_i16(buf, quantise_viewer(z));
    };

    // Reconstructed XYZ.
    for (int gr = 0; gr < grid_h; ++gr) {
        for (int gc = 0; gc < grid_w; ++gc) {
            const size_t src =
                (static_cast<size_t>(gr) * static_cast<size_t>(opts.stride) *
                     static_cast<size_t>(cloud.width) +
                 static_cast<size_t>(gc) * static_cast<size_t>(opts.stride)) *
                3u;
            append_xyz(cloud.data[src], cloud.data[src + 1],
                       cloud.data[src + 2]);
        }
    }
    // Ground truth XYZ.
    if (gt) {
        for (int gr = 0; gr < grid_h; ++gr) {
            for (int gc = 0; gc < grid_w; ++gc) {
                const auto &p =
                    gt->at<cv::Vec3f>(gr * opts.stride, gc * opts.stride);
                append_xyz(p[0], p[1], p[2]);
            }
        }
    }
    // Colour, stored as R,G,B (input is BGR).
    if (rgb) {
        for (int gr = 0; gr < grid_h; ++gr) {
            for (int gc = 0; gc < grid_w; ++gc) {
                const auto &p =
                    rgb->at<cv::Vec3b>(gr * opts.stride, gc * opts.stride);
                buf.push_back(static_cast<char>(p[2]));
                buf.push_back(static_cast<char>(p[1]));
                buf.push_back(static_cast<char>(p[0]));
            }
        }
    }

    std::string encoded((buf.size() + 2) / 3 * 4, '\0');
    size_t encoded_len = 0;
    base64_encode(buf.data(), buf.size(), encoded.data(), &encoded_len, 0);
    encoded.resize(encoded_len);
    return encoded;
}

void write_viewer_frame_js(const std::string &path, const std::string &id,
                           const std::string &base64) {
    const bool id_ok =
        !id.empty() && std::all_of(id.begin(), id.end(), [](char ch) {
            return (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
                   (ch >= '0' && ch <= '9') || ch == '_' || ch == '.' ||
                   ch == '-';
        });
    if (!id_ok) {
        throw std::invalid_argument(
            "write_viewer_frame_js: frame id must match [A-Za-z0-9_.-]+, "
            "got \"" +
            id + "\"");
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("write_viewer_frame_js: cannot open " + path);
    }
    out << "CloudViewer.addFrame(\"" << id << "\", \"" << base64 << "\");\n";
    if (!out) {
        throw std::runtime_error("write_viewer_frame_js: failed to write " +
                                 path);
    }
}

} // namespace score

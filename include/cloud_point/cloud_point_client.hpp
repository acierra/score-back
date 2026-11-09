#pragma once

#include "cloud_point/point_cloud_builder.hpp"
#include "cloud_point/stereo_matcher.hpp"
#include "cloud_point/stereo_matcher_factory.hpp"
#include "cloud_point/stereo_preprocessor.hpp"
#include "cloud_point/stereo_rectifier.hpp"
#include <expected>
#include <memory>
#include <string>

namespace score {

class TCPConnector;
class RpcClient;

/// @brief End-to-end stereo point cloud client.
///
/// Connects to a JSON-RPC stereo server, fetches calibration once on connect(),
/// then produces point clouds on demand via compute_cloud().
///
/// Thread safety: not thread-safe. Do not call methods concurrently.
class CloudPointClient {
  public:
    /// @brief Recoverable per-frame error.
    struct Error {
        std::string message;
    };

    /// @brief Construct client (does not connect).
    /// @param ip               Server IP address.
    /// @param port             Server port.
    /// @param algo             Stereo matching algorithm (GPU falls back to CPU
    /// if
    ///                         unavailable).
    /// @param opts             Depth filtering options.
    /// @param num_disparities  SGBM disparity levels (default 128; use 160 for
    ///                         small-baseline rigs such as SCARED).
    /// @param matcher_params   SGBM tuning / post-filters for the CPU matcher.
    /// @param preprocessing    Ordered chain after rectification; default none.
    CloudPointClient(
        std::string ip, int port,
        StereoAlgorithmType algo        = StereoAlgorithmType::GPU,
        PointCloudBuilder::Options opts = {}, int num_disparities = 128,
        CpuStereoMatcher::Params matcher_params = CpuStereoMatcher::Params{},
        PreprocessingConfig preprocessing       = {});

    ~CloudPointClient();

    /// @brief Connect to server and fetch calibration once.
    /// @throws std::runtime_error on connection or calibration failure.
    void connect();

    /// @brief Return true if connected and all pipeline components are ready.
    [[nodiscard]] bool connected() const noexcept;

    /// @brief One reconstructed frame: the cloud plus the colour image it was
    ///        computed from, both in the rectified-left pixel grid.
    struct Frame {
        PointCloud cloud;
        /// CV_8UC3 BGR rectified left image; same width/height as cloud, so
        /// pixel (r, c) is the colour of cloud point (r, c).
        cv::Mat rectified_left_bgr;
    };

    /// @brief Compute one point cloud: fetch image pair → gray → rectify →
    ///        preprocessing → disparity → reproject.
    ///
    /// Equivalent to compute_frame() without the colour image (and without
    /// paying for its rectification).
    ///
    /// Recoverable per-frame failures (RPC error, decode error) are returned as
    /// Error. Calling before connect() returns Error immediately.
    [[nodiscard]] std::expected<PointCloud, Error> compute_cloud();

    /// @brief Compute one point cloud together with the rectified left colour
    ///        image (for viewer export, see encode_viewer_frame()).
    ///
    /// Same pipeline and failure modes as compute_cloud(); additionally
    /// rectifies the left colour image (~5 ms).
    [[nodiscard]] std::expected<Frame, Error> compute_frame();

  private:
    /// Shared pipeline; the colour image is produced only when requested.
    [[nodiscard]] std::expected<Frame, Error> compute(bool with_colour);

    std::string ip_;
    int port_;
    StereoAlgorithmType algo_;
    PointCloudBuilder::Options opts_;
    int num_disparities_;
    CpuStereoMatcher::Params matcher_params_;
    StereoPreprocessorChain preprocessor_;
    std::unique_ptr<TCPConnector> connector_;
    std::unique_ptr<RpcClient> client_;
    std::unique_ptr<StereoRectifier> rectifier_;
    std::unique_ptr<IStereoMatcher> matcher_;
    std::unique_ptr<PointCloudBuilder> builder_;
};

/// @brief PLY export settings.
struct PlyOptions {
    /// Emit a triangle mesh built from the organised pixel grid.  Web viewers
    /// (e.g. Meshy) fabricate triangles from consecutive vertices of a
    /// vertex-only PLY, which draws long slivers across the surface; a real
    /// mesh renders correctly everywhere and still contains every point.
    bool triangulate;
    /// Reject triangles whose longest edge exceeds this fraction of the
    /// triangle's mean depth (breaks the mesh at occlusion boundaries).
    float max_edge_depth_ratio;
    /// Write binary_little_endian instead of ASCII.  A full 1280x1024 mesh
    /// is ~40 MB in binary versus ~85 MB in ASCII and parses much faster.
    bool binary;
    /// Decimate the grid by this factor (1 = full resolution).  Each output
    /// vertex is the mean of the valid pixels in its stride x stride block
    /// (blocks less than half valid are dropped), so stride 4 both shrinks
    /// the mesh 16x and cuts per-pixel disparity noise ~4x.  Web viewers
    /// cope far better with such a mesh than with the raw 1.1 M vertices.
    int stride;
    PlyOptions() noexcept
        : triangulate(true), max_edge_depth_ratio(0.05f), binary(true),
          stride(1) {}
    PlyOptions(bool tri, float ratio, bool bin = true, int step = 1) noexcept
        : triangulate(tri), max_edge_depth_ratio(ratio), binary(bin),
          stride(step) {}
};

/// @brief Write valid points (and optionally a grid-triangulated mesh) as
///        PLY for MeshLab / web-viewer inspection.
/// @param cloud  Source point cloud.
/// @param path   Output file path.
/// @param opts   Export settings; defaults to a binary triangulated mesh.
/// @return       Number of faces written (0 when not triangulating).
/// @throws std::invalid_argument if opts.stride < 1.
size_t write_ply(const PointCloud &cloud, const std::string &path,
                 const PlyOptions &opts = PlyOptions{});

/// @brief Viewer frame export settings (see encode_viewer_frame()).
struct ViewerFrameOptions {
    /// Sample every stride-th pixel in each direction (1 = every pixel).
    /// Samples are raw pixel values, never averaged, so error statistics
    /// recomputed from the export stay honest.  Stride 4 on a 1280x1024
    /// cloud gives ~1.2 MB (~1.6 MB base64) per frame with ground truth.
    int stride;
    ViewerFrameOptions() noexcept : stride(4) {}
    explicit ViewerFrameOptions(int step) noexcept : stride(step) {}
};

/// @brief Encode a cloud (plus optional ground truth and colour) as a base64
///        "CPV1" payload for the offline web viewer.
///
/// Binary layout (little-endian):
///   - `"CPV1"`, `u16 width`, `u16 height`, `u16 stride`, `u8 flags`
///     (bit0 = has ground truth, bit1 = has RGB), `u8 reserved` (0);
///   - `int16 xyz[width*height*3]` reconstructed XYZ, row-major, sample
///     (r, c) = cloud pixel (r*stride, c*stride);
///   - `int16 gt_xyz[width*height*3]` if bit0, ground truth at the same pixel;
///   - `u8 rgb[width*height*3]` if bit1, colour at the same pixel as R,G,B.
///
/// width = floor(cloud.width / stride), height = floor(cloud.height / stride).
/// XYZ are in units of 0.02 mm (rounded to nearest, clamped to ±32767, i.e.
/// ±655 mm); a pixel with any non-finite component is stored as INT16_MIN on
/// all three components.
///
/// @param cloud             Source cloud (rectified-left frame, metres).
/// @param gt_rectified      Optional CV_32FC3 rectified ground-truth point
///                          map of the cloud's size (metres); nullptr or an
///                          empty Mat means no ground truth.
/// @param rgb_rectified_bgr Optional CV_8UC3 BGR rectified left image of
///                          the cloud's size; nullptr or an empty Mat means no
///                          colour.
/// @param opts              Sampling stride.
/// @return                  Standard base64 (padded, no line breaks).
/// @throws std::invalid_argument if opts.stride < 1, the cloud is empty or
///         malformed, the sampled grid is empty or exceeds 65535 in either
///         direction, or an optional input has the wrong type or size.
std::string
encode_viewer_frame(const PointCloud &cloud, const cv::Mat *gt_rectified,
                    const cv::Mat *rgb_rectified_bgr,
                    const ViewerFrameOptions &opts = ViewerFrameOptions{});

/// @brief Write a viewer frame script `CloudViewer.addFrame("<id>", "<b64>");`.
///
/// The viewer site loads such files as `frames/<id>.js` via a `<script>`
/// tag, which works from `file://` where fetch() does not.
/// @param path    Output file path.
/// @param id      Frame identifier; must match `[A-Za-z0-9_.-]+`.
/// @param base64  Payload from encode_viewer_frame().
/// @throws std::invalid_argument if id is empty or contains other characters.
/// @throws std::runtime_error if the file cannot be written.
void write_viewer_frame_js(const std::string &path, const std::string &id,
                           const std::string &base64);

} // namespace score

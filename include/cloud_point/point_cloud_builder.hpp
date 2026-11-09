#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <opencv2/core.hpp>
#include <vector>

namespace score {

/// @brief Dense point cloud in row-major XYZ layout.
///
/// Invalid (occluded or out-of-range) points are represented as quiet NaN on
/// all three coordinates.  valid_points() filters them out.
struct PointCloud {
    int width{0};
    int height{0};
    std::vector<float> data; ///< width * height * 3 floats, XYZ row-major

    /// @brief Return only the non-NaN points.
    [[nodiscard]] std::vector<std::array<float, 3>> valid_points() const;
};

/// @brief Reprojects a disparity map to a 3-D point cloud using the
///        reprojection matrix Q produced by cv::stereoRectify.
///
/// Pipeline inside build():
///   1. disparity offset correction (Options::disparity_offset_px);
///   2. reprojection with Q;
///   3. depth-range filter ([min_depth_m, max_depth_m]);
///   4. optional neighbourhood-consistency outlier filter
///      (Options::outlier_filter).
///
/// Thread safety: build() is const and safe to call concurrently once the
/// object is constructed.
class PointCloudBuilder {
  public:
    /// @brief Why a pixel is (in)valid in the cloud returned by build().
    ///
    /// Stored as CV_8U in the optional reason map, one entry per pixel.
    enum class Reason : std::uint8_t {
        valid            = 0,   ///< point is present in the cloud
        matcher_rejected = 1,   ///< raw or offset-corrected disparity ≤ 0, or
                                ///< the reprojection is non-finite / sentinel
        depth_range        = 2, ///< z outside [min_depth_m, max_depth_m]
        outlier_filter     = 3, ///< removed by the outlier filter
        preprocessing_mask = 4, ///< pipeline excluded a masked correspondence
    };

    /// @brief Human-readable, snake_case name of a reason code.
    [[nodiscard]] static const char *reason_name(Reason r) noexcept;

    /// @brief Depth filtering thresholds and post-processing options.
    struct Options {
        /// @brief Neighbourhood-consistency outlier filter (design D5).
        ///
        /// For each valid pixel the valid z of the neighbours in a
        /// window × window neighbourhood (excluding the pixel itself) are
        /// gathered.  The pixel is invalidated when fewer than
        /// min_neighbours are found, or when |z − median| exceeds
        /// max(abs_threshold_m, rel_threshold · median).  Decisions are made
        /// against the unfiltered cloud, so the result does not depend on
        /// scan order.  The filter only invalidates points; it never moves
        /// or adds one.
        struct OutlierFilter {
            bool enabled;
            int window;             ///< odd, ≥ 3
            double abs_threshold_m; ///< ≥ 0
            double rel_threshold;   ///< ≥ 0, multiplies the median depth
            int min_neighbours;     ///< 0 ≤ min_neighbours < window²
            OutlierFilter() noexcept
                : enabled(false), window(5), abs_threshold_m(0.002),
                  rel_threshold(0.03), min_neighbours(6) {}
        };

        float min_depth_m;
        float max_depth_m;
        /// @brief Added to every raw disparity > 0 before reprojection
        ///        (corrected = raw + offset).  A corrected value ≤ 0 makes
        ///        the pixel invalid.  With 0 the disparity map is untouched.
        double disparity_offset_px;
        OutlierFilter outlier_filter;

        // Explicit constructor avoids a GCC limitation with nested-struct
        // default-member-initialisers used as default function arguments.
        Options() noexcept
            : min_depth_m(0.01f), max_depth_m(10.0f), disparity_offset_px(0.0),
              outlier_filter() {}
        Options(float min_m, float max_m) noexcept
            : min_depth_m(min_m), max_depth_m(max_m), disparity_offset_px(0.0),
              outlier_filter() {}
    };

    /// @brief Construct builder.
    /// @param q     4x4 CV_64F reprojection matrix from cv::stereoRectify.
    /// @param opts  Depth-range filter, disparity offset and outlier filter.
    /// @throws std::invalid_argument if q is not 4x4 CV_64F, if
    ///         disparity_offset_px is not finite, or if the outlier filter is
    ///         enabled with an even or < 3 window, a negative threshold, or
    ///         min_neighbours outside [0, window²).
    explicit PointCloudBuilder(cv::Mat q, Options opts = Options{});

    /// @brief Reproject disparity to a point cloud.
    ///
    /// Accepts:
    ///   - CV_16S  SGBM fixed-point output (values = disparity * 16)
    ///   - CV_32F  already in pixel units
    ///
    /// Invalid points (disparity ≤ 0 before or after the offset correction,
    /// |z| ≥ 10000, z outside [min_depth_m, max_depth_m], or rejected by the
    /// outlier filter) are stored as quiet NaN.
    ///
    /// @param disparity  Disparity map (CV_16S or CV_32F).
    /// @param reason     Optional out-parameter.  When non-null it receives a
    ///                   CV_8U map of the same size as @p disparity holding a
    ///                   Reason code per pixel.
    /// @return           Populated PointCloud.
    /// @throws std::invalid_argument on unsupported disparity type.
    [[nodiscard]] PointCloud build(const cv::Mat &disparity,
                                   cv::Mat *reason = nullptr) const;

    /// @brief Options this builder was constructed with.
    [[nodiscard]] const Options &options() const noexcept { return opts_; }

  private:
    cv::Mat q_;
    Options opts_;
};

} // namespace score

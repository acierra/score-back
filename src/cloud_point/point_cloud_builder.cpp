#include "cloud_point/point_cloud_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>

namespace score {

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

using Reason = PointCloudBuilder::Reason;

/// @brief Median of the first @p n entries of @p v (reordered in place).
double median_of(float *v, std::size_t n) {
    float *const mid = v + n / 2;
    if (n <= 32) {
        // Small windows: a straight insertion sort beats introselect.
        for (std::size_t i = 1; i < n; ++i) {
            const float key = v[i];
            std::size_t j   = i;
            while (j > 0 && v[j - 1] > key) {
                v[j] = v[j - 1];
                --j;
            }
            v[j] = key;
        }
        if (n % 2 == 1) {
            return static_cast<double>(*mid);
        }
        return 0.5 * (static_cast<double>(mid[-1]) + static_cast<double>(*mid));
    }
    std::nth_element(v, mid, v + n);
    if (n % 2 == 1) {
        return static_cast<double>(*mid);
    }
    // Even count: average of the two middle values.  After nth_element every
    // element before `mid` is ≤ *mid, so the lower middle is their maximum.
    const float lower = *std::max_element(v, mid);
    return 0.5 * (static_cast<double>(lower) + static_cast<double>(*mid));
}

/// @brief Append the valid (non-NaN) z of columns [c0, c1] of one cloud row
///        to @p out, skipping column @p skip (or none when skip < 0).
inline std::size_t gather_row(const float *row, int c0, int c1, int skip,
                              float *out, std::size_t n) {
    for (int cc = c0; cc <= c1; ++cc) {
        if (cc == skip) {
            continue;
        }
        const float nz = row[static_cast<std::size_t>(cc) * 3u + 2];
        out[n] = nz;
        n += static_cast<std::size_t>(!std::isnan(nz)); // branchless append
    }
    return n;
}

/// @brief Neighbourhood-consistency filter (design D5).
///
/// Reads @p in (unfiltered), writes the invalidations into @p out, which must
/// start as a copy of @p in.  Rows run under cv::parallel_for_; each row of
/// @p out (and of @p reason, when provided) is written by exactly one task.
void apply_outlier_filter(const PointCloud &in, PointCloud &out,
                          const PointCloudBuilder::Options::OutlierFilter &f,
                          cv::Mat &reason) {
    const int width  = in.width;
    const int height = in.height;
    const int radius = f.window / 2;
    const double abs_thr = f.abs_threshold_m;
    const double rel_thr = f.rel_threshold;
    const std::size_t max_neighbours =
        static_cast<std::size_t>(f.window) * static_cast<std::size_t>(f.window);

    const float *src = in.data.data();
    float *dst       = out.data.data();

    cv::parallel_for_(cv::Range(0, height), [&](const cv::Range &rows) {
        // One slot of slack: gather_row writes before it decides to keep.
        std::vector<float> zs(max_neighbours + 1);

        for (int r = rows.start; r < rows.end; ++r) {
            std::uint8_t *reason_row =
                reason.empty() ? nullptr : reason.ptr<std::uint8_t>(r);
            const int r0 = std::max(0, r - radius);
            const int r1 = std::min(height - 1, r + radius);

            for (int c = 0; c < width; ++c) {
                const std::size_t idx =
                    (static_cast<std::size_t>(r) *
                         static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(c)) *
                    3u;
                const float z = src[idx + 2];
                if (std::isnan(z)) {
                    continue; // already invalid; keep its reason
                }

                const int c0 = std::max(0, c - radius);
                const int c1 = std::min(width - 1, c + radius);

                std::size_t n = 0;
                for (int rr = r0; rr <= r1; ++rr) {
                    const float *row =
                        src + (static_cast<std::size_t>(rr) *
                               static_cast<std::size_t>(width)) *
                                  3u;
                    n = gather_row(row, c0, c1, rr == r ? c : -1, zs.data(), n);
                }

                bool reject = n < static_cast<std::size_t>(f.min_neighbours);
                if (!reject && n > 0) {
                    // Fast path (exact): the median lies in [mn, mx] and
                    // rel·median ≥ rel·mn, so when every neighbour is within
                    // max(abs, rel·mn) of z the pixel is certainly kept.
                    float mn = zs[0], mx = zs[0];
                    for (std::size_t i = 1; i < n; ++i) {
                        mn = std::min(mn, zs[i]);
                        mx = std::max(mx, zs[i]);
                    }
                    const double zd    = static_cast<double>(z);
                    const double t_min = std::max(abs_thr, rel_thr * mn);
                    const bool certainly_ok =
                        (mx - zd) <= t_min && (zd - mn) <= t_min;
                    if (!certainly_ok) {
                        const double med = median_of(zs.data(), n);
                        const double tol = std::max(abs_thr, rel_thr * med);
                        reject = std::abs(zd - med) > tol;
                    }
                }

                if (reject) {
                    dst[idx]     = kNaN;
                    dst[idx + 1] = kNaN;
                    dst[idx + 2] = kNaN;
                    if (reason_row) {
                        reason_row[c] =
                            static_cast<std::uint8_t>(Reason::outlier_filter);
                    }
                }
            }
        }
    });
}

} // namespace

// ---------------------------------------------------------------------------
// PointCloud::valid_points
// ---------------------------------------------------------------------------

std::vector<std::array<float, 3>> PointCloud::valid_points() const {
    std::vector<std::array<float, 3>> pts;
    pts.reserve(static_cast<size_t>(width * height));
    for (int i = 0; i < height * width; ++i) {
        const float x = data[static_cast<size_t>(i) * 3];
        const float y = data[static_cast<size_t>(i) * 3 + 1];
        const float z = data[static_cast<size_t>(i) * 3 + 2];
        if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
            pts.push_back({x, y, z});
        }
    }
    return pts;
}

// ---------------------------------------------------------------------------
// PointCloudBuilder::reason_name
// ---------------------------------------------------------------------------

const char *PointCloudBuilder::reason_name(Reason r) noexcept {
    switch (r) {
    case Reason::valid:
        return "valid";
    case Reason::matcher_rejected:
        return "matcher_rejected";
    case Reason::depth_range:
        return "depth_range";
    case Reason::outlier_filter:
        return "outlier_filter";
    case Reason::preprocessing_mask:
        return "preprocessing_mask";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// PointCloudBuilder constructor
// ---------------------------------------------------------------------------

PointCloudBuilder::PointCloudBuilder(cv::Mat q, Options opts)
    : q_(std::move(q)), opts_(opts) {
    if (q_.rows != 4 || q_.cols != 4 || q_.type() != CV_64F) {
        throw std::invalid_argument("Q matrix must be 4x4 CV_64F, got " +
                                    std::to_string(q_.rows) + "x" +
                                    std::to_string(q_.cols) +
                                    " type=" + std::to_string(q_.type()));
    }
    if (!std::isfinite(opts_.disparity_offset_px)) {
        throw std::invalid_argument("disparity_offset_px must be finite");
    }
    const auto &f = opts_.outlier_filter;
    if (f.enabled) {
        if (f.window < 3 || f.window % 2 == 0) {
            throw std::invalid_argument(
                "outlier_filter.window must be odd and >= 3, got " +
                std::to_string(f.window));
        }
        if (!(f.abs_threshold_m >= 0.0)) {
            throw std::invalid_argument(
                "outlier_filter.abs_threshold_m must be >= 0, got " +
                std::to_string(f.abs_threshold_m));
        }
        if (!(f.rel_threshold >= 0.0)) {
            throw std::invalid_argument(
                "outlier_filter.rel_threshold must be >= 0, got " +
                std::to_string(f.rel_threshold));
        }
        const long long max_n = static_cast<long long>(f.window) * f.window;
        if (f.min_neighbours < 0 || f.min_neighbours >= max_n) {
            throw std::invalid_argument(
                "outlier_filter.min_neighbours must be in [0, window^2), got " +
                std::to_string(f.min_neighbours) + " for window " +
                std::to_string(f.window));
        }
    }
}

// ---------------------------------------------------------------------------
// PointCloudBuilder::build
// ---------------------------------------------------------------------------

PointCloud PointCloudBuilder::build(const cv::Mat &disparity,
                                    cv::Mat *reason) const {
    cv::Mat disp32;
    if (disparity.type() == CV_16S) {
        // SGBM stores disparity in fixed-point with a factor of 16
        disparity.convertTo(disp32, CV_32F, 1.0 / 16.0);
    } else if (disparity.type() == CV_32F) {
        disp32 = disparity;
    } else {
        throw std::invalid_argument(
            "Disparity must be CV_16S or CV_32F, got type=" +
            std::to_string(disparity.type()));
    }

    // Disparity offset correction (design D2): corrected = raw + δ for raw > 0
    // only, so the matcher's invalid sentinel stays invalid.  With δ == 0 the
    // map is left untouched so the output is bit-identical to no correction.
    if (opts_.disparity_offset_px != 0.0) {
        cv::Mat corrected(disp32.size(), CV_32F);
        const double offset = opts_.disparity_offset_px;
        for (int r = 0; r < disp32.rows; ++r) {
            const float *src = disp32.ptr<float>(r);
            float *dst       = corrected.ptr<float>(r);
            for (int c = 0; c < disp32.cols; ++c) {
                const float d = src[c];
                dst[c] = (d > 0.0f)
                             ? static_cast<float>(static_cast<double>(d) + offset)
                             : d;
            }
        }
        disp32 = corrected;
    }

    // Reproject to 3-D.  handleMissingValues=false is used intentionally:
    // OpenCV 4.6 incorrectly marks all CV_32F pixels as "missing" when
    // handleMissingValues=true (empirically verified).  Our own filter loop
    // below already covers all sentinel cases (d≤0, |z|≥10000, depth range).
    cv::Mat xyz;
    cv::reprojectImageTo3D(disp32, xyz, q_, /*handleMissingValues=*/false);

    PointCloud cloud;
    cloud.width  = disparity.cols;
    cloud.height = disparity.rows;
    cloud.data.resize(static_cast<size_t>(cloud.width) *
                      static_cast<size_t>(cloud.height) * 3u);

    cv::Mat reasons;
    if (reason != nullptr) {
        reasons.create(cloud.height, cloud.width, CV_8U);
    }

    for (int r = 0; r < cloud.height; ++r) {
        std::uint8_t *reason_row =
            reasons.empty() ? nullptr : reasons.ptr<std::uint8_t>(r);
        for (int c = 0; c < cloud.width; ++c) {
            const auto &pt = xyz.at<cv::Vec3f>(r, c);
            const float z  = pt[2];
            const float d  = disp32.at<float>(r, c);

            const size_t idx =
                (static_cast<size_t>(r) * static_cast<size_t>(cloud.width) +
                 static_cast<size_t>(c)) *
                3u;

            // Mark invalid when: disparity ≤ 0 (raw or corrected), OpenCV
            // sentinel |z|≥10000 or non-finite reprojection, or depth outside
            // the user-specified range.
            Reason why = Reason::valid;
            if (d <= 0.0f || !std::isfinite(z) || std::abs(z) >= 10000.0f ||
                !std::isfinite(pt[0]) || !std::isfinite(pt[1])) {
                why = Reason::matcher_rejected;
            } else if (z < opts_.min_depth_m || z > opts_.max_depth_m) {
                why = Reason::depth_range;
            }

            if (why != Reason::valid) {
                cloud.data[idx]     = kNaN;
                cloud.data[idx + 1] = kNaN;
                cloud.data[idx + 2] = kNaN;
            } else {
                cloud.data[idx]     = pt[0];
                cloud.data[idx + 1] = pt[1];
                cloud.data[idx + 2] = z;
            }
            if (reason_row) {
                reason_row[c] = static_cast<std::uint8_t>(why);
            }
        }
    }

    if (opts_.outlier_filter.enabled) {
        PointCloud filtered = cloud;
        apply_outlier_filter(cloud, filtered, opts_.outlier_filter, reasons);
        cloud = std::move(filtered);
    }

    if (reason != nullptr) {
        *reason = reasons;
    }
    return cloud;
}

} // namespace score

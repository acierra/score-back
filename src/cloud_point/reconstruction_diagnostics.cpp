#include "cloud_point/reconstruction_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace score {

namespace {

constexpr double kBinWidthM        = 0.01;  ///< 10 mm depth bins
constexpr std::size_t kMinBinCount = 200;   ///< bins below this are omitted
constexpr double kFitMaxError3dM   = 0.02;  ///< D3: points beyond are ignored

using Reason = PointCloudBuilder::Reason;

bool finite(const cv::Vec3f &p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

/// @brief Median of @p v; reorders @p v.  NaN when empty.
double median_inplace(std::vector<double> &v) {
    if (v.empty()) {
        return ReconstructionDiagnostics::kNaN;
    }
    const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
    std::nth_element(v.begin(), mid, v.end());
    if (v.size() % 2 == 1) {
        return *mid;
    }
    const double lower = *std::max_element(v.begin(), mid);
    return 0.5 * (lower + *mid);
}

/// @brief One matched pixel (finite GT and finite prediction).
struct Matched {
    double z_gt;
    double z_rec;
    double err_3d;
    int bin; ///< floor(z_gt / 10 mm); INT_MIN when outside the depth range
};

constexpr int kNoBin = std::numeric_limits<int>::min();

} // namespace

// ---------------------------------------------------------------------------
// ReprojectionGeometry
// ---------------------------------------------------------------------------

ReprojectionGeometry ReprojectionGeometry::from_q(const cv::Mat &q) {
    if (q.rows != 4 || q.cols != 4 || q.type() != CV_64F) {
        throw std::invalid_argument(
            "ReprojectionGeometry: Q must be 4x4 CV_64F");
    }
    ReprojectionGeometry g;
    g.f   = q.at<double>(2, 3);
    g.q32 = q.at<double>(3, 2);
    g.q33 = q.at<double>(3, 3);
    if (!(std::abs(g.q32) > 0.0) || !std::isfinite(g.q32) ||
        !std::isfinite(g.f) || !(g.f > 0.0)) {
        throw std::invalid_argument(
            "ReprojectionGeometry: Q(3,2) must be non-zero and Q(2,3) > 0");
    }
    g.fB = g.f / std::abs(g.q32);
    return g;
}

// ---------------------------------------------------------------------------
// diagnose_reconstruction
// ---------------------------------------------------------------------------

ReconstructionDiagnostics
diagnose_reconstruction(const PointCloud &predicted, const cv::Mat &ground_truth,
                        const ReconstructionDiagnosticsInputs &inputs) {
    if (ground_truth.type() != CV_32FC3) {
        throw std::invalid_argument(
            "diagnose_reconstruction: ground truth must be CV_32FC3");
    }
    if (predicted.width != ground_truth.cols ||
        predicted.height != ground_truth.rows) {
        throw std::invalid_argument(
            "diagnose_reconstruction: prediction and ground-truth dimensions "
            "do not match");
    }
    const auto pixel_count = static_cast<std::size_t>(predicted.width) *
                             static_cast<std::size_t>(predicted.height);
    if (predicted.data.size() != pixel_count * 3u) {
        throw std::invalid_argument(
            "diagnose_reconstruction: prediction storage size is invalid");
    }
    if (inputs.reason.type() != CV_8U ||
        inputs.reason.cols != predicted.width ||
        inputs.reason.rows != predicted.height) {
        throw std::invalid_argument(
            "diagnose_reconstruction: reason map must be CV_8U with the "
            "prediction's dimensions");
    }
    if (inputs.num_disparities <= 0) {
        throw std::invalid_argument(
            "diagnose_reconstruction: num_disparities must be positive");
    }
    if (!std::isfinite(inputs.disparity_offset_px)) {
        throw std::invalid_argument(
            "diagnose_reconstruction: disparity_offset_px must be finite");
    }
    const ReprojectionGeometry geom = ReprojectionGeometry::from_q(inputs.q);

    const double search_lo = static_cast<double>(inputs.min_disparity);
    const double search_hi =
        static_cast<double>(inputs.min_disparity + inputs.num_disparities);
    // cv::StereoSGBM never matches the leftmost minDisparity + numDisparities
    // columns: the search range there reaches beyond the right image's left
    // edge, so the whole band is emitted as invalid whatever the scene
    // (stereosgbm.cpp: minX1 = max(minD + numD, 0)).  Any valid ground-truth
    // pixel in that band that the search range could otherwise reach is an
    // `image_border` loss, not a matcher rejection.
    const int border_cols =
        std::max(inputs.min_disparity + inputs.num_disparities, 0);

    ReconstructionDiagnostics out;
    std::vector<Matched> matched;
    matched.reserve(pixel_count / 2);

    std::size_t lost_search = 0, lost_border = 0, lost_matcher = 0,
                lost_depth = 0, lost_outlier = 0;
    std::size_t valid_predicted = 0, unscored = 0;

    for (int row = 0; row < ground_truth.rows; ++row) {
        const std::uint8_t *reason_row = inputs.reason.ptr<std::uint8_t>(row);
        for (int col = 0; col < ground_truth.cols; ++col) {
            const auto idx =
                (static_cast<std::size_t>(row) * predicted.width + col) * 3u;
            const cv::Vec3f estimate(predicted.data[idx],
                                     predicted.data[idx + 1],
                                     predicted.data[idx + 2]);
            const bool has_estimate = finite(estimate);
            const cv::Vec3f truth   = ground_truth.at<cv::Vec3f>(row, col);
            const bool has_truth    = finite(truth);

            if (has_estimate) {
                ++valid_predicted;
                if (!has_truth) {
                    ++unscored;
                }
            }
            if (!has_truth) {
                continue;
            }
            ++out.ground_truth_points;

            if (has_estimate) {
                const cv::Vec3f delta = estimate - truth;
                const double err =
                    std::sqrt(static_cast<double>(delta[0]) * delta[0] +
                              static_cast<double>(delta[1]) * delta[1] +
                              static_cast<double>(delta[2]) * delta[2]);
                Matched m;
                m.z_gt   = truth[2];
                m.z_rec  = estimate[2];
                m.err_3d = err;
                m.bin    = kNoBin;
                if (m.z_gt >= inputs.min_depth_m && m.z_gt <= inputs.max_depth_m) {
                    m.bin = static_cast<int>(std::floor(m.z_gt / kBinWidthM));
                }
                matched.push_back(m);
                continue;
            }

            // Unmatched: attribute to exactly one cause, in the D4 order.
            const double d_gt  = geom.disparity(truth[2]);
            const double d_raw = d_gt - inputs.disparity_offset_px;
            if (!(d_raw >= search_lo && d_raw < search_hi)) {
                ++lost_search;
            } else if (col < border_cols) {
                ++lost_border;
            } else {
                switch (static_cast<Reason>(reason_row[col])) {
                case Reason::matcher_rejected:
                    ++lost_matcher;
                    break;
                case Reason::depth_range:
                    ++lost_depth;
                    break;
                case Reason::outlier_filter:
                    ++lost_outlier;
                    break;
                case Reason::valid:
                default:
                    throw std::invalid_argument(
                        "diagnose_reconstruction: reason map marks pixel (" +
                        std::to_string(row) + ", " + std::to_string(col) +
                        ") as " +
                        PointCloudBuilder::reason_name(
                            static_cast<Reason>(reason_row[col])) +
                        " but the cloud has no point there");
                }
            }
        }
    }

    if (out.ground_truth_points == 0) {
        throw std::invalid_argument(
            "diagnose_reconstruction: ground truth has no valid points");
    }

    const double gt_count = static_cast<double>(out.ground_truth_points);
    out.matched_points    = matched.size();
    out.coverage          = static_cast<double>(matched.size()) / gt_count;
    out.coverage_loss.out_of_search_range =
        static_cast<double>(lost_search) / gt_count;
    out.coverage_loss.image_border = static_cast<double>(lost_border) / gt_count;
    out.coverage_loss.matcher_rejected =
        static_cast<double>(lost_matcher) / gt_count;
    out.coverage_loss.depth_range = static_cast<double>(lost_depth) / gt_count;
    out.coverage_loss.outlier_filter =
        static_cast<double>(lost_outlier) / gt_count;
    out.unscored_fraction =
        valid_predicted == 0
            ? 0.0
            : static_cast<double>(unscored) / static_cast<double>(valid_predicted);

    if (matched.empty()) {
        return out;
    }

    // --- Tail and overall bias -------------------------------------------
    {
        std::size_t over5 = 0, over20 = 0, over50 = 0;
        double max_err = 0.0;
        std::vector<double> signed_z;
        signed_z.reserve(matched.size());
        for (const auto &m : matched) {
            over5 += m.err_3d > 0.005;
            over20 += m.err_3d > 0.020;
            over50 += m.err_3d > 0.050;
            max_err = std::max(max_err, m.err_3d);
            signed_z.push_back(m.z_rec - m.z_gt);
        }
        const double n     = static_cast<double>(matched.size());
        out.tail_over_5mm  = static_cast<double>(over5) / n;
        out.tail_over_20mm = static_cast<double>(over20) / n;
        out.tail_over_50mm = static_cast<double>(over50) / n;
        out.max_3d_m       = max_err;
        out.bias_z_median_m = median_inplace(signed_z);
    }

    // --- Depth bins (10 mm, keyed by GT z, ≥ 200 points) ------------------
    std::map<int, std::vector<std::size_t>> bins;
    for (std::size_t i = 0; i < matched.size(); ++i) {
        if (matched[i].bin != kNoBin) {
            bins[matched[i].bin].push_back(i);
        }
    }
    for (auto it = bins.begin(); it != bins.end();) {
        if (it->second.size() < kMinBinCount) {
            it = bins.erase(it);
        } else {
            ++it;
        }
    }
    std::vector<double> scratch;
    for (const auto &[key, indices] : bins) {
        ReconstructionDiagnostics::DepthBin bin;
        bin.lo_m  = static_cast<double>(key) * kBinWidthM;
        bin.hi_m  = static_cast<double>(key + 1) * kBinWidthM;
        bin.count = indices.size();
        scratch.clear();
        for (const auto i : indices) {
            scratch.push_back(matched[i].z_rec - matched[i].z_gt);
        }
        bin.bias_z_median_m = median_inplace(scratch);
        scratch.clear();
        for (const auto i : indices) {
            scratch.push_back(matched[i].err_3d);
        }
        bin.abs_3d_median_m = median_inplace(scratch);
        out.depth_bias.push_back(bin);
    }

    // --- Calibration-error fits (D3) --------------------------------------
    {
        std::vector<double> offsets, scales;
        offsets.reserve(matched.size());
        scales.reserve(matched.size());
        for (const auto &m : matched) {
            if (m.err_3d >= kFitMaxError3dM || !(m.z_gt > 0.0) ||
                !(m.z_rec > 0.0)) {
                continue;
            }
            offsets.push_back(geom.disparity(m.z_gt) - geom.disparity(m.z_rec));
            scales.push_back(m.z_rec / m.z_gt);
        }
        out.fit_disparity_offset_px = median_inplace(offsets);
        out.fit_depth_scale         = median_inplace(scales);

        // Residual: RMS over the reported bins of the bin median signed error
        // after applying the model.
        const auto residual = [&](auto &&corrected_z) {
            if (bins.empty()) {
                return ReconstructionDiagnostics::kNaN;
            }
            double sum_sq = 0.0;
            for (const auto &[key, indices] : bins) {
                scratch.clear();
                for (const auto i : indices) {
                    scratch.push_back(corrected_z(matched[i]) - matched[i].z_gt);
                }
                const double med = median_inplace(scratch);
                sum_sq += med * med;
            }
            return std::sqrt(sum_sq / static_cast<double>(bins.size()));
        };
        if (std::isfinite(out.fit_disparity_offset_px)) {
            const double delta = out.fit_disparity_offset_px;
            out.fit_disparity_offset_residual_m =
                residual([&](const Matched &m) {
                    return geom.depth(geom.disparity(m.z_rec) + delta);
                });
        }
        if (std::isfinite(out.fit_depth_scale) && out.fit_depth_scale != 0.0) {
            const double s = out.fit_depth_scale;
            out.fit_depth_scale_residual_m =
                residual([&](const Matched &m) { return m.z_rec / s; });
        }
    }

    return out;
}

} // namespace score

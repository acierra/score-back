#include "cloud_point/stereo_matcher_factory.hpp"
#include "cloud_point/cpu_stereo_matcher.hpp"
#include "cloud_point/gpu_stereo_matcher.hpp"
#include <glog/logging.h>
#include <stdexcept>
#include <string>

namespace score {

std::unique_ptr<IStereoMatcher>
StereoMatcherFactory::create(StereoAlgorithmType type, int num_disparities,
                             CpuStereoMatcher::Params cpu_params) {
    if (num_disparities <= 0 || num_disparities % 16 != 0) {
        throw std::invalid_argument(
            "StereoMatcherFactory: num_disparities must be a positive "
            "multiple of 16, got " +
            std::to_string(num_disparities));
    }
    switch (type) {
    case StereoAlgorithmType::CPU:
        return std::make_unique<CpuStereoMatcher>(0, num_disparities,
                                                  cpu_params);
    case StereoAlgorithmType::GPU:
        try {
            return std::make_unique<GpuStereoMatcher>(0, num_disparities);
        } catch (const std::exception &e) {
            LOG(WARNING) << "GPU stereo matcher unavailable: " << e.what()
                         << ". Falling back to CPU.";
            return std::make_unique<CpuStereoMatcher>(0, num_disparities,
                                                      cpu_params);
        }
    }
    return nullptr;
}

} // namespace score

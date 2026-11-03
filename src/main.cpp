#include "cloud_point_rpc/cli.hpp"
#include "cloud_point_rpc/config.hpp"
#include <fstream>
#include <glog/logging.h>
#include <iostream>

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr       = 1;

    std::string config_path = "config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    // Check if config file exists
    std::ifstream f(config_path.c_str());
    if (!f.good()) {
        std::cerr << "Config file not found: " << config_path << std::endl;
        std::cerr << "Please create config.yaml or provide path as argument."
                  << std::endl;
        return 1;
    }
    f.close();

    try {
        auto config = score::ConfigLoader::load(config_path);
        score::CliStereoOptions stereo;
        stereo.use_gpu         = config.cloud_point.algorithm == "gpu";
        stereo.num_disparities = config.cloud_point.num_disparities;
        stereo.min_depth_m = static_cast<float>(config.cloud_point.min_depth_m);
        stereo.max_depth_m = static_cast<float>(config.cloud_point.max_depth_m);
        stereo.ply_stride  = config.cloud_point.ply_stride;
        stereo.wls_filter  = config.cloud_point.wls_filter;
        stereo.disparity_offset_px    = config.cloud_point.disparity_offset_px;
        const auto &of                = config.cloud_point.outlier_filter;
        stereo.outlier_filter.enabled = of.enabled;
        stereo.outlier_filter.window  = of.window;
        stereo.outlier_filter.abs_threshold_m = of.abs_threshold_m;
        stereo.outlier_filter.rel_threshold   = of.rel_threshold;
        stereo.outlier_filter.min_neighbours  = of.min_neighbours;
        stereo.viewer.output_dir              = config.viewer.output_dir;
        stereo.viewer.template_dir            = config.viewer.template_dir;
        stereo.viewer.open_command            = config.viewer.open_command;
        stereo.viewer.stride                  = config.viewer.stride;
        return score::run_cli(std::cin, std::cout, config.server.ip,
                              config.server.port, stereo);
    } catch (const std::exception &e) {
        std::cerr << "Failed to start CLI: " << e.what() << std::endl;
        return 1;
    }
}

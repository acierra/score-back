#include "cloud_point_rpc/config.hpp"
#include "cloud_point_rpc/rpc_dto.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/service.hpp"
#include "cloud_point_rpc/tcp_server.hpp"
#include <glog/logging.h>
#include <string>

using json = nlohmann::json;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr   = 1;

    std::string config_path = "config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    LOG(INFO) << "Starting Cloud Point RPC Server (Test Mock)...";

    try {
        auto config = score::ConfigLoader::load(config_path);
        LOG(INFO) << "Loaded config from " << config_path;

        // Inject test data into service
        score::Service service(config.test_data);
        score::RpcServer rpc_server;

        rpc_server.register_method("get-intrinsic-params", [&](const json &) {
            return service.get_intrinsic_params();
        });

        rpc_server.register_method("get-extrinsic-params", [&](const json &) {
            return service.get_extrinsic_params();
        });

        rpc_server.register_method("get-cloud-point", [&](const json &) {
            return service.get_cloud_point();
        });

        rpc_server.register_method("get-stereo-calibration", [&](const json &) {
            nlohmann::json j;
            score::to_json(j, service.get_stereo_calibration());
            return j;
        });

        rpc_server.register_method("get-image-pair", [&](const json &) {
            nlohmann::json j;
            score::to_json(j, service.get_image_pair());
            return j;
        });

        score::TcpServer server(config.server.ip, config.server.port,
                                [&](const std::string &request) {
                                    return rpc_server.process(request);
                                });

        server.start();
        server.join();

    } catch (const std::exception &e) {
        LOG(ERROR) << "Fatal error: " << e.what();
        return 1;
    }

    return 0;
}

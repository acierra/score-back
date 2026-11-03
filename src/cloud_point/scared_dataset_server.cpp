/// @file scared_dataset_server.cpp
/// @brief RPC server backed by a SCARED dataset keyframe directory.
///
/// Usage: scared_dataset_server <keyframe_dir> [port]
///
/// Serves get-stereo-calibration and get-image-pair matching the wire
/// protocol consumed by CloudPointClient.  The same images are returned on
/// every get-image-pair call (single-keyframe source); the frame counter
/// increments so the client can detect stale frames if desired.
#include "cloud_point/scared_dataset_loader.hpp"
#include "cloud_point_rpc/rpc_dto.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/tcp_server.hpp"
#include <atomic>
#include <glog/logging.h>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = 1;

    if (argc < 2) {
        LOG(ERROR) << "Usage: " << argv[0] << " <keyframe_dir> [port]";
        return 1;
    }

    const std::string keyframe_dir = argv[1];

    LOG(INFO) << "SCARED dataset server starting";
    LOG(INFO) << "  keyframe_dir = " << keyframe_dir;

    try {
        const int port = (argc >= 3) ? std::stoi(argv[2]) : 8080;
        LOG(INFO) << "  port         = " << port;

        score::ScaredDatasetLoader loader(keyframe_dir);

        std::atomic<uint64_t> frame_counter{0};
        score::RpcServer rpc_server;

        rpc_server.register_method(
            "get-stereo-calibration", [&](const json &) -> json {
                json j;
                score::to_json(j, loader.calibration());
                return j;
            });

        rpc_server.register_method(
            "get-image-pair", [&](const json &) -> json {
                json j;
                score::to_json(j, loader.image_pair(frame_counter++));
                return j;
            });

        score::TcpServer server(
            "0.0.0.0", port,
            [&](const std::string &request) {
                return rpc_server.process(request);
            });

        server.start();
        LOG(INFO) << "SCARED dataset server ready on port " << port;
        server.join();

    } catch (const std::exception &e) {
        LOG(ERROR) << "Fatal error: " << e.what();
        return 1;
    }

    return 0;
}

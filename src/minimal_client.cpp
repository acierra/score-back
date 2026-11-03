#include "cloud_point_rpc/config.hpp"
#include "cloud_point_rpc/tcp_connector.hpp"
#include <glog/logging.h>
#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr       = 1;

    std::string config_path = "config.yml";
    if (argc > 1) {
        config_path = argv[1];
    }

    try {
        auto config = score::ConfigLoader::load(config_path);

        score::TCPConnector connector(config.server.ip,
                                      static_cast<size_t>(config.server.port));

        const std::string request =
            R"({"jsonrpc":"2.0","method":"ping","params":{},"id":1})";

        std::string response = connector.Send(request);
        std::cout << response << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

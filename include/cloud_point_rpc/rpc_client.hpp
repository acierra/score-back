#pragma once

#include "cloud_point_rpc/rpc_dto.hpp"
#include "cloud_point_rpc/tcp_connector.hpp"
#include <asio.hpp>
#include <glog/logging.h>
#include <jsonrpccxx/client.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace score {

class RpcClient : public jsonrpccxx::JsonRpcClient {
  public:
    explicit RpcClient(TCPConnector &connector)
        : jsonrpccxx::JsonRpcClient(connector, jsonrpccxx::version::v2) {}

    [[nodiscard]] std::vector<double> get_intrinsic_params() {
        return call<std::vector<double>>("get-intrinsic-params");
    }

    [[nodiscard]] std::vector<double> get_extrinsic_params() {
        return call<std::vector<double>>("get-extrinsic-params");
    }

    [[nodiscard]] std::vector<std::vector<double>> get_cloud_point() {
        return call<std::vector<std::vector<double>>>("get-cloud-point");
    }

    [[nodiscard]] StereoCalibrationRPC get_stereo_calibration() {
        return call<nlohmann::json>("get-stereo-calibration")
            .get<StereoCalibrationRPC>();
    }

    [[nodiscard]] ImagePairRPC get_image_pair() {
        return call<nlohmann::json>("get-image-pair").get<ImagePairRPC>();
    }

    /// @brief Call a method with no params.
    template <typename ReturnType>
    [[nodiscard]] ReturnType call(std::string_view name) {
        return this->CallMethod<ReturnType>(id_++, name.data());
    }

    /// @brief Call a method with named params (JSON object keys → values).
    template <typename ReturnType>
    [[nodiscard]] ReturnType call(std::string_view name,
                                  const nlohmann::json &params) {
        jsonrpccxx::named_parameter named;
        for (auto &[k, v] : params.items()) {
            named[k] = v;
        }
        return this->CallMethodNamed<ReturnType>(id_++, name.data(), named);
    }

    ~RpcClient() = default;

  private:
    int id_{0};
};

} // namespace score

#pragma once

#include "export.h"
#include <functional>
#include <jsonrpccxx/server.hpp>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <variant>
extern "C" {

struct rpc_string {
    rpc_string(const char *data, uint64_t size) : s(data, size) {}
    rpc_string() = default;

    std::string s;
};
}

namespace score {

class CRPC_EXPORT RpcServer {
  public:
    using Handler    = std::function<std::variant<nlohmann::json, std::string>(
        const nlohmann::json &)>;
    using callback_t = rpc_string *(*)(rpc_string *);

  public:
    /// @note +1 method implicitly added: get-available-methods
    RpcServer();

    void register_method(const std::string &name, Handler handler);
    void register_method(const std::string &name, callback_t handler);

    uint64_t get_count() noexcept;
    std::span<std::string_view> get_method_names() noexcept;
    std::string_view get_method_name_by_id(uint64_t id) noexcept;

    ///@param request_str json rpc 2.0 formatted string
    [[nodiscard]] std::string process(const std::string &request_str);

  private:
    std::vector<std::string_view> handler_names_;
    std::map<std::string, Handler> handlers_;
};

} // namespace score

#include "cloud_point_rpc/rpc_server.hpp"
#include "server_api.h"
#include <glog/logging.h>
#include <variant>
using json = nlohmann::json;

namespace score {

namespace {
json create_error(int code, const std::string &message,
                  const json &id = nullptr) {
    return {{"jsonrpc", "2.0"},
            {"error", {{"code", code}, {"message", message}}},
            {"id", id}};
}

struct CreateSuccess {
    json obj;
    json id;

    void operator()(const json &result) {
        obj = {{"jsonrpc", "2.0"}, {"result", result}, {"id", id}};
    }
    void operator()(const std::string &result) {
        obj = {{"jsonrpc", "2.0"}, {"result", result}, {"id", id}};
    }
};

} // namespace

template <typename T> struct Deleter {
    void operator()(T *element) {
        (void)element;
        LOG(ERROR) << "Called default deleter";
    }
};

template <> struct Deleter<rpc_string> {
    void operator()(rpc_string *element) {
        if (element) {
            crpc_str_destroy(element);
        }
    }
};
using rpcStringPtr = std::unique_ptr<rpc_string, Deleter<rpc_string>>;

RpcServer::RpcServer() {
    register_method("get-available-methods",
                    [&](const json &) { return get_method_names(); });
}

void RpcServer::register_method(const std::string &name, Handler handler) {
    handlers_[name] = std::move(handler);
    handler_names_.push_back(handlers_.find(name)->first);
}

void RpcServer::register_method(const std::string &name, callback_t handler) {
    handlers_[name] = [handler](const nlohmann::json &j)
        -> std::variant<nlohmann::json, std::string> {
        rpc_string tmp;
        tmp.s    = j.dump();
        auto res = rpcStringPtr(handler(&tmp));
        if (!res) {
            LOG(ERROR) << "Method is invalid";
            return {};
        }
        std::variant<nlohmann::json, std::string> ret;
        try {
            ret = json::parse(res->s);
        } catch (std::exception &e) {
            DLOG(INFO) << "return value is not a json";
            ret = res->s;
        }
        return ret;
    };
    handler_names_.push_back(handlers_.find(name)->first);
}

std::span<std::string_view> RpcServer::get_method_names() noexcept {
    return this->handler_names_;
}

uint64_t RpcServer::get_count() noexcept { return this->handler_names_.size(); }

std::string_view RpcServer::get_method_name_by_id(uint64_t id) noexcept {
    if (id >= handler_names_.size()) {
        LOG(ERROR) << __func__
                   << std::format(
                          ": called with id = {} which is bigger, than size={}",
                          id, handler_names_.size());
        return {};
    }
    return handler_names_.at(id);
}

std::string RpcServer::process(const std::string &request_str) {
    LOG(INFO) << request_str;
    json request;
    try {
        request = json::parse(request_str);
    } catch (const json::parse_error &) {
        LOG(ERROR) << "json parse error; " << __func__;
        return create_error(-32700, "Parse error").dump();
    }

    // Batch requests are not supported in this minimal version, assume single
    // object
    if (!request.contains("jsonrpc") || request["jsonrpc"] != "2.0" ||
        !request.contains("method") || !request.contains("id")) {
        return create_error(-32600, "Invalid Request",
                            request.value("id", json(nullptr)))
            .dump();
    }

    std::string method = request["method"];
    json id            = request["id"];
    json params        = request.value("params", json::object());

    auto it            = handlers_.find(method);
    if (it == handlers_.end()) {
        return create_error(-32601, "Method not found", id).dump();
    }

    try {
        auto result = it->second(params);
        CreateSuccess visitor;
        visitor.id = id;
        std::visit(visitor, result);
        return visitor.obj.dump();
    } catch (const std::exception &e) {
        return create_error(-32000, e.what(), id).dump(); // Server error
    }
}

} // namespace score

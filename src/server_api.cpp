#include "server_api.h"
#include "cloud_point_rpc/config.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/tcp_server.hpp"
#include <algorithm>
#include <glog/logging.h>
#include <list>
#include <memory>
#include <string>

static std::list<std::unique_ptr<rpc_string>> gc;
std::mutex gc_mtx;
std::mutex server_mtx;
score::RpcServer rpc_server;
std::unique_ptr<score::TcpServer> server = nullptr;

extern "C" {

const char *crpc_str_get_data(const rpc_string *that) {
    if (!that) {
        LOG(ERROR) << "Tried to get data on nullptr";
        return nullptr;
    }
    return that->s.c_str();
}

uint64_t crpc_str_get_size(const rpc_string *that) {
    if (!that) {
        LOG(ERROR) << "Tried to get size on nullptr";
        return 0;
    }
    return that->s.size();
}

rpc_string *crpc_str_create(const char *data, uint64_t size) {
    if (!data) {
        LOG(ERROR) << "Tried to create with nullptr data";
        return nullptr;
    }
    std::lock_guard lock(gc_mtx);
    gc.push_back(std::make_unique<rpc_string>(data, size));
    return gc.back().get();
}

void crpc_str_destroy(rpc_string *that) {
    if (!that) {
        LOG(ERROR) << "Tried to destroy on nullptr";
        return;
    }
    std::lock_guard lock(gc_mtx);
    auto it = std::ranges::find(gc, that, &std::unique_ptr<rpc_string>::get);
    if (it != gc.end())
        gc.erase(it);
}

void crpc_init(const char *config_path) {
    if (!google::IsGoogleLoggingInitialized())
        google::InitGoogleLogging("CloudPointRPC");
    if (config_path == nullptr) {
        LOG(ERROR) << "config_path was not provided";
        return;
    }
    try {
        auto config = score::ConfigLoader::load(config_path);
        LOG(INFO) << "Loaded config from " << config_path;

        server = std::make_unique<score::TcpServer>(
            config.server.ip, config.server.port,
            [&](const std::string &request) {
                std::lock_guard lock(server_mtx);
                return rpc_server.process(request);
            });
        server->start();
    } catch (const std::exception &e) {
        LOG(ERROR) << "Fatal error: " << e.what();
    }
}

void crpc_init_with_address(const char *ip, int port) {
    if (!google::IsGoogleLoggingInitialized())
        google::InitGoogleLogging("CloudPointRPC");
    if (!ip) {
        LOG(ERROR) << "ip was not provided";
        return;
    }
    try {
        server = std::make_unique<score::TcpServer>(
            std::string(ip), static_cast<size_t>(port),
            [&](const std::string &request) {
                std::lock_guard lock(server_mtx);
                return rpc_server.process(request);
            });
        server->start();
        LOG(INFO) << "Server started on " << ip << ":" << port;
    } catch (const std::exception &e) {
        LOG(ERROR) << "Fatal error: " << e.what();
    }
}

void crpc_deinit() {
    if (server) {
        // Must stop() before join(): stop() clears running_ so the accept
        // thread can exit; join() alone would block forever while a client
        // thread is blocked in a read (deadlock observed in Unity e2e).
        server->stop();
        server->join();
    }
    server.reset();
    std::lock_guard lock(gc_mtx);
    gc.clear();
}

void crpc_add_method(callback_t cb, rpc_string *name) {
    if (!name || !cb) {
        LOG(ERROR) << "Invalid arguments (nullptr)";
        return;
    }
    std::lock_guard lock(server_mtx);
    rpc_server.register_method(name->s, cb);
}

rpc_string *crpc_get_method_name_by_id(uint64_t id) {
    auto value = rpc_server.get_method_name_by_id(id);

    return crpc_str_create(value.data(), value.size());
}

uint64_t crpc_get_methods_count() { return rpc_server.get_count(); }
}

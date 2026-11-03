#pragma once
#include "cloud_point_rpc/serialize.hpp"
#include "export.h"
#include "jsonrpccxx/iclientconnector.hpp"
#include <asio.hpp>
#include <cloud_point_rpc/tcp_read.hpp>
#include <glog/logging.h>
#include <string>
namespace score {
/**
 * TCPConnector main purpose is to implement jsonrpccxx::IClientConnector Send
 * method As an internal implementation, TCPConnector adds to the beginning of
 * the json "expected size" of a packet in case when the packet was for some
 * reason on any level fractured.
 */
class CRPC_EXPORT TCPConnector : public jsonrpccxx::IClientConnector {
  public:
    TCPConnector(const std::string &ip, size_t port) noexcept(false)
        : io_context_(), socket_(io_context_) {
        try {
            LOG(INFO) << "Client connecting to " << ip << ":" << port;
            asio::ip::tcp::endpoint endpoint(asio::ip::make_address(ip), port);
            socket_.connect(endpoint);
            LOG(INFO) << "Client connected";
        } catch (const std::exception &e) {
            throw std::runtime_error(std::string("Connection Failed: ") +
                                     e.what());
        }
    }

    std::string Send(const std::string &request) override {
        // Send
        LOG(INFO) << "Client sending: " << request
                  << " Size: " << request.size();
        auto remove_const = request;
        inplace_size_embedding(remove_const);
        asio::write(socket_, asio::buffer(remove_const));
        return tcp_read(socket_, "TCPConnector] ");
    }

  private:
    asio::io_context io_context_;
    asio::ip::tcp::socket socket_;
};

} // namespace score
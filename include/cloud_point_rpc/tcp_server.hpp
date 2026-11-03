#pragma once

#include "export.h"
#include <asio.hpp>
#include <atomic>
#include <cloud_point_rpc/tcp_read.hpp>
#include <functional>
#include <glog/logging.h>
#include <list>
#include <ranges>
#include <string>
#include <thread>
namespace score {

class CRPC_EXPORT TcpServer {
  public:
    using RequestProcessor = std::function<std::string(const std::string &)>;

    TcpServer(const std::string &ip, int port, RequestProcessor processor)
        : ip_(ip), port_(port), processor_(std::move(processor)),
          acceptor_(io_context_), running_(false) {}

    ~TcpServer() {
        stop();
        std::lock_guard lock(cliThrMtx_);
        for (auto &thread : client_threads_ | std::views::keys) {
            thread.join();
        }
    }

    void start() {
        using namespace std::chrono_literals;
        try {
            asio::ip::tcp::endpoint endpoint(asio::ip::make_address(ip_),
                                             port_);
            acceptor_.open(endpoint.protocol());
            acceptor_.set_option(asio::ip::tcp::acceptor::reuse_address(true));
            acceptor_.bind(endpoint);
            acceptor_.listen();

            running_ = true;
            LOG(INFO) << "Server listening on " << ip_ << ":" << port_;

            accept_thread_ = std::jthread([this]() {
                LOG(INFO) << "Accept thread started";
                while (running_) {
                    {
                        std::lock_guard lock(cliThrMtx_);
                        client_threads_.remove_if([](auto &client_info) {
                            bool result = false;
                            if (client_info.second.wait_for(0ms) ==
                                std::future_status::ready) {
                                client_info.first.join();
                                result = true;
                            }
                            return result;
                        });
                    }
                    try {
                        auto socket = std::make_shared<asio::ip::tcp::socket>(
                            io_context_);
                        {
                            std::lock_guard lock(acceptorMtx_);
                            acceptor_.accept(*socket);
                        }
                        LOG(INFO)
                            << "New connection from "
                            << socket->remote_endpoint().address().to_string();
                        auto done = std::make_shared<std::promise<bool>>();
                        {
                            std::lock_guard lock(cliThrMtx_);
                            client_threads_.push_back(std::make_pair(
                                std::jthread([this, socket, done]() {
                                    handle_client(socket);
                                    done->set_value(true);
                                }),
                                done->get_future()));
                        }
                    } catch (const std::system_error &e) {
                        LOG(INFO) << "Accept exception: " << e.what();
                        if (running_) {
                            LOG(WARNING) << "Accept failed: " << e.what();
                        }
                    }
                }
                LOG(INFO) << "Accept thread exiting";
            });
        } catch (const std::exception &e) {
            LOG(ERROR) << "Server start failed: " << e.what();
            throw;
        }
    }

    void stop() {
        if (!running_)
            return;
        LOG(INFO) << "Stopping server...";
        running_ = false;
        // Ensure accept unblocks by connecting a dummy socket FIRST,
        // while the acceptor is still open. This avoids a race where close()
        // removes the listen endpoint before the connect completes.
        try {
            asio::ip::tcp::endpoint endpoint(asio::ip::make_address(ip_),
                                             port_);
            asio::ip::tcp::socket dummy_sock(io_context_);
            asio::error_code connect_ec;
            std::ignore = dummy_sock.connect(endpoint, connect_ec);
        } catch (...) {
            // Ignore
        }
        // Now close the acceptor to unblock any pending accept()
        asio::error_code ec;
        {
            std::lock_guard lock(acceptorMtx_);
            std::ignore = acceptor_.close(ec);
        }
        if (ec.value()) {
            LOG(ERROR) << std::format(
                "acceptor closed with a value returned = {}", ec.value());
        }
        LOG(INFO) << "Acceptor closed";
    }

    void join() {
        if (accept_thread_.joinable()) {
            LOG(INFO) << "Joining accept thread...";
            accept_thread_.join();
            LOG(INFO) << "Accept thread joined";
        }
    }

  private:
    void handle_client(std::shared_ptr<asio::ip::tcp::socket> socket) {
        LOG(INFO) << "Server reading from client...";
        while (true) {
            try {
                auto payload = tcp_read(*socket, "TCPServer] ");
                if (payload.empty())
                    break; // connection closed or read error
                std::string response = processor_(payload);
                response += "\n";
                DLOG(INFO) << "Server sending response: " << response;
                inplace_size_embedding(response);
                asio::write(*socket, asio::buffer(response));
                LOG(INFO) << "Server sent response";
            } catch (const std::exception &e) {
                LOG(WARNING) << "Client handling error: " << e.what();
                break;
            }
        }
    }

    std::string ip_;
    int port_;
    RequestProcessor processor_;

    asio::io_context io_context_;
    asio::ip::tcp::acceptor acceptor_;

    std::atomic<bool> running_;
    std::list<std::pair<std::jthread, std::future<bool>>> client_threads_;
    std::mutex cliThrMtx_;
    std::mutex acceptorMtx_;
    std::jthread accept_thread_;
};

} // namespace score

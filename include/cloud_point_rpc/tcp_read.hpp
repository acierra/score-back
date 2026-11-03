#pragma once
#include <array>
#include <asio.hpp>
#include <cloud_point_rpc/serialize.hpp>
#include <glog/logging.h>
namespace score {

static inline std::string tcp_read(asio::ip::tcp::socket &socket,
                                   std::string_view prefix) {
    std::string result;
    std::array<char, 8> header;
    LOG(INFO) << prefix.data() << "trying to read";
    try {
        size_t len = asio::read(socket, asio::buffer(header, header.size()));
        if (len != sizeof(uint64_t)) {
            LOG(ERROR) << prefix.data() << "failed to read header";
            return result;
        }
        std::vector<uint8_t> v(header.begin(), header.begin() + 8);
        uint64_t packet_size = deserialize<uint64_t>(v);
        LOG(INFO) << prefix.data()
                  << "Received packet of size: " << packet_size;

        std::vector<char> payload(packet_size);
        len = asio::read(socket, asio::buffer(payload));

        // if failed to fetch in one pack, try to read until size
        while (len < packet_size) {
            len += asio::read(socket, asio::buffer(payload.data() + len,
                                                   payload.size() - len));
        }

        LOG(INFO) << prefix.data()
                  << std::format("Was able to read len={}", len);
        result = std::string(payload.begin(), payload.end());
        DLOG(INFO) << prefix.data() << "Payload: \n" << result;
    } catch (const std::exception &e) {
        LOG(WARNING) << prefix.data() << "handling error: " << e.what();
    }
    return result;
}

} // namespace score
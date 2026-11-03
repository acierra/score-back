//
// Created by vptyp on 11.03.2026.
//
#include "cloud_point_rpc/rpc_coder.hpp"

#include "libbase64.h"
#include <glog/logging.h>
namespace score {

Base64RPCCoder::Base64RPCCoder()  = default;
Base64RPCCoder::~Base64RPCCoder() = default;

/**
 * Tries to decode ASCII complained string to the raw bytes
 * @param encoded ASCII complained base64 encoded string
 * @return vector of raw bytes << allocated on encoded.size() / 4 * 3 + 1 size
 */
std::vector<char> Base64RPCCoder::decode(const std::string &encoded) {
    if (encoded.length() > (std::numeric_limits<size_t>::max() / 3) * 4)
        throw std::length_error("Base64 input too large");
    DLOG(INFO) << "Base64RPCCoder::decode";
    std::vector<char> result((encoded.length() >> 2) * 3 + 1);
    size_t result_len = 0;
    base64_decode(encoded.data(), encoded.size(), result.data(), &result_len,
                  0);
    DLOG(INFO) << "result_len: " << result_len;
    result.resize(result_len);
    return result;
}
/**
 *
 * @param data raw byte stream
 * @return encoded base64 string
 */
std::string Base64RPCCoder::encode(const std::vector<char> &data) {
    if (data.size() > (std::numeric_limits<size_t>::max() / 4) * 3)
        throw std::length_error("raw input is too large");
    DLOG(INFO) << "Base64RPCCoder::encode";
    size_t result_len = 0;
    std::string result((data.size() + 2) / 3 * 4, 0);
    base64_encode(data.data(), data.size(), result.data(), &result_len, 0);
    DLOG(INFO) << "result_len: " << result_len;
    result.resize(result_len);
    return result;
}
} // namespace score

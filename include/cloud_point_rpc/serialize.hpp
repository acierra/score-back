#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace score {

template <typename T>
concept NumericType = requires(T param) {
    requires std::is_integral_v<T> || std::is_floating_point_v<T>;
    requires !std::is_same_v<bool, T>;
    requires std::is_arithmetic_v<decltype(param + 1)>;
    requires !std::is_pointer_v<T>;
};

template <NumericType T> std::vector<uint8_t> serialize(const T &data) {
    std::vector<uint8_t> buffer;
    buffer.resize(sizeof(data));

    std::memcpy(buffer.data(), &data, sizeof(data));

    return buffer;
}

inline void inplace_size_embedding(std::string &str) {
    auto size = str.size();
    auto tmp  = serialize(size);
    str.insert(str.begin(), tmp.begin(), tmp.end());
}

template <NumericType T> T deserialize(const std::vector<uint8_t> &buffer) {
    return *reinterpret_cast<const T *>(buffer.data());
}

} // namespace score
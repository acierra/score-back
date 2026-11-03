//
// Created by vptyp on 12.03.2026.
//
#pragma once
#include <vector>

namespace score {

struct ImageRPC {
    int width{0};
    int height{0};
    enum class Type {
        UNKNOWN,
        BGR,
        RGBA,
        DEPTH,
    } type{Type::UNKNOWN};
    std::vector<unsigned char> data;
};

} // namespace score
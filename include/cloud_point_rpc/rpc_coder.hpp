//
// Created by vptyp on 11.03.2026.
//

#pragma once

#include "export.h"
#include <string>
#include <vector>
namespace score {

class CRPC_EXPORT IRPCCoder {
  public:
    virtual ~IRPCCoder()                                         = default;
    virtual std::vector<char> decode(const std::string &encoded) = 0;
    virtual std::string encode(const std::vector<char> &data)    = 0;
};

class CRPC_EXPORT Base64RPCCoder final : public IRPCCoder {
  public:
    Base64RPCCoder();
    ~Base64RPCCoder() override;

    std::vector<char> decode(const std::string &encoded) override;
    std::string encode(const std::vector<char> &data) override;
};

} // namespace score

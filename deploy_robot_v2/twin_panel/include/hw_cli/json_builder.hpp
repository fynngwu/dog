#pragma once

#include <string>

#include "hw_cli/types.hpp"

namespace hw_cli {

std::string ToJson(const ResultPayload& payload);

}  // namespace hw_cli

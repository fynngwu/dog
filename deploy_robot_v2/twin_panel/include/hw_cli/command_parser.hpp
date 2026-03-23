#pragma once

#include "hw_cli/types.hpp"

namespace hw_cli {

bool ParseArgs(int argc, char** argv, CommandLineArgs& out, std::string& err);
bool ParseActionCsv(const std::string& csv, JointArray& out, std::string& err);

}  // namespace hw_cli

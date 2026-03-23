#include "hw_cli/command_parser.hpp"

#include <cstdlib>
#include <sstream>

namespace hw_cli {

bool ParseActionCsv(const std::string& csv, JointArray& out, std::string& err) {
    std::stringstream ss(csv);
    std::string item;
    int idx = 0;
    while (std::getline(ss, item, ',')) {
        if (idx >= static_cast<int>(out.size())) {
            err = "too many action values";
            return false;
        }
        try {
            out[idx++] = std::stof(item);
        } catch (...) {
            err = "bad float in --action";
            return false;
        }
    }
    if (idx != static_cast<int>(out.size())) {
        err = "need exactly 12 action values";
        return false;
    }
    return true;
}

bool ParseArgs(int argc, char** argv, CommandLineArgs& out, std::string& err) {
    if (argc < 2) {
        err = "missing verb";
        return false;
    }
    out.verb = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--json") {
            out.json = true;
        } else if (arg == "--low-gain") {
            out.low_gain = true;
        } else if (arg == "--auto-report") {
            out.auto_report = true;
        } else if (arg == "--ensure-enabled") {
            out.ensure_enabled = true;
        } else if (arg == "--sample-delay-ms" && i + 1 < argc) {
            out.sample_delay_ms = std::max(0, std::atoi(argv[++i]));
        } else if (arg == "--seconds" && i + 1 < argc) {
            out.seconds = std::stof(argv[++i]);
        } else if (arg == "--action" && i + 1 < argc) {
            out.action_csv = argv[++i];
        } else {
            err = "unknown argument: " + arg;
            return false;
        }
    }
    return true;
}

}  // namespace hw_cli

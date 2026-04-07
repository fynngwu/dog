#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "twin_agent.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;

void OnSignal(int) {
    g_stop = 1;
}

bool ParsePort(const char* text, int& port) {
    try {
        const int value = std::stoi(text);
        if (value <= 0 || value > 65535) {
            return false;
        }
        port = value;
        return true;
    } catch (...) {
        return false;
    }
}
}  // namespace

int main(int argc, char** argv) {
    int cmd_port = 47001;
    int state_port = 47002;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--cmd-port" && i + 1 < argc) {
            if (!ParsePort(argv[++i], cmd_port)) {
                std::cerr << "invalid --cmd-port" << std::endl;
                return 2;
            }
        } else if (arg == "--state-port" && i + 1 < argc) {
            if (!ParsePort(argv[++i], state_port)) {
                std::cerr << "invalid --state-port" << std::endl;
                return 2;
            }
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: dog_debug_daemon [--cmd-port 47001] [--state-port 47002]\n";
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << std::endl;
            return 2;
        }
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    twin::TwinAgent agent(cmd_port, state_port);
    if (!agent.Start()) {
        return 1;
    }

    while (!g_stop && agent.IsRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    agent.Stop();
    return 0;
}

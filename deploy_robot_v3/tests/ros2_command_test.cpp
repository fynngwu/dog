#include "ros2_command_source.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

std::atomic<bool> g_running{true};

namespace {
void signal_handler(int) {
    g_running = false;
}
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    minimal::ROS2CommandSource cmd_source;
    if (!cmd_source.Initialize()) {
        std::cerr << "[ros2_command_test] initialize failed" << std::endl;
        return 1;
    }

    std::cout << "[ros2_command_test] listening on /cmd_vel, press Ctrl+C to stop" << std::endl;
    while (g_running) {
        const auto cmd = cmd_source.GetCommand();
        std::cout << "[ros2_command_test] cmd=[" << cmd[0] << ", " << cmd[1] << ", " << cmd[2] << "]" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    cmd_source.Stop();
    return 0;
}

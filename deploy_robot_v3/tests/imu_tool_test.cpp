#include "imu_reader.hpp"

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

    minimal::IMUReader imu;
    if (!imu.Initialize()) {
        std::cerr << "[imu_tool_test] initialize failed" << std::endl;
        return 1;
    }

    std::cout << "[imu_tool_test] reading imu, press Ctrl+C to stop" << std::endl;
    while (g_running) {
        imu.PrintDebug();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    imu.Stop();
    return 0;
}

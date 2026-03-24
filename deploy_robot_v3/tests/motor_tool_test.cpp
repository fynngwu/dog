#include "motor_io.hpp"

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

    minimal::MotorIO motor_io;
    if (!motor_io.Initialize()) {
        std::cerr << "[motor_tool_test] initialize failed" << std::endl;
        return 1;
    }
    if (!motor_io.EnableAndMoveToOffsets(2.0f)) {
        std::cerr << "[motor_tool_test] enable/move failed" << std::endl;
        return 1;
    }

    std::cout << "[motor_tool_test] printing joint observation, press Ctrl+C to stop" << std::endl;
    while (g_running) {
        const auto joint_obs = motor_io.GetJointObs();
        std::cout << "[motor_tool_test] pos0=" << joint_obs[0]
                  << " vel0=" << joint_obs[12]
                  << " healthy=" << (motor_io.AllMotorsHealthy(100) ? "Y" : "N")
                  << std::endl;
        motor_io.HoldOffsets();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    for (int i = 0; i < 5; ++i) {
        motor_io.HoldOffsets();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}

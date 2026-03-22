/**
 * @file inspect_feedback_main.cpp
 * @brief 模式1：检测电机反馈是否正常
 * 
 * 流程：
 * 1. BindCAN
 * 2. BindMotor
 * 3. EnableMotor
 * 4. EnableAutoReport
 * 5. 等待反馈 online
 * 6. 打印每个电机 position/velocity/torque
 * 7. 退出
 * 
 * 用途：验证通信链路是否正常
 */

#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include "robstride.hpp"
#include "can_interface.hpp"
#include "robot_config.hpp"

int main() {
    std::cout << "=== Inspect Feedback Tool ===" << std::endl;
    std::cout << "Purpose: Verify motor communication and feedback" << std::endl;
    std::cout << std::endl;

    // 创建控制器
    auto controller = std::make_shared<RobstrideController>();

    // 绑定 CAN 接口
    std::cout << "[Step 1] Binding CAN interfaces..." << std::endl;
    for (char cid : cfg::kCanIds) {
        std::string name = std::string("candle") + cid;
        auto iface = std::make_shared<CANInterface>(name.c_str());
        controller->BindCAN(iface);
        std::cout << "  Bound: " << name << std::endl;
    }

    // 绑定电机
    std::cout << "[Step 2] Binding motors..." << std::endl;
    std::vector<int> motor_indices(cfg::kNumMotors, -1);
    
    for (int j = 0; j < 4; ++j) {
        std::string can_if = std::string("candle") + cfg::kCanIds[j];
        for (int i = 0; i < 3; ++i) {
            int gid = j + i * 4;
            
            auto mi = std::make_unique<RobstrideController::MotorInfo>();
            mi->motor_id = cfg::kMotorIds[gid];
            mi->host_id = 0xFD;

            int idx = controller->BindMotor(can_if.c_str(), std::move(mi));
            motor_indices[gid] = idx;
            std::cout << "  Motor ID " << cfg::kMotorIds[gid] << " -> index " << idx << std::endl;
        }
    }

    // 使能电机
    std::cout << "[Step 3] Enabling motors..." << std::endl;
    for (int idx : motor_indices) {
        controller->EnableMotor(idx);
        controller->SetMITParams(idx, {cfg::kMitKp, cfg::kMitKd, cfg::kMitVelLimit, cfg::kMitTorqueLimit});
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 开启自动上报
    std::cout << "[Step 4] Enabling auto report..." << std::endl;
    for (int idx : motor_indices) {
        controller->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        controller->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 等待反馈
    std::cout << "[Step 5] Waiting for feedback (3s timeout)..." << std::endl;
    std::vector<bool> received(cfg::kNumMotors, false);
    auto t0 = std::chrono::steady_clock::now();
    const int timeout_ms = 3000;

    while (true) {
        int valid_cnt = 0;
        for (size_t i = 0; i < motor_indices.size(); ++i) {
            if (received[i]) {
                valid_cnt++;
                continue;
            }
            // 使用 online 标志判断是否收到反馈
            if (controller->IsMotorOnline(motor_indices[i])) {
                received[i] = true;
                valid_cnt++;
            }
        }

        if (valid_cnt == cfg::kNumMotors) {
            std::cout << "  All motors online!" << std::endl;
            break;
        }

        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (dt > timeout_ms) {
            std::cerr << "  Timeout! Missing motors: ";
            for (size_t i = 0; i < received.size(); ++i) {
                if (!received[i]) std::cerr << i << " ";
            }
            std::cerr << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 打印状态
    std::cout << std::endl;
    std::cout << "[Step 6] Motor states:" << std::endl;
    std::cout << std::setw(8) << "Index" 
              << std::setw(8) << "ID"
              << std::setw(12) << "Position"
              << std::setw(12) << "Velocity"
              << std::setw(12) << "Torque" << std::endl;
    std::cout << std::string(52, '-') << std::endl;
    
    for (int i = 0; i < cfg::kNumMotors; ++i) {
        auto s = controller->GetMotorState(motor_indices[i]);
        std::cout << std::setw(8) << i 
                  << std::setw(8) << cfg::kMotorIds[i]
                  << std::setw(12) << std::fixed << std::setprecision(4) << s.position
                  << std::setw(12) << s.velocity
                  << std::setw(12) << s.torque << std::endl;
    }

    std::cout << std::endl;
    std::cout << "=== Inspect Complete ===" << std::endl;
    std::cout << "All motors feedback verified." << std::endl;
    
    return 0;
}
/**
 * @file zero_calibration_main.cpp
 * @brief 单独的校零工具
 * 
 * 警告：此工具会对电机发送 SetZero 命令！
 * 
 * 用途：
 * - 在电机安装或维护时校准零点
 * - 绝对不要在正常 policy bringup 中调用
 * 
 * 流程：
 * 1. 绑定 CAN 和电机
 * 2. 使能电机
 * 3. 等待所有电机反馈在线
 * 4. 确认用户操作
 * 5. 发送 SetZero
 * 6. 验证零点
 */

#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

#include "robstride.hpp"
#include "can_interface.hpp"
#include "robot_config.hpp"

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   ZERO CALIBRATION TOOL" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "WARNING: This tool will send SetZero commands to motors!" << std::endl;
    std::cout << "         This should ONLY be used during motor installation" << std::endl;
    std::cout << "         or maintenance." << std::endl;
    std::cout << std::endl;
    std::cout << "DO NOT use this during normal policy operation!" << std::endl;
    std::cout << std::endl;
    std::cout << "Press Enter to continue or Ctrl+C to abort..." << std::endl;
    std::cin.get();

    // 创建控制器
    auto controller = std::make_shared<RobstrideController>();

    // 绑定 CAN
    std::cout << "[Step 1] Binding CAN interfaces..." << std::endl;
    for (char cid : cfg::kCanIds) {
        std::string name = std::string("candle") + cid;
        auto iface = std::make_shared<CANInterface>(name.c_str());
        if (!iface->IsValid()) {
            std::cerr << "  ERROR: Failed to open " << name << std::endl;
            return 1;
        }
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

    // 等待反馈 - 使用真正的 online 检查
    std::cout << "[Step 5] Waiting for feedback (5s timeout)..." << std::endl;
    std::vector<bool> received(cfg::kNumMotors, false);
    auto t0 = std::chrono::steady_clock::now();
    const int timeout_ms = 5000;
    
    while (true) {
        int valid_cnt = 0;
        for (size_t i = 0; i < motor_indices.size(); ++i) {
            if (received[i]) {
                valid_cnt++;
                continue;
            }
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
            std::cerr << "  ERROR: Feedback timeout! Missing motors: ";
            for (size_t i = 0; i < received.size(); ++i) {
                if (!received[i]) std::cerr << i << " ";
            }
            std::cerr << std::endl;
            std::cerr << "  Cannot proceed with calibration. Check motor connections." << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 打印当前位置
    std::cout << std::endl;
    std::cout << "[Current Positions BEFORE SetZero]" << std::endl;
    for (int i = 0; i < cfg::kNumMotors; ++i) {
        auto s = controller->GetMotorState(motor_indices[i]);
        std::cout << "  Motor " << i << " (ID " << cfg::kMotorIds[i] << "): " 
                  << std::fixed << std::setprecision(4) << s.position << " rad" << std::endl;
    }

    // 确认
    std::cout << std::endl;
    std::cout << "Are you sure you want to set these positions as ZERO?" << std::endl;
    std::cout << "Type 'YES' to confirm: ";
    
    std::string confirm;
    std::cin >> confirm;
    
    if (confirm != "YES") {
        std::cout << "Aborted." << std::endl;
        return 0;
    }

    // 发送 SetZero
    std::cout << std::endl;
    std::cout << "[Step 6] Sending SetZero commands..." << std::endl;
    for (int i = 0; i < cfg::kNumMotors; ++i) {
        std::cout << "  Setting zero for motor " << i << "..." << std::endl;
        controller->SetZero(motor_indices[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        controller->SetZero(motor_indices[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 等待生效
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 打印校零后位置
    std::cout << std::endl;
    std::cout << "[Current Positions AFTER SetZero]" << std::endl;
    for (int i = 0; i < cfg::kNumMotors; ++i) {
        auto s = controller->GetMotorState(motor_indices[i]);
        std::cout << "  Motor " << i << " (ID " << cfg::kMotorIds[i] << "): " 
                  << std::fixed << std::setprecision(4) << s.position << " rad" << std::endl;
    }

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "   ZERO CALIBRATION COMPLETE" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
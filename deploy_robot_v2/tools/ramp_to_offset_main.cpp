/**
 * @file ramp_to_offset_main.cpp
 * @brief 模式2：安全上电并平滑移动到 offset 姿态
 * 
 * 流程：
 * 1. Enable + AutoReport
 * 2. 等待反馈 online
 * 3. 读取当前 12 个关节的绝对角度
 * 4. 用插值从 current_abs_pos 平滑移动到 joint_offsets
 * 5. hold 住 offset
 * 6. 退出
 * 
 * 注意：绝对不调用 SetZero()
 * 
 * 用途：解决"上电先冲零点再回 offset"的问题
 */

#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

#include "robstride.hpp"
#include "can_interface.hpp"
#include "robot_config.hpp"
#include "robot_bringup.hpp"

int main() {
    std::cout << "=== Ramp to Offset Tool ===" << std::endl;
    std::cout << "Purpose: Safe power-on and smooth move to standing pose" << std::endl;
    std::cout << "WARNING: This tool does NOT call SetZero()" << std::endl;
    std::cout << std::endl;

    // 创建控制器和启动器
    auto controller = std::make_shared<RobstrideController>();
    RobotBringup bringup(controller);

    // 绑定 CAN
    std::cout << "[Step 1] Binding CAN interfaces..." << std::endl;
    if (!bringup.BindAllCAN()) {
        std::cerr << "Failed to bind CAN interfaces" << std::endl;
        return 1;
    }

    // 绑定电机
    std::cout << "[Step 2] Binding motors..." << std::endl;
    std::vector<int> motor_indices;
    if (!bringup.BindAllMotors(motor_indices)) {
        std::cerr << "Failed to bind motors" << std::endl;
        return 1;
    }

    // 使能电机
    std::cout << "[Step 3] Enabling motors..." << std::endl;
    if (!bringup.EnableAll(motor_indices)) {
        std::cerr << "Failed to enable motors" << std::endl;
        return 1;
    }

    // 开启自动上报
    std::cout << "[Step 4] Enabling auto report..." << std::endl;
    if (!bringup.EnableAutoReportAll(motor_indices)) {
        std::cerr << "Failed to enable auto report" << std::endl;
        return 1;
    }

    // 等待反馈
    std::cout << "[Step 5] Waiting for feedback..." << std::endl;
    if (!bringup.WaitForFeedback(motor_indices, 3000)) {
        std::cerr << "Feedback timeout" << std::endl;
        return 1;
    }

    // 读取当前位置
    std::cout << "[Step 6] Reading current positions..." << std::endl;
    auto current_pos = bringup.ReadCurrentAbsPositions(motor_indices);
    
    std::cout << "  Current positions (rad):" << std::endl;
    for (int i = 0; i < cfg::kNumMotors; ++i) {
        std::cout << "    Motor " << i << ": " << std::fixed << std::setprecision(4) 
                  << current_pos[i] << std::endl;
    }

    // 平滑移动到 offset
    std::cout << "[Step 7] Moving to offset pose (2 seconds)..." << std::endl;
    if (!bringup.MoveCurrentToOffsets(motor_indices, 2.0f)) {
        std::cerr << "Failed to move to offsets" << std::endl;
        return 1;
    }

    // Hold 在 offset
    std::cout << "[Step 8] Holding offset pose (press Ctrl+C to exit)..." << std::endl;
    std::cout << "  Motor states:" << std::endl;
    
    while (true) {
        bringup.HoldOffsets(motor_indices);
        
        // 打印状态
        std::cout << "\r  ";
        for (int i = 0; i < cfg::kNumMotors; ++i) {
            auto s = controller->GetMotorState(motor_indices[i]);
            std::cout << std::fixed << std::setprecision(2) << s.position << " ";
        }
        std::cout << std::flush;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
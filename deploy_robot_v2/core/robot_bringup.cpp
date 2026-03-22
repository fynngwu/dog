#include "robot_bringup.hpp"
#include "robot_config.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

RobotBringup::RobotBringup(std::shared_ptr<RobstrideController> controller)
    : controller_(std::move(controller)) {}

bool RobotBringup::BindAllCAN() {
    for (char cid : cfg::kCanIds) {
        std::string name = std::string("candle") + cid;
        auto iface = std::make_shared<CANInterface>(name.c_str());
        if (!iface || !iface->IsValid()) {
            std::cerr << "[RobotBringup] Failed to open CAN interface: " << name << std::endl;
            return false;
        }
        controller_->BindCAN(iface);
        can_ifaces_.push_back(iface);
        std::cout << "[RobotBringup] Bound CAN interface: " << name << std::endl;
    }
    return true;
}

bool RobotBringup::BindAllMotors(std::vector<int>& motor_indices) {
    motor_indices.resize(cfg::kNumMotors, -1);
    
    for (int j = 0; j < 4; ++j) {
        std::string can_if = std::string("candle") + cfg::kCanIds[j];
        for (int i = 0; i < 3; ++i) {
            int gid = j + i * 4;
            
            auto mi = std::make_unique<RobstrideController::MotorInfo>();
            mi->motor_id = cfg::kMotorIds[gid];
            mi->host_id = 0xFD;
            mi->max_torque = cfg::kMitTorqueLimit;
            mi->max_speed = cfg::kMitVelLimit;
            mi->max_kp = 500.0f;
            mi->max_kd = 5.0f;

            int idx = controller_->BindMotor(can_if.c_str(), std::move(mi));
            if (idx < 0) {
                std::cerr << "[RobotBringup] Failed to bind motor " << cfg::kMotorIds[gid] 
                          << " on " << can_if << std::endl;
                return false;
            }
            motor_indices[gid] = idx;
            std::cout << "[RobotBringup] Bound motor ID " << cfg::kMotorIds[gid] 
                      << " -> index " << idx << std::endl;
        }
    }
    return true;
}

bool RobotBringup::EnableAll(const std::vector<int>& motor_indices, bool low_gain, int retry_count) {
    const int kWaitMs = 50;  // 每次使能后等待反馈的时间

    for (int idx : motor_indices) {
        bool enabled = false;
        for (int attempt = 1; attempt <= retry_count; ++attempt) {
            // 发送使能命令
            if (controller_->EnableMotor(idx) != 0) {
                std::cerr << "[RobotBringup] Enable motor index " << idx
                          << " send failed (attempt " << attempt << "/" << retry_count << ")" << std::endl;
                if (attempt < retry_count) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                continue;
            }

            // 等待电机反馈确认
            std::this_thread::sleep_for(std::chrono::milliseconds(kWaitMs));
            if (controller_->IsMotorOnline(idx)) {
                enabled = true;
                break;
            }

            std::cerr << "[RobotBringup] Enable motor index " << idx
                      << " no feedback (attempt " << attempt << "/" << retry_count << ")" << std::endl;
        }

        if (!enabled) {
            std::cerr << "[RobotBringup] Failed to enable motor index " << idx
                      << " after " << retry_count << " attempts" << std::endl;
            return false;
        }

        if (low_gain) {
            // 低增益模式：用于地面排查
            controller_->SetMITParams(idx, {10.0f, 0.3f, cfg::kMitVelLimit, cfg::kMitTorqueLimit});
        } else {
            // 正常增益模式
            controller_->SetMITParams(idx, {
                cfg::kMitKp, cfg::kMitKd, cfg::kMitVelLimit, cfg::kMitTorqueLimit
            });
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[RobotBringup] All motors enabled"
              << (low_gain ? " (LOW-GAIN mode: kp=10, kd=0.3)" : "") << std::endl;
    return true;
}

bool RobotBringup::EnableAutoReportAll(const std::vector<int>& motor_indices) {
    for (int idx : motor_indices) {
        // 发送两次更稳定 (保持与现有代码一致)
        controller_->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        controller_->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "[RobotBringup] Auto report enabled for all motors" << std::endl;
    return true;
}

bool RobotBringup::WaitForFeedback(const std::vector<int>& motor_indices, int timeout_ms) {
    auto t0 = std::chrono::steady_clock::now();
    std::vector<bool> received(motor_indices.size(), false);
    
    std::cout << "[RobotBringup] Waiting for feedback..." << std::endl;
    
    while (true) {
        int valid_cnt = 0;
        for (size_t i = 0; i < motor_indices.size(); ++i) {
            if (received[i]) {
                valid_cnt++;
                continue;
            }
            // 使用 online 标志判断是否收到过反馈
            if (controller_->IsMotorOnline(motor_indices[i])) {
                received[i] = true;
                valid_cnt++;
            }
        }
        
        if (valid_cnt == static_cast<int>(motor_indices.size())) {
            std::cout << "[RobotBringup] All motors feedback online" << std::endl;
            return true;
        }

        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (dt > timeout_ms) {
            std::cerr << "[RobotBringup] Feedback timeout. Missing motors: ";
            for (size_t i = 0; i < motor_indices.size(); ++i) {
                if (!received[i]) {
                    std::cerr << i << " ";
                }
            }
            std::cerr << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::vector<float> RobotBringup::ReadCurrentAbsPositions(const std::vector<int>& motor_indices) const {
    std::vector<float> q(motor_indices.size(), 0.0f);
    for (size_t i = 0; i < motor_indices.size(); ++i) {
        q[i] = controller_->GetMotorState(motor_indices[i]).position;
    }
    return q;
}

bool RobotBringup::SmoothMoveAbs(const std::vector<int>& motor_indices,
                                 const std::vector<float>& from_abs,
                                 const std::vector<float>& to_abs,
                                 float duration_sec,
                                 int hz,
                                 StopCondition stop_condition) {
    if (motor_indices.size() != from_abs.size() || motor_indices.size() != to_abs.size()) {
        std::cerr << "[RobotBringup] SmoothMoveAbs: size mismatch" << std::endl;
        return false;
    }

    int steps = std::max(1, static_cast<int>(duration_sec * hz));
    int period_ms = 1000 / hz;

    std::cout << "[RobotBringup] Smooth move: " << steps << " steps at " << hz << " Hz" << std::endl;

    for (int s = 1; s <= steps; ++s) {
        // 检查停止条件
        if (stop_condition && stop_condition()) {
            std::cerr << "[RobotBringup] Smooth move interrupted by stop condition" << std::endl;
            return false;
        }

        // 检查电机是否在线
        if (!controller_->AllMotorsOnlineFresh(motor_indices, 200)) {
            std::cerr << "[RobotBringup] Smooth move interrupted: motor offline or stale feedback" << std::endl;
            EmergencyHoldAll(motor_indices, 3);
            return false;
        }

        float a = static_cast<float>(s) / static_cast<float>(steps);
        // 使用平滑插值 (ease-in-out)
        float smooth_a = a * a * (3.0f - 2.0f * a);

        for (size_t i = 0; i < motor_indices.size(); ++i) {
            float cmd = (1.0f - smooth_a) * from_abs[i] + smooth_a * to_abs[i];
            int ret = controller_->SendMITCommand(motor_indices[i], cmd);
            if (ret != 0) {
                std::cerr << "[RobotBringup] Smooth move interrupted: send command failed for motor " << i << std::endl;
                EmergencyHoldAll(motor_indices, 3);
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    std::cout << "[RobotBringup] Smooth move completed" << std::endl;
    return true;
}

bool RobotBringup::MoveCurrentToOffsets(const std::vector<int>& motor_indices,
                                        float duration_sec,
                                        StopCondition stop_condition) {
    auto q0 = ReadCurrentAbsPositions(motor_indices);

    std::cout << "[RobotBringup] Current positions: ";
    for (float q : q0) std::cout << q << " ";
    std::cout << std::endl;

    std::cout << "[RobotBringup] Target offsets: ";
    for (float q : cfg::kJointOffsets) std::cout << q << " ";
    std::cout << std::endl;

    return SmoothMoveAbs(motor_indices, q0, cfg::kJointOffsets, duration_sec, 100, stop_condition);
}

bool RobotBringup::HoldOffsets(const std::vector<int>& motor_indices) {
    bool ok = true;
    for (size_t i = 0; i < motor_indices.size(); ++i) {
        if (controller_->SendMITCommand(motor_indices[i], cfg::kJointOffsets[i]) != 0) {
            ok = false;
        }
    }
    return ok;
}

bool RobotBringup::EmergencyHoldAll(const std::vector<int>& motor_indices, int repeat) {
    bool ok = true;
    for (int r = 0; r < repeat; ++r) {
        ok = HoldOffsets(motor_indices) && ok;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return ok;
}

bool CheckSensorsReady(const std::shared_ptr<IMUComponent>& imu,
                       const std::shared_ptr<RobstrideController>& controller,
                       const std::vector<int>& motor_indices,
                       int motor_fresh_ms,
                       int imu_fresh_ms) {
    if (!imu->IsReady() || !imu->IsFresh(imu_fresh_ms)) {
        return false;
    }
    if (!controller->AllMotorsOnlineFresh(motor_indices, motor_fresh_ms)) {
        return false;
    }
    return true;
}
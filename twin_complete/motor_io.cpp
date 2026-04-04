#include "motor_io.hpp"
#include "can_interface.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace minimal {

MotorIO::MotorIO() : controller_(std::make_shared<RobstrideController>()) {
    motor_indices_.resize(kNumMotors, -1);
}

MotorIO::~MotorIO() = default;

bool MotorIO::Initialize() {
    for (char cid : kCanIds) {
        std::string name = std::string("candle") + cid;
        auto iface = std::make_shared<CANInterface>(name.c_str());
        if (!iface || !iface->IsValid()) {
            std::cerr << "[MotorIO] Failed to open CAN interface: " << name << std::endl;
            return false;
        }
        controller_->BindCAN(iface);
        std::cout << "[MotorIO] Bound CAN interface: " << name << std::endl;
    }

    for (int j = 0; j < 4; ++j) {
        std::string can_if = std::string("candle") + kCanIds[j];
        for (int i = 0; i < 3; ++i) {
            int gid = j + i * 4;

            auto mi = std::make_unique<RobstrideController::MotorInfo>();
            mi->motor_id = kMotorIds[gid];
            mi->host_id = 0xFD;
            mi->max_torque = kMitTorqueLimit;
            mi->max_speed = kMitVelLimit;
            mi->max_kp = 500.0f;
            mi->max_kd = 5.0f;

            int idx = controller_->BindMotor(can_if.c_str(), std::move(mi));
            if (idx < 0) {
                std::cerr << "[MotorIO] Failed to bind motor ID " << kMotorIds[gid]
                          << " on " << can_if << std::endl;
                return false;
            }
            motor_indices_[gid] = idx;
            std::cout << "[MotorIO] Bound motor ID " << kMotorIds[gid]
                      << " -> index " << idx << std::endl;
        }
    }

    return true;
}

void MotorIO::ConfigureMITParams() {
    for (int idx : motor_indices_) {
        MIT_params params;
        params.kp = kMitKp;
        params.kd = kMitKd;
        params.vel_limit = kMitVelLimit;
        params.torque_limit = kMitTorqueLimit;
        controller_->SetMITParams(idx, params);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

bool MotorIO::EnableAll() {
    constexpr int kRetryCount = 20;
    constexpr int kWaitMs = 50;

    for (int idx : motor_indices_) {
        bool enabled = false;
        for (int attempt = 1; attempt <= kRetryCount; ++attempt) {
            if (controller_->EnableMotor(idx) != 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(kWaitMs));
            if (controller_->IsMotorOnline(idx)) {
                enabled = true;
                break;
            }
        }

        if (!enabled) {
            std::cerr << "[MotorIO] Failed to enable motor index " << idx << std::endl;
            return false;
        }
    }

    ConfigureMITParams();

    for (int idx : motor_indices_) {
        controller_->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        controller_->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    auto t0 = std::chrono::steady_clock::now();
    constexpr int kTimeoutMs = 3000;
    while (true) {
        int valid_cnt = 0;
        for (int idx : motor_indices_) {
            if (controller_->IsMotorOnline(idx)) valid_cnt++;
        }
        if (valid_cnt == static_cast<int>(motor_indices_.size())) {
            break;
        }

        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (dt > kTimeoutMs) {
            std::cerr << "[MotorIO] Feedback timeout" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int idx : motor_indices_) {
        controller_->DisableAutoReport(idx);
    }

    std::cout << "[MotorIO] All motors enabled and online" << std::endl;
    return true;
}

bool MotorIO::DisableAll() {
    bool ok = true;
    for (int idx : motor_indices_) {
        if (controller_->DisableMotor(idx) != 0) {
            ok = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return ok;
}

bool MotorIO::MoveToOffset(float duration_sec) {
    auto current_pos = ReadCurrentPositions();
    return SmoothMove(current_pos,
                      std::vector<float>(kJointOffsets, kJointOffsets + kNumMotors),
                      duration_sec, 100);
}

bool MotorIO::EnableAndMoveToOffset(float duration_sec) {
    return EnableAll() && MoveToOffset(duration_sec);
}

std::vector<float> MotorIO::GetJointObs() const {
    std::vector<float> obs(kNumMotors * 2, 0.0f);

    for (int i = 0; i < kNumMotors; ++i) {
        auto state = controller_->GetMotorState(motor_indices_[i]);
        float pos = state.position;
        float vel = state.velocity;

        if (i >= 8 && i <= 11) {
            obs[i] = kJointDirection[i] * ((pos - kJointOffsets[i]) / kKneeRatio);
            obs[kNumMotors + i] = kJointDirection[i] * (vel / kKneeRatio);
        } else {
            obs[i] = kJointDirection[i] * (pos - kJointOffsets[i]);
            obs[kNumMotors + i] = kJointDirection[i] * vel;
        }
    }

    return obs;
}

std::vector<float> MotorIO::ComputeTargetPositions(
    const std::vector<float>& actions, float action_scale) const {

    std::vector<float> targets(kNumMotors, 0.0f);

    for (int i = 0; i < kNumMotors; ++i) {
        float act = actions[i];
        if (i >= 8 && i <= 11) {
            act *= kKneeRatio;
        }

        float desired = kJointDirection[i] * act * action_scale + kJointOffsets[i];
        float lower = kJointOffsets[i] + kXmlMin[i];
        float upper = kJointOffsets[i] + kXmlMax[i];
        targets[i] = std::clamp(desired, lower, upper);
    }

    return targets;
}

bool MotorIO::SendActions(const std::vector<float>& actions, float action_scale) {
    if (actions.size() != static_cast<size_t>(kNumMotors)) {
        std::cerr << "[MotorIO] Action size mismatch: expected " << kNumMotors
                  << ", got " << actions.size() << std::endl;
        return false;
    }

    auto targets = ComputeTargetPositions(actions, action_scale);

    for (int i = 0; i < kNumMotors; ++i) {
        if (controller_->SendMITCommand(motor_indices_[i], targets[i]) != 0) {
            std::cerr << "[MotorIO] Send command failed for motor " << i << std::endl;
            return false;
        }
    }
    return true;
}

bool MotorIO::AllMotorsHealthy(int max_age_ms) const {
    return controller_->AllMotorsOnlineFresh(motor_indices_, max_age_ms);
}

int MotorIO::CountOnlineMotors(int max_age_ms) const {
    int cnt = 0;
    for (int idx : motor_indices_) {
        auto age = controller_->GetLastOnlineAgeMs(idx);
        if (age >= 0 && age <= max_age_ms) cnt++;
    }
    return cnt;
}

int MotorIO::MaxFeedbackAgeMs() const {
    int max_age = -1;
    for (int idx : motor_indices_) {
        auto age = controller_->GetLastOnlineAgeMs(idx);
        if (age > max_age) max_age = static_cast<int>(age);
    }
    return max_age;
}

void MotorIO::GetMotorStates(std::vector<float>& positions,
                             std::vector<float>& velocities,
                             std::vector<float>& torques) const {
    controller_->GetAllMotorStates(motor_indices_, positions, velocities, torques);
}

std::vector<float> MotorIO::ReadCurrentPositions() const {
    std::vector<float> positions(kNumMotors, 0.0f);
    for (int i = 0; i < kNumMotors; ++i) {
        positions[i] = controller_->GetMotorState(motor_indices_[i]).position;
    }
    return positions;
}

bool MotorIO::SmoothMove(const std::vector<float>& from,
                         const std::vector<float>& to,
                         float duration_sec, int hz) {
    if (from.size() != static_cast<size_t>(kNumMotors) ||
        to.size() != static_cast<size_t>(kNumMotors)) {
        std::cerr << "[MotorIO] SmoothMove size mismatch" << std::endl;
        return false;
    }

    int steps = std::max(1, static_cast<int>(duration_sec * hz));
    int period_ms = std::max(1, 1000 / hz);

    for (int s = 1; s <= steps; ++s) {
        if (!AllMotorsHealthy(200)) {
            std::cerr << "[MotorIO] Motor offline during smooth move" << std::endl;
            return false;
        }

        float a = static_cast<float>(s) / static_cast<float>(steps);
        float smooth_a = a * a * (3.0f - 2.0f * a);

        for (int i = 0; i < kNumMotors; ++i) {
            float cmd = (1.0f - smooth_a) * from[i] + smooth_a * to[i];
            if (controller_->SendMITCommand(motor_indices_[i], cmd) != 0) {
                std::cerr << "[MotorIO] Send failed during smooth move for motor " << i << std::endl;
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
    return true;
}

void MotorIO::EnableAllAutoReport() {
    for (int idx : motor_indices_) {
        controller_->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

bool MotorIO::HoldOffsets() {
    for (int i = 0; i < kNumMotors; ++i) {
        if (controller_->SendMITCommand(motor_indices_[i], kJointOffsets[i]) != 0) {
            return false;
        }
    }
    return true;
}

}  // namespace minimal

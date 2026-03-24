#include "motor_io.hpp"
#include "can_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>

namespace minimal {

MotorIO::MotorIO() : controller_(std::make_shared<RobstrideController>()) {
    motor_indices_.resize(kNumMotors, -1);
}

MotorIO::~MotorIO() = default;

bool MotorIO::Initialize() {
    // Bind CAN interfaces
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

    // Bind motors (4 legs × 3 motors per leg)
    for (int j = 0; j < 4; ++j) {
        std::string can_if = std::string("candle") + kCanIds[j];
        for (int i = 0; i < 3; ++i) {
            int gid = j + i * 4;  // Global motor ID in kMotorIds

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

// Note: Target position is kJointOffsets, the ready-to-stand position.
bool MotorIO::EnableAndMoveToOffsets(float duration_sec) {
    constexpr int kRetryCount = 20;
    constexpr int kWaitMs = 50;

    // 1) Enable all motors with feedback check
    for (int idx : motor_indices_) {
        bool enabled = false;
        for (int attempt = 1; attempt <= kRetryCount; ++attempt) {
            if (controller_->EnableMotor(idx) != 0) {
                std::cerr << "[MotorIO] Enable motor index " << idx
                          << " send failed (attempt " << attempt << "/" << kRetryCount << ")" << std::endl;
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

        // Set MIT parameters
        MIT_params params;
        params.kp = kMitKp;
        params.kd = kMitKd;
        params.vel_limit = kMitVelLimit;
        params.torque_limit = kMitTorqueLimit;
        controller_->SetMITParams(idx, params);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[MotorIO] All motors enabled" << std::endl;

    // 2) Warm up MIT feedback path without moving motors.
    // Temporarily set kp=0, kd=small for harmless damping-only commands.
    // This ensures feedback is fresh before we read current positions.
    for (int idx : motor_indices_) {
        MIT_params damping_only;
        damping_only.kp = 0.0f;
        damping_only.kd = 0.1f;  // Small damping, harmless
        damping_only.vel_limit = kMitVelLimit;
        damping_only.torque_limit = kMitTorqueLimit;
        controller_->SetMITParams(idx, damping_only);
    }

    constexpr int kWarmupCycles = 20;
    for (int n = 0; n < kWarmupCycles; ++n) {
        for (int i = 0; i < kNumMotors; ++i) {
            // Send harmless command (kp=0, so position target doesn't matter)
            if (controller_->SendMITCommand(motor_indices_[i], 0.0f) != 0) {
                std::cerr << "[MotorIO] MIT warmup failed for motor " << i << std::endl;
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!AllMotorsHealthy(200)) {
        std::cerr << "[MotorIO] Motors not healthy after MIT warmup" << std::endl;
        return false;
    }
    std::cout << "[MotorIO] MIT feedback path warmed up (damping-only mode)" << std::endl;

    // 3) Restore normal MIT parameters for position control
    for (int idx : motor_indices_) {
        MIT_params params;
        params.kp = kMitKp;
        params.kd = kMitKd;
        params.vel_limit = kMitVelLimit;
        params.torque_limit = kMitTorqueLimit;
        controller_->SetMITParams(idx, params);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 4) Read real current positions after feedback is fresh
    auto current_pos = ReadCurrentPositions();
    std::cout << "[MotorIO] Current positions: ";
    for (float p : current_pos) std::cout << p << " ";
    std::cout << std::endl;

    std::cout << "[MotorIO] Target offsets: ";
    for (float o : kJointOffsets) std::cout << o << " ";
    std::cout << std::endl;

    // 5) Smoothly move from current pose to offsets
    return SmoothMove(
        current_pos,
        std::vector<float>(kJointOffsets, kJointOffsets + kNumMotors),
        duration_sec,
        100
    );
}

std::vector<float> MotorIO::GetJointObs() const {
    std::vector<float> obs(kNumMotors * 2, 0.0f);  // [12 pos, 12 vel]

    for (int i = 0; i < kNumMotors; ++i) {
        auto state = controller_->GetMotorState(motor_indices_[i]);
        float pos = state.position;
        float vel = state.velocity;

        // Apply direction mapping and gear ratio
        // Policy coordinate = sign * (motor relative coordinate)
        if (i >= 8 && i <= 11) {  // Knee joints
            obs[i] = kJointDirection[i] * ((pos - kJointOffsets[i]) / kKneeRatio);
            obs[kNumMotors + i] = kJointDirection[i] * (vel / kKneeRatio);
        } else {  // HipA/HipF joints
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

        // Apply knee gear ratio
        if (i >= 8 && i <= 11) {
            act *= kKneeRatio;
        }

        // desired_abs = sign * action * scale + offset
        float desired = kJointDirection[i] * act * action_scale + kJointOffsets[i];

        // Clamp to limits
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

    // Best-effort send to all motors, report failure if any
    bool all_success = true;
    for (int i = 0; i < kNumMotors; ++i) {
        if (controller_->SendMITCommand(motor_indices_[i], targets[i]) != 0) {
            std::cerr << "[MotorIO] Send command failed for motor " << i << std::endl;
            all_success = false;
        }
    }

    return all_success;
}

bool MotorIO::AllMotorsHealthy(int max_age_ms) const {
    return controller_->AllMotorsOnlineFresh(motor_indices_, max_age_ms);
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
    int period_ms = 1000 / hz;

    std::cout << "[MotorIO] Smooth move: " << steps << " steps at " << hz << " Hz" << std::endl;

    for (int s = 1; s <= steps; ++s) {
        // Check motor health
        if (!AllMotorsHealthy(200)) {
            std::cerr << "[MotorIO] Motor offline during smooth move, holding current pose" << std::endl;
            HoldCurrentPose();  // Safe-hold on failure
            return false;
        }

        // Smooth interpolation (ease-in-out)
        float a = static_cast<float>(s) / static_cast<float>(steps);
        float smooth_a = a * a * (3.0f - 2.0f * a);

        for (int i = 0; i < kNumMotors; ++i) {
            float cmd = (1.0f - smooth_a) * from[i] + smooth_a * to[i];
            if (controller_->SendMITCommand(motor_indices_[i], cmd) != 0) {
                std::cerr << "[MotorIO] Send failed during smooth move for motor " << i << std::endl;
                HoldCurrentPose();  // Safe-hold on failure
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    std::cout << "[MotorIO] Smooth move completed" << std::endl;
    return true;
}

bool MotorIO::HoldOffsets() {
    for (int i = 0; i < kNumMotors; ++i) {
        if (controller_->SendMITCommand(motor_indices_[i], kJointOffsets[i]) != 0) {
            return false;
        }
    }
    return true;
}

bool MotorIO::HoldCurrentPose() {
    for (int i = 0; i < kNumMotors; ++i) {
        auto state = controller_->GetMotorState(motor_indices_[i]);
        if (controller_->SendMITCommand(motor_indices_[i], state.position) != 0) {
            return false;
        }
    }
    return true;
}

}  // namespace minimal

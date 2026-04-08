#include "motor_io.hpp"

#include "can_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

namespace minimal {

const char* const kJointNames[kNumMotors] = {
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
};

MotorIO::MotorIO() : controller_(std::make_shared<RobstrideController>()) {
    motor_indices_.assign(kNumMotors, -1);
}

MotorIO::~MotorIO() = default;

bool MotorIO::Initialize() {
    for (char cid : kCanIds) {
        const std::string name = std::string("candle") + cid;
        auto iface = std::make_shared<CANInterface>(name.c_str());
        if (!iface || !iface->IsValid()) {
            LOG("ERROR", "MotorIO", "init", ("failed to open CAN interface: " + name).c_str());
            return false;
        }
        controller_->BindCAN(iface);
    }

    for (int j = 0; j < 4; ++j) {
        const std::string can_if = std::string("candle") + kCanIds[j];
        for (int i = 0; i < 3; ++i) {
            const int gid = j + i * 4;
            auto info = std::make_unique<RobstrideController::MotorInfo>();
            info->motor_id = kMotorIds[gid];
            info->host_id = 0xFD;
            info->max_torque = kMitTorqueLimitDefault;
            info->max_speed = kMitVelLimitDefault;
            info->max_kp = 500.0f;
            info->max_kd = 5.0f;
            const int idx = controller_->BindMotor(can_if.c_str(), std::move(info));
            if (idx < 0) {
                LOG("ERROR", "MotorIO", "init", ("failed to bind motor " + std::to_string(gid) + " on " + can_if).c_str());
                return false;
            }
            motor_indices_[gid] = idx;
        }
    }

    ConfigureMITParams();
    return true;
}

void MotorIO::ConfigureMITParams() {
    for (int idx : motor_indices_) {
        MIT_params params;
        params.kp = mit_config_.kp;
        params.kd = mit_config_.kd;
        params.vel_limit = mit_config_.vel_limit;
        params.torque_limit = mit_config_.torque_limit;
        controller_->SetMITParams(idx, params);
    }
}

bool MotorIO::EnableAll() {
    for (int gi = 0; gi < kNumMotors; ++gi) {
        const int idx = motor_indices_[gi];
        if (controller_->EnableMotor(idx) != 0) {
            std::string msg = std::string("enable failed for ") + kJointNames[gi]
                              + " motor_idx=" + std::to_string(idx);
            LOG("ERROR", "MotorIO", "enable", msg.c_str());
            last_fault_.has_fault = true;
            last_fault_.motor_index = gi;
            last_fault_.joint_name = kJointNames[gi];
            last_fault_.code = "enable_failed";
            last_fault_.message = msg;
            return false;
        }
    }
    ConfigureMITParams();
    return true;
}

void MotorIO::SetMITConfig(float kp, float kd, float vel_limit, float torque_limit) {
    mit_config_.kp = kp;
    mit_config_.kd = kd;
    mit_config_.vel_limit = vel_limit;
    mit_config_.torque_limit = torque_limit;
    ConfigureMITParams();
}

bool MotorIO::DisableAll() {
    for (int idx : motor_indices_) {
        controller_->DisableMotor(idx);
    }
    return true;
}

void MotorIO::EnableAllAutoReport() {
    for (int idx : motor_indices_) {
        controller_->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void MotorIO::EnableJoint(int joint_index) {
    if (joint_index < 0 || joint_index >= kNumMotors) return;
    const int idx = motor_indices_[joint_index];
    controller_->EnableMotor(idx);
    controller_->EnableAutoReport(idx);
    ConfigureMITParams();
}

std::vector<float> MotorIO::GetJointObs() const {
    std::vector<float> obs(kNumMotors * 2, 0.0f);
    for (int i = 0; i < kNumMotors; ++i) {
        const auto state = controller_->GetMotorState(motor_indices_[i]);
        float pos_rel = kJointDirection[i] * (state.position - kJointOffsets[i]);
        float vel_rel = kJointDirection[i] * state.velocity;
        if (i >= 8) {
            pos_rel /= kKneeRatio;
            vel_rel /= kKneeRatio;
        }
        obs[i] = pos_rel;
        obs[kNumMotors + i] = vel_rel;
    }
    return obs;
}

std::vector<float> MotorIO::ComputeAbsoluteTargetsFromRelative(const std::vector<float>& actions,
                                                               float action_scale) const {
    std::vector<float> targets(kNumMotors, 0.0f);
    for (int i = 0; i < kNumMotors; ++i) {
        float act = actions[i];
        if (i >= 8) {
            act *= kKneeRatio;
        }
        const float desired = kJointDirection[i] * act * action_scale + kJointOffsets[i];
        targets[i] = ClampAbsoluteTargetRad(i, desired);
    }
    return targets;
}

bool MotorIO::SendActions(const std::vector<float>& actions, float action_scale) {
    if (actions.size() != static_cast<size_t>(kNumMotors)) {
        return false;
    }
    const auto abs_targets = ComputeAbsoluteTargetsFromRelative(actions, action_scale);
    return SendAbsoluteTargets(abs_targets, nullptr);
}

bool MotorIO::SendJointRelativeTargets(const std::vector<float>& rel_targets) {
    return SendActions(rel_targets, 1.0f);
}

bool MotorIO::SendSingleMotorRelativeTarget(int motor_index, float rel_target) {
    if (motor_index < 0 || motor_index >= kNumMotors) {
        return false;
    }
    float act = rel_target;
    if (motor_index >= 8) {
        act *= kKneeRatio;
    }
    const float abs_target = ClampAbsoluteTargetRad(motor_index,
        kJointDirection[motor_index] * act + kJointOffsets[motor_index]);
    return controller_->SendMITCommand(motor_indices_[motor_index], abs_target) == 0;
}

void MotorIO::GetMotorStates(std::vector<float>& positions,
                             std::vector<float>& velocities,
                             std::vector<float>& torques) const {
    controller_->GetAllMotorStates(motor_indices_, positions, velocities, torques);
}

bool MotorIO::MoveToOffset(float duration_sec) {
    return SmoothMoveAbsolute(ReadCurrentPositions(),
                              std::vector<float>(kJointOffsets, kJointOffsets + kNumMotors),
                              duration_sec,
                              100,
                              []() { return false; },
                              nullptr);
}

std::vector<float> MotorIO::ReadCurrentPositions() const {
    std::vector<float> positions(kNumMotors, 0.0f);
    for (int i = 0; i < kNumMotors; ++i) {
        positions[i] = controller_->GetMotorState(motor_indices_[i]).position;
    }
    return positions;
}


float MotorIO::ClampAbsoluteTargetRad(int motor_index, float abs_target) const {
    if (motor_index < 0 || motor_index >= kNumMotors) {
        return abs_target;
    }
    const float lower = kJointOffsets[motor_index] + kXmlMin[motor_index];
    const float upper = kJointOffsets[motor_index] + kXmlMax[motor_index];
    return std::clamp(abs_target, lower, upper);
}

bool MotorIO::SmoothMoveAbsolute(const std::vector<float>& from_abs,
                                 const std::vector<float>& to_abs,
                                 float duration_sec,
                                 int hz,
                                 const std::function<bool()>& should_abort,
                                 std::string* err) {
    if (from_abs.size() != static_cast<size_t>(kNumMotors) || to_abs.size() != static_cast<size_t>(kNumMotors)) {
        if (err) *err = "SmoothMoveAbsolute size mismatch";
        return false;
    }
    const int steps = std::max(1, static_cast<int>(duration_sec * hz));
    const int period_ms = std::max(1, 1000 / std::max(1, hz));

    for (int s = 1; s <= steps; ++s) {
        if (should_abort && should_abort()) {
            if (err) *err = "motion aborted";
            return false;
        }
        const float a = static_cast<float>(s) / static_cast<float>(steps);
        const float h = a * a * (3.0f - 2.0f * a);
        std::vector<float> abs_targets(kNumMotors, 0.0f);
        for (int i = 0; i < kNumMotors; ++i) {
            abs_targets[i] = ClampAbsoluteTargetRad(i, (1.0f - h) * from_abs[i] + h * to_abs[i]);
        }
        if (!SendAbsoluteTargets(abs_targets, err)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
    return true;
}

bool MotorIO::SendAbsoluteTargets(const std::vector<float>& abs_targets, std::string* err) {
    if (abs_targets.size() != static_cast<size_t>(kNumMotors)) {
        if (err) *err = "expected 12 absolute targets";
        return false;
    }
    for (int i = 0; i < kNumMotors; ++i) {
        if (controller_->SendMITCommand(motor_indices_[i], ClampAbsoluteTargetRad(i, abs_targets[i])) != 0) {
            std::string msg = std::string("SendMITCommand failed for ") + kJointNames[i];
            LOG("ERROR", "MotorIO", "send", msg.c_str());
            if (err) *err = msg;
            last_fault_.has_fault = true;
            last_fault_.motor_index = i;
            last_fault_.joint_name = kJointNames[i];
            last_fault_.code = "send_failed";
            last_fault_.message = msg;
            return false;
        }
    }
    return true;
}

MotorFault MotorIO::CheckFeedbackFresh(int max_age_ms) {
    std::vector<MotorFault> faults;
    controller_->GetOfflineMotors(motor_indices_, max_age_ms, faults);
    if (!faults.empty()) {
        // Map controller-level motor_index back to joint index
        for (const auto& f : faults) {
            for (int gi = 0; gi < kNumMotors; ++gi) {
                if (motor_indices_[gi] == f.motor_index) {
                    last_fault_ = f;
                    last_fault_.motor_index = gi;
                    last_fault_.joint_name = kJointNames[gi];
                    return last_fault_;
                }
            }
        }
        last_fault_ = faults[0];
    }
    return last_fault_;
}

std::vector<std::string> MotorIO::GetOfflineJoints(int max_age_ms) const {
    std::vector<MotorFault> faults;
    controller_->GetOfflineMotors(motor_indices_, max_age_ms, faults);
    std::vector<std::string> names;
    for (const auto& f : faults) {
        for (int gi = 0; gi < kNumMotors; ++gi) {
            if (motor_indices_[gi] == f.motor_index) {
                names.push_back(kJointNames[gi]);
                break;
            }
        }
    }
    return names;
}

}  // namespace minimal

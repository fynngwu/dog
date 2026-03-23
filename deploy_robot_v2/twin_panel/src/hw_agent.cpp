#include "hw_cli/hw_agent.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace hw_cli {

HwAgent::HwAgent()
    : controller_(std::make_shared<RobstrideController>()),
      bringup_(std::make_unique<RobotBringup>(controller_)) {}

bool HwAgent::Connect(std::string& err) {
    if (!motor_indices_.empty()) return true;
    if (!bringup_->BindAllCAN()) {
        err = "BindAllCAN failed";
        return false;
    }
    if (!bringup_->BindAllMotors(motor_indices_)) {
        err = "BindAllMotors failed";
        return false;
    }
    return true;
}

bool HwAgent::EnsureEnabled(bool low_gain, bool auto_report, std::string& err) {
    if (!Connect(err)) return false;
    if (!bringup_->EnableAll(motor_indices_, low_gain)) {
        err = "EnableAll failed";
        return false;
    }
    if (auto_report) {
        if (!bringup_->EnableAutoReportAll(motor_indices_)) {
            err = "EnableAutoReportAll failed";
            return false;
        }
    }
    if (!bringup_->WaitForFeedback(motor_indices_, 2000)) {
        err = "WaitForFeedback failed";
        return false;
    }
    return true;
}

bool HwAgent::DisableAll(std::string& err) {
    if (!Connect(err)) return false;
    for (int idx : motor_indices_) {
        if (controller_->DisableMotor(idx) != 0) {
            err = "DisableMotor failed";
            return false;
        }
    }
    return true;
}

bool HwAgent::MoveToOffsets(float seconds, bool low_gain, bool auto_report, std::string& err) {
    if (!EnsureEnabled(low_gain, auto_report, err)) return false;
    if (!bringup_->MoveCurrentToOffsets(motor_indices_, seconds)) {
        err = "MoveCurrentToOffsets failed";
        return false;
    }
    return true;
}

bool HwAgent::HoldOffsets(bool low_gain, bool auto_report, int sample_delay_ms, int max_age_ms, std::string& err) {
    if (!EnsureEnabled(low_gain, auto_report, err)) return false;
    if (!bringup_->HoldOffsets(motor_indices_)) {
        err = "HoldOffsets failed";
        return false;
    }
    ResultPayload ignore;
    return SampleState(sample_delay_ms, max_age_ms, ignore, err);
}

JointArray HwAgent::JointRelFromRaw(const JointArray& raw_action) const {
    JointArray out{};
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = raw_action[i] * semantics_.action_scale;
    }
    return out;
}

JointArray HwAgent::MotorAbsFromJointRel(const JointArray& joint_rel) const {
    JointArray target{};
    for (size_t i = 0; i < target.size(); ++i) {
        float motor_delta = joint_rel[i];
        if (i >= 8) motor_delta *= semantics_.knee_ratio;
        // 应用关节方向符号：sign * action * scale + offset
        const float desired = semantics_.joint_direction[i] * motor_delta + semantics_.offsets[i];
        const float lower = semantics_.offsets[i] + semantics_.xml_min[i];
        const float upper = semantics_.offsets[i] + semantics_.xml_max[i];
        target[i] = std::clamp(desired, lower, upper);
    }
    return target;
}

std::vector<float> HwAgent::JointRelFromMotorAbs(const std::vector<float>& motor_abs) const {
    std::vector<float> rel(motor_abs.size(), 0.0f);
    for (size_t i = 0; i < motor_abs.size(); ++i) {
        // 反向处理符号：(motor_abs - offset) / sign
        const float delta = motor_abs[i] - semantics_.offsets[i];
        float joint_rel = delta * semantics_.joint_direction[i];  // 除以 sign 等于乘以 sign（因为 sign=±1）
        rel[i] = (i >= 8) ? (joint_rel / semantics_.knee_ratio) : joint_rel;
    }
    return rel;
}

bool HwAgent::SampleState(int sample_delay_ms, int max_age_ms, ResultPayload& out, std::string& err) {
    if (sample_delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_delay_ms));
    }
    
    // 检查电机在线数量
    out.motors_online_count = 0;
    for (int idx : motor_indices_) {
        if (controller_->IsMotorOnline(idx)) {
            out.motors_online_count++;
        }
    }
    
    // 检查反馈新鲜度
    out.sample_fresh = controller_->AllMotorsOnlineFresh(motor_indices_, max_age_ms);
    
    // 获取最大反馈年龄
    out.max_feedback_age_ms = 0;
    for (int idx : motor_indices_) {
        int age = controller_->GetLastOnlineAgeMs(idx);
        if (age > out.max_feedback_age_ms) {
            out.max_feedback_age_ms = age;
        }
    }
    
    // 读取状态
    out.motor_abs.assign(cfg::kNumMotors, 0.0f);
    out.motor_vel.assign(cfg::kNumMotors, 0.0f);
    out.motor_tau.assign(cfg::kNumMotors, 0.0f);
    controller_->GetAllMotorStates(motor_indices_, out.motor_abs, out.motor_vel, out.motor_tau);
    out.hw_joint_rel = JointRelFromMotorAbs(out.motor_abs);
    
    // 如果不新鲜，返回错误
    if (!out.sample_fresh) {
        err = "feedback not fresh, max_age=" + std::to_string(out.max_feedback_age_ms) + "ms";
        return false;
    }
    
    return true;
}

bool HwAgent::SendRawAction(const JointArray& raw_action,
                            bool ensure_enabled,
                            bool low_gain,
                            bool auto_report,
                            int sample_delay_ms,
                            int max_age_ms,
                            ResultPayload& out,
                            std::string& err) {
    if (!Connect(err)) return false;
    if (ensure_enabled) {
        if (!EnsureEnabled(low_gain, auto_report, err)) return false;
    }
    out.raw_action = raw_action;
    out.target_joint_rel = JointRelFromRaw(raw_action);
    out.target_motor_abs = MotorAbsFromJointRel(out.target_joint_rel);
    for (size_t i = 0; i < out.target_motor_abs.size(); ++i) {
        if (controller_->SendMITCommand(motor_indices_[i], out.target_motor_abs[i]) != 0) {
            err = "SendMITCommand failed";
            return false;
        }
    }
    return SampleState(sample_delay_ms, max_age_ms, out, err);
}

bool HwAgent::GetState(int sample_delay_ms, int max_age_ms, ResultPayload& out, std::string& err) {
    if (!Connect(err)) return false;
    // 纯查询，不开启 auto-report，不改变电机状态
    return SampleState(sample_delay_ms, max_age_ms, out, err);
}

bool HwAgent::EnableAutoReport(std::string& err) {
    if (!Connect(err)) return false;
    if (!bringup_->EnableAutoReportAll(motor_indices_)) {
        err = "EnableAutoReportAll failed";
        return false;
    }
    return true;
}

bool HwAgent::DisableAutoReport(std::string& err) {
    if (!Connect(err)) return false;
    // 发送停止自动上报命令
    for (int idx : motor_indices_) {
        if (controller_->DisableAutoReport(idx) != 0) {
            err = "DisableAutoReport failed for motor " + std::to_string(idx);
            return false;
        }
    }
    return true;
}

}  // namespace hw_cli

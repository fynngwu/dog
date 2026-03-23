#pragma once

#include <array>
#include <string>
#include <vector>

#include "robot_config.hpp"

namespace hw_cli {

using JointArray = std::array<float, cfg::kNumMotors>;

// 辅助函数：将 vector<float> 转换为 array<float, 12>
inline JointArray VecToArray(const std::vector<float>& v) {
    JointArray arr{};
    for (size_t i = 0; i < cfg::kNumMotors && i < v.size(); ++i) {
        arr[i] = v[i];
    }
    return arr;
}

struct CommandLineArgs {
    std::string verb;
    bool json = false;
    bool low_gain = false;
    bool auto_report = false;
    bool ensure_enabled = false;
    int sample_delay_ms = 15;
    float seconds = 2.0f;
    std::string action_csv;
};

struct Semantics {
    float action_scale = cfg::kActionScale;
    float knee_ratio = cfg::kKneeRatio;
    JointArray offsets = VecToArray(cfg::kJointOffsets);
    JointArray xml_min = VecToArray(cfg::kXmlMin);
    JointArray xml_max = VecToArray(cfg::kXmlMax);
    JointArray joint_direction = VecToArray(cfg::kJointDirection);  // HipA/HipF=-1, Knee=+1
};

struct ResultPayload {
    bool ok = false;
    std::string mode;
    int seq = 0;
    int rtt_ms = 0;
    std::string error;
    // 状态可信度字段
    bool sample_fresh = false;        // 采样是否新鲜
    int max_feedback_age_ms = -1;     // 最大反馈年龄 (ms)，-1 表示未知
    int motors_online_count = 0;      // 在线电机数量
    // 动作数据
    JointArray raw_action{};
    JointArray target_joint_rel{};
    JointArray target_motor_abs{};
    std::vector<float> motor_abs;
    std::vector<float> motor_vel;
    std::vector<float> motor_tau;
    std::vector<float> hw_joint_rel;
};

}  // namespace hw_cli

#pragma once
#include <vector>
#include <string>

namespace cfg {

// ============================================================================
// 电机数量和维度配置
// ============================================================================
constexpr int kNumMotors = 12;
constexpr int kHistoryLen = 10;
constexpr int kSingleObsDim = 45;      // IMU(6) + Command(3) + JointPos(12) + JointVel(12) + PrevAction(12)
constexpr int kPolicyInputDim = 450;   // 10 * 45
constexpr int kPolicyOutputDim = 12;
constexpr int kControlHz = 50;

// ============================================================================
// Policy 输出缩放
// ============================================================================
constexpr float kActionScale = 0.25f;
constexpr float kKneeRatio = 1.667f;

// ============================================================================
// MIT 模式参数
// ============================================================================
constexpr float kMitKp = 40.0f;
constexpr float kMitKd = 0.5f;
constexpr float kMitVelLimit = 44.0f;
constexpr float kMitTorqueLimit = 17.0f;

// ============================================================================
// CAN 接口配置
// ============================================================================
static const std::vector<char> kCanIds = {'0', '1', '2', '3'};

// ============================================================================
// 电机 ID 映射
// 布局: LF LR RF RR (每腿 HipA, HipF, Knee)
// ============================================================================
static const std::vector<int> kMotorIds = {
    1, 5, 9, 13,   // HipA: LF, LR, RF, RR
    2, 6, 10, 14,  // HipF: LF, LR, RF, RR
    3, 7, 11, 15,  // Knee: LF, LR, RF, RR
};

// ============================================================================
// 关节偏移量 (站立姿态)
// ============================================================================
constexpr float HIPA_OFFSET = 0.37f;
constexpr float HIPF_OFFSET = 0.13f;
constexpr float KNEE_OFFSET = 1.06f * kKneeRatio;

static const std::vector<float> kJointOffsets = {
    HIPA_OFFSET, -HIPA_OFFSET, -HIPA_OFFSET, HIPA_OFFSET,   // HipA
    HIPF_OFFSET, HIPF_OFFSET, -HIPF_OFFSET, -HIPF_OFFSET,   // HipF
    KNEE_OFFSET, KNEE_OFFSET, -KNEE_OFFSET, -KNEE_OFFSET,   // Knee
};

// ============================================================================
// 关节限位 (相对于 offset 的偏移量)
// ============================================================================
static const std::vector<float> kXmlMin = {
    -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,    // HipA
    -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,    // HipF
    -1.2217299f * kKneeRatio, -1.2217299f * kKneeRatio, -0.6f, -0.6f  // Knee
};

static const std::vector<float> kXmlMax = {
    0.7853982f, 0.7853982f, 0.7853982f, 0.7853982f,        // HipA
    0.8726683f, 0.8726683f, 1.2217342f, 1.2217305f,        // HipF
    0.6f, 0.6f, 1.2217287f * kKneeRatio, 1.2217287f * kKneeRatio  // Knee
};

// ============================================================================
// 设备路径
// ============================================================================
static const std::string kImuDevice = "/dev/ttyCH341USB0";
static const std::string kGamepadDevice = "/dev/input/js0";
static const std::string kPolicyEnginePath = "/home/ares/pure_cpp/policy.engine";

// ============================================================================
// 日志配置
// ============================================================================
static const std::string kLogDir = "/home/wufy/git_resp/dog/logs";

// ============================================================================
// ROS2 输入配置
// ============================================================================
inline constexpr const char* kRos2CmdTopic = "/cmd_vel";
inline constexpr float kRos2CmdVxScale = 0.5f;
inline constexpr float kRos2CmdVyScale = 0.5f;
inline constexpr float kRos2CmdYawScale = 0.5f;
inline constexpr int   kRos2CmdDeadmanMs = 200;

}  // namespace cfg
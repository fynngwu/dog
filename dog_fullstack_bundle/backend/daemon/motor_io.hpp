#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "log.hpp"
#include "robstride.hpp"

namespace minimal {

constexpr int kNumMotors = 12;
constexpr float kKneeRatio = 1.667f;
constexpr float kActionScale = 0.25f;
constexpr float kMitKpDefault = 40.0f;
constexpr float kMitKdDefault = 0.5f;
constexpr float kMitVelLimitDefault = 44.0f;
constexpr float kMitTorqueLimitDefault = 17.0f;

constexpr int kMotorIds[kNumMotors] = {
    1, 5, 9, 13,
    2, 6, 10, 14,
    3, 7, 11, 15,
};

constexpr float kJointDirection[kNumMotors] = {
    -1.0f, -1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f, -1.0f, -1.0f,
    +1.0f, +1.0f, +1.0f, +1.0f,
};

constexpr float kHipAOffset = 0.37f;
constexpr float kHipFOffset = 0.13f;
constexpr float kKneeOffset = 1.06f * kKneeRatio;

constexpr float kJointOffsets[kNumMotors] = {
    kHipAOffset, -kHipAOffset, -kHipAOffset, kHipAOffset,
    kHipFOffset, kHipFOffset, -kHipFOffset, -kHipFOffset,
    kKneeOffset, kKneeOffset, -kKneeOffset, -kKneeOffset,
};

constexpr float kXmlMin[kNumMotors] = {
    -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,
    -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,
    -1.2217299f * kKneeRatio, -1.2217299f * kKneeRatio, -0.6f, -0.6f,
};

constexpr float kXmlMax[kNumMotors] = {
    0.7853982f, 0.7853982f, 0.7853982f, 0.7853982f,
    0.8726683f, 0.8726683f, 1.2217342f, 1.2217305f,
    0.6f, 0.6f, 1.2217287f * kKneeRatio, 1.2217287f * kKneeRatio,
};

constexpr char kCanIds[] = {'0', '1', '2', '3'};

extern const char* const kJointNames[kNumMotors];

struct MITConfig {
    float kp = kMitKpDefault;
    float kd = kMitKdDefault;
    float vel_limit = kMitVelLimitDefault;
    float torque_limit = kMitTorqueLimitDefault;
};

class MotorIO {
public:
    MotorIO();
    ~MotorIO();

    bool Initialize();
    bool EnableAll();
    bool DisableAll();
    void EnableAllAutoReport();

    /** Enable a single motor by joint index and its auto-report. */
    void EnableJoint(int joint_index);
    void SetMITConfig(float kp, float kd, float vel_limit, float torque_limit);

    bool MoveToOffset(float duration_sec = 2.0f);

    std::vector<float> GetJointObs() const;
    bool SendActions(const std::vector<float>& actions, float action_scale = kActionScale);
    bool SendJointRelativeTargets(const std::vector<float>& rel_targets);
    bool SendSingleMotorRelativeTarget(int motor_index, float rel_target);

    void GetMotorStates(std::vector<float>& positions,
                        std::vector<float>& velocities,
                        std::vector<float>& torques) const;

    float ClampAbsoluteTargetRad(int motor_index, float abs_target) const;

    /** Check all motors for stale feedback, return first fault found. */
    MotorFault CheckFeedbackFresh(int max_age_ms);

    /** Return joint names of all motors that haven't reported feedback within max_age_ms. */
    std::vector<std::string> GetOfflineJoints(int max_age_ms) const;

    /** Return the most recent fault recorded by this layer. */
    const MotorFault& GetLastFault() const { return last_fault_; }

private:
    std::shared_ptr<RobstrideController> controller_;
    std::vector<int> motor_indices_;
    MITConfig mit_config_;
    MotorFault last_fault_;

    void ConfigureMITParams();
    std::vector<float> ReadCurrentPositions() const;
    std::vector<float> ComputeAbsoluteTargetsFromRelative(const std::vector<float>& actions,
                                                          float action_scale) const;
    bool SmoothMoveAbsolute(const std::vector<float>& from_abs,
                            const std::vector<float>& to_abs,
                            float duration_sec,
                            int hz,
                            const std::function<bool()>& should_abort,
                            std::string* err);
    bool SendAbsoluteTargets(const std::vector<float>& abs_targets, std::string* err = nullptr);
};

}  // namespace minimal

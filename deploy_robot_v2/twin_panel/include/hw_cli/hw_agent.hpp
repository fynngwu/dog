#pragma once

#include <memory>
#include <vector>

#include "hw_cli/types.hpp"
#include "robstride.hpp"
#include "robot_bringup.hpp"

namespace hw_cli {

class HwAgent {
public:
    HwAgent();

    bool Connect(std::string& err);
    bool EnsureEnabled(bool low_gain, bool auto_report, std::string& err);
    bool DisableAll(std::string& err);
    bool MoveToOffsets(float seconds, bool low_gain, bool auto_report, std::string& err);
    bool HoldOffsets(bool low_gain, bool auto_report, int sample_delay_ms, int max_age_ms, std::string& err);
    bool SendRawAction(const JointArray& raw_action,
                       bool ensure_enabled,
                       bool low_gain,
                       bool auto_report,
                       int sample_delay_ms,
                       int max_age_ms,
                       ResultPayload& out,
                       std::string& err);
    // 纯查询，不改变电机状态
    bool GetState(int sample_delay_ms, int max_age_ms, ResultPayload& out, std::string& err);
    // 单独控制 auto-report
    bool EnableAutoReport(std::string& err);
    bool DisableAutoReport(std::string& err);

private:
    bool SampleState(int sample_delay_ms, int max_age_ms, ResultPayload& out, std::string& err);
    JointArray JointRelFromRaw(const JointArray& raw_action) const;
    JointArray MotorAbsFromJointRel(const JointArray& joint_rel) const;
    std::vector<float> JointRelFromMotorAbs(const std::vector<float>& motor_abs) const;

    Semantics semantics_;
    std::shared_ptr<RobstrideController> controller_;
    std::unique_ptr<RobotBringup> bringup_;
    std::vector<int> motor_indices_;
};

}  // namespace hw_cli
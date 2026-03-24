#pragma once

#include <memory>
#include <vector>

#include "motor_math.hpp"
#include "robstride.hpp"
#include "robot_constants.hpp"

namespace minimal {

class MotorIO {
public:
    MotorIO();
    ~MotorIO();

    bool Initialize();
    bool EnableAndMoveToOffsets(float duration_sec = 3.0f);
    std::vector<float> GetJointObs() const;
    bool SendActions(const std::vector<float>& actions, float action_scale = kActionScale);
    bool AllMotorsHealthy(int max_age_ms = 100) const;
    void GetMotorStates(std::vector<float>& positions,
                        std::vector<float>& velocities,
                        std::vector<float>& torques) const;
    bool HoldOffsets();
    void CaptureHoldPose();
    bool HoldLatchedPose();
    void DebugPrintMotorAges() const;

private:
    std::shared_ptr<RobstrideController> controller_;
    std::vector<int> motor_indices_;
    std::vector<float> hold_targets_;
    bool hold_captured_ = false;

    std::vector<float> ReadCurrentPositions() const;
    bool SmoothMove(const std::vector<float>& from,
                    const std::vector<float>& to,
                    float duration_sec, int hz = 100);
};

}  // namespace minimal

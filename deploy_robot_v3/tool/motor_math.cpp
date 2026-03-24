#include "motor_math.hpp"

#include <algorithm>

namespace minimal {

std::vector<float> ComputeMotorTargets(const std::vector<float>& actions,
                                       float action_scale) {
    std::vector<float> targets(kNumMotors, 0.0f);
    if (actions.size() != static_cast<size_t>(kNumMotors)) {
        return targets;
    }

    for (int i = 0; i < kNumMotors; ++i) {
        float act = actions[i];
        if (i >= 8 && i <= 11) {
            act *= kKneeRatio;
        }

        const float desired = kJointDirection[i] * act * action_scale + kJointOffsets[i];
        const float lower = kJointOffsets[i] + kXmlMin[i];
        const float upper = kJointOffsets[i] + kXmlMax[i];
        targets[i] = std::clamp(desired, lower, upper);
    }

    return targets;
}

std::vector<float> MapMotorStateToJointObs(const std::vector<float>& positions,
                                          const std::vector<float>& velocities) {
    std::vector<float> obs(kNumMotors * 2, 0.0f);
    if (positions.size() != static_cast<size_t>(kNumMotors) ||
        velocities.size() != static_cast<size_t>(kNumMotors)) {
        return obs;
    }

    for (int i = 0; i < kNumMotors; ++i) {
        if (i >= 8 && i <= 11) {
            obs[i] = kJointDirection[i] * ((positions[i] - kJointOffsets[i]) / kKneeRatio);
            obs[kNumMotors + i] = kJointDirection[i] * (velocities[i] / kKneeRatio);
        } else {
            obs[i] = kJointDirection[i] * (positions[i] - kJointOffsets[i]);
            obs[kNumMotors + i] = kJointDirection[i] * velocities[i];
        }
    }

    return obs;
}

}  // namespace minimal

#pragma once

#include <vector>
#include "robot_constants.hpp"

namespace minimal {

std::vector<float> ComputeMotorTargets(const std::vector<float>& actions,
                                       float action_scale = kActionScale);

std::vector<float> MapMotorStateToJointObs(const std::vector<float>& positions,
                                          const std::vector<float>& velocities);

}  // namespace minimal

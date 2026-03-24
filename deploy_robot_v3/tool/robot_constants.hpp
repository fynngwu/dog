#pragma once

namespace minimal {

constexpr int kNumMotors = 12;
constexpr float kKneeRatio = 1.667f;
constexpr float kActionScale = 0.25f;
constexpr float kMitKp = 40.0f;
constexpr float kMitKd = 0.5f;
constexpr float kMitVelLimit = 44.0f;
constexpr float kMitTorqueLimit = 17.0f;

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

}  // namespace minimal

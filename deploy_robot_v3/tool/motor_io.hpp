#pragma once

#include <vector>
#include <memory>
#include "robstride.hpp"

namespace minimal {

// Configuration constants
constexpr int kNumMotors = 12;
constexpr float kKneeRatio = 1.667f;
constexpr float kActionScale = 0.25f;
constexpr float kMitKp = 40.0f;
constexpr float kMitKd = 0.5f;
constexpr float kMitVelLimit = 44.0f;
constexpr float kMitTorqueLimit = 17.0f;

// Motor IDs (LF LR RF RR, each leg: HipA, HipF, Knee)
constexpr int kMotorIds[kNumMotors] = {
    1, 5, 9, 13,   // HipA: LF, LR, RF, RR
    2, 6, 10, 14,  // HipF: LF, LR, RF, RR
    3, 7, 11, 15,  // Knee: LF, LR, RF, RR
};

// Joint direction mapping (HipA/HipF=-1, Knee=+1)
constexpr float kJointDirection[kNumMotors] = {
    -1.0f, -1.0f, -1.0f, -1.0f,   // HipA
    -1.0f, -1.0f, -1.0f, -1.0f,   // HipF
    +1.0f, +1.0f, +1.0f, +1.0f,   // Knee
};

// Joint offsets (standing pose)
constexpr float kHipAOffset = 0.37f;
constexpr float kHipFOffset = 0.13f;
constexpr float kKneeOffset = 1.06f * kKneeRatio;  // ≈1.767

constexpr float kJointOffsets[kNumMotors] = {
    kHipAOffset, -kHipAOffset, -kHipAOffset, kHipAOffset,   // HipA
    kHipFOffset, kHipFOffset, -kHipFOffset, -kHipFOffset,   // HipF
    kKneeOffset, kKneeOffset, -kKneeOffset, -kKneeOffset,   // Knee
};

// Joint limits (relative to offset)
constexpr float kXmlMin[kNumMotors] = {
    -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,    // HipA
    -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,    // HipF
    -1.2217299f * kKneeRatio, -1.2217299f * kKneeRatio, -0.6f, -0.6f  // Knee
};

constexpr float kXmlMax[kNumMotors] = {
    0.7853982f, 0.7853982f, 0.7853982f, 0.7853982f,        // HipA
    0.8726683f, 0.8726683f, 1.2217342f, 1.2217305f,        // HipF
    0.6f, 0.6f, 1.2217287f * kKneeRatio, 1.2217287f * kKneeRatio  // Knee
};

// CAN interface names
constexpr char kCanIds[] = {'0', '1', '2', '3'};

/**
 * @brief Motor I/O layer for robot control
 *
 * Handles CAN + Robstride integration, joint direction mapping,
 * zero-point positioning, and command sending.
 */
class MotorIO {
public:
    MotorIO();
    ~MotorIO();

    /**
     * @brief Initialize CAN interfaces and bind motors
     * @return true on success
     */
    bool Initialize();

    /**
     * @brief Enable motors and smoothly move to offset positions
     *
     * The robot moves to kJointOffsets which is the ready-to-stand position.
     *
     * @param duration_sec Duration of the smooth move (default 3.0s)
     * @return true on success
     */
    bool EnableAndMoveToOffsets(float duration_sec = 3.0f);

    /**
     * @brief Get joint observations (positions and velocities)
     * @return Vector of [12 positions, 12 velocities] in policy coordinate system
     */
    std::vector<float> GetJointObs() const;

    /**
     * @brief Send actions to motors
     * @param actions Policy output (12 values)
     * @param action_scale Action scaling factor (default uses kActionScale)
     * @return true on success
     */
    bool SendActions(const std::vector<float>& actions, float action_scale = kActionScale);

    /**
     * @brief Check if all motors are healthy and providing fresh feedback
     * @param max_age_ms Maximum age of feedback in ms (default 100ms)
     * @return true if all motors are online and fresh
     */
    bool AllMotorsHealthy(int max_age_ms = 100) const;

    /**
     * @brief Get current motor states for diagnostics
     */
    void GetMotorStates(std::vector<float>& positions,
                       std::vector<float>& velocities,
                       std::vector<float>& torques) const;

    /**
     * @brief Hold all motors at their offset positions
     */
    bool HoldOffsets();

    /**
     * @brief Capture current positions as hold targets (call once)
     *
     * Reads current motor positions and stores them internally.
     * Subsequent calls to HoldLatchedPose() will send these cached targets.
     * This prevents chasing noisy measurements in hold mode.
     */
    void CaptureHoldPose();

    /**
     * @brief Hold all motors at the latched positions (call repeatedly)
     *
     * Sends the positions captured by CaptureHoldPose().
     * Returns false if no pose has been captured.
     */
    bool HoldLatchedPose();

    /**
     * @brief Debug: print motor feedback ages
     */
    void DebugPrintMotorAges() const;

private:
    std::shared_ptr<RobstrideController> controller_;
    std::vector<int> motor_indices_;  // Mapping from logical index to controller index
    std::vector<float> hold_targets_;  // Cached hold positions
    bool hold_captured_ = false;       // Whether hold_targets_ is valid

    /**
     * @brief Compute target positions from policy actions
     * @param actions Policy output (12 values)
     * @param action_scale Action scaling factor
     * @return Vector of 12 absolute motor positions
     */
    std::vector<float> ComputeTargetPositions(
        const std::vector<float>& actions, float action_scale) const;

    /**
     * @brief Read current absolute positions from all motors
     */
    std::vector<float> ReadCurrentPositions() const;

    /**
     * @brief Smoothly interpolate from current to target positions
     */
    bool SmoothMove(const std::vector<float>& from,
                   const std::vector<float>& to,
                   float duration_sec, int hz = 100);
};

}  // namespace minimal

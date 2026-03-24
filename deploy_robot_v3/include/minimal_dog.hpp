#pragma once

#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <array>

#include "motor_io.hpp"
#include "policy.hpp"
#include "ros2_command_source.hpp"

namespace minimal {

/**
 * @brief Minimal quadruped robot controller
 *
 * Core control pipeline:
 * 1. Initialize (IMU thread, motor CAN, move to zero pose)
 * 2. Prewarm (collect 10 frames of valid observations)
 * 3. Run loop @ 50Hz:
 *    - Build observation (IMU, gravity projection, command, joints, prev action)
 *    - Update history (10 frames)
 *    - Run policy inference
 *    - Send motor commands
 */
class MinimalDog {
public:
    static constexpr int kObsDim = 45;       // Single frame observation dimension
    static constexpr int kHistoryLen = 10;   // Number of history frames
    static constexpr int kPolicyInputDim = 450;  // kObsDim * kHistoryLen
    static constexpr int kControlHz = 50;    // Control loop frequency
    static constexpr float kActionRateLimit = 0.05f;  // Max delta per tick for action smoothing

    MinimalDog();
    ~MinimalDog();

    /**
     * @brief Initialize all subsystems
     * @return true on success
     */
    bool Initialize();

    /**
     * @brief Run the main control loop
     * @param duration_sec Duration to run (0 = infinite)
     */
    void Run(float duration_sec = 0.0f);

    /**
     * @brief Stop the robot (emergency hold)
     */
    void Stop();

    // ========================================================================
    // Public: Static callbacks for IMU C SDK
    // ========================================================================
    static MinimalDog* imu_instance_;  // For C callback
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t ucMs);
    static void SerialWriteRegister(uint8_t* p_ucData, uint32_t uiLen);

private:
    // ========================================================================
    // IMU
    // ========================================================================

    struct IMUData {
        float gyro[3];   // [X, Y, Z] deg/s (raw from IMU)
        float quat[4];   // [w, x, y, z]
        std::atomic<bool> valid;  // Atomic to prevent data race
        std::chrono::steady_clock::time_point last_update;
        int imu_fd_;     // File descriptor from auto-scan
    };

    bool InitIMU();
    void IMUUpdateLoop();

    /**
     * @brief Compute gravity projection from quaternion
     * @param quat Quaternion [w, x, y, z]
     * @return [gx, gy, gz] normalized gravity vector
     */
    std::vector<float> ComputeGravityProjection(const float quat[4]) const;

    /**
     * @brief Get mapped IMU observation (6 dims: gyro + gravity)
     *
     * Mapping (same as v2):
     * - Gyro: Y(forward), -X(left), Z(up) in rad/s
     * - Gravity: -gy(forward), gx(left), -gz(up)
     */
    std::vector<float> GetIMUObs() const;

    bool IsIMUFresh(int timeout_ms = 100) const;

    // ========================================================================
    // Command input
    // ========================================================================

    /**
     * @brief Get command (3 dims: vx, vy, yaw_rate)
     *
     * Returns command from ROS2 /cmd_vel topic with deadman timeout.
     * Falls back to zero command if ROS2 is not available.
     */
    std::array<float, 3> GetCommand() const;

    // ========================================================================
    // Observation Building
    // ========================================================================

    /**
     * @brief Build single observation frame (45 dims)
     *
     * Layout:
     *  [0:3]   - Angular velocity (rad/s)
     *  [3:6]   - Projected gravity (normalized)
     *  [6:9]   - Command (vx, vy, yaw_rate)
     *  [9:21]  - Joint positions (12)
     *  [21:33] - Joint velocities (12)
     *  [33:45] - Previous actions (12)
     */
    std::vector<float> BuildSingleObs();

    /**
     * @brief Flatten history to 450-dim policy input
     */
    std::vector<float> FlattenHistory() const;

    // ========================================================================
    // Prewarm
    // ========================================================================

    /**
     * @brief Collect valid frames to fill history
     *
     * Only counts frames where IMU and motors are fresh.
     * Command is forced to zero during prewarm.
     */
    bool Prewarm();

    // ========================================================================
    // Member variables
    // ========================================================================

    // Motor and Policy
    std::unique_ptr<MotorIO> motor_io_;
    std::unique_ptr<Policy> policy_;

    // IMU
    std::thread imu_thread_;
    std::atomic<bool> imu_running_;
    mutable std::mutex imu_mutex_;
    IMUData imu_data_;

    // Observation history
    std::deque<std::vector<float>> obs_history_;
    std::vector<float> prev_actions_;

    // Command source (ROS2 cmd_vel)
    std::unique_ptr<ROS2CommandSource> cmd_source_;

    // Running state
    std::atomic<bool> running_;
    std::atomic<bool> motors_enabled_;  // Track if motors were enabled

    // Policy failure tracking
    int policy_fail_count_ = 0;
};

}  // namespace minimal

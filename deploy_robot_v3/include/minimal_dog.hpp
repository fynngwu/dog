#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <vector>

#include "imu_reader.hpp"
#include "motor_io.hpp"
#include "obs_builder.hpp"
#include "policy.hpp"
#include "ros2_command_source.hpp"

namespace minimal {

class MinimalDog {
public:
    static constexpr int kControlHz = 50;
    static constexpr float kActionRateLimit = 0.05f;

    MinimalDog();
    ~MinimalDog();

    bool Initialize();
    void Run(float duration_sec = 0.0f);
    void Stop();

private:
    bool Prewarm();
    std::array<float, 3> GetCommand() const;
    bool SensorsHealthy() const;
    void HoldOffsetsAndResetActions();

    std::unique_ptr<IMUReader> imu_reader_;
    std::unique_ptr<MotorIO> motor_io_;
    std::unique_ptr<Policy> policy_;
    std::unique_ptr<ROS2CommandSource> cmd_source_;
    ObsBuilder obs_builder_;

    std::atomic<bool> running_;
    std::atomic<bool> motors_enabled_;
    std::vector<float> prev_actions_;
    int policy_fail_count_ = 0;
};

}  // namespace minimal

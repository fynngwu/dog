#include "minimal_dog.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

extern std::atomic<bool> g_running;

namespace minimal {

MinimalDog::MinimalDog()
    : running_(false),
      motors_enabled_(false) {
    prev_actions_.assign(12, 0.0f);
}

MinimalDog::~MinimalDog() {
    Stop();
}

bool MinimalDog::Initialize() {
    std::cout << "[MinimalDog] Initializing..." << std::endl;

    imu_reader_ = std::make_unique<IMUReader>();
    if (!imu_reader_->Initialize()) {
        std::cerr << "[MinimalDog] IMU initialization failed" << std::endl;
        return false;
    }

    motor_io_ = std::make_unique<MotorIO>();
    if (!motor_io_->Initialize()) {
        std::cerr << "[MinimalDog] MotorIO initialization failed" << std::endl;
        return false;
    }

    policy_ = std::make_unique<Policy>(POLICY_ENGINE_PATH);
    if (!policy_->IsValid()) {
        std::cerr << "[MinimalDog] Policy initialization failed" << std::endl;
        return false;
    }

    cmd_source_ = std::make_unique<ROS2CommandSource>();
    if (!cmd_source_->Initialize()) {
        std::cerr << "[MinimalDog] ROS2 command source initialization failed" << std::endl;
        return false;
    }

    if (!motor_io_->EnableAndMoveToOffsets(3.0f)) {
        std::cerr << "[MinimalDog] Enable and move to offsets failed" << std::endl;
        return false;
    }

    obs_builder_.Reset();
    running_ = true;
    motors_enabled_ = true;
    std::cout << "[MinimalDog] Initialization complete" << std::endl;
    return true;
}

std::array<float, 3> MinimalDog::GetCommand() const {
    if (!cmd_source_) {
        return {0.0f, 0.0f, 0.0f};
    }
    return cmd_source_->GetCommand();
}

bool MinimalDog::SensorsHealthy() const {
    return imu_reader_ && motor_io_ &&
           imu_reader_->IsFresh(100) &&
           motor_io_->AllMotorsHealthy(100);
}

void MinimalDog::HoldOffsetsAndResetActions() {
    prev_actions_.assign(12, 0.0f);
    if (motor_io_) {
        motor_io_->HoldOffsets();
    }
}

bool MinimalDog::Prewarm() {
    std::cout << "[MinimalDog] Prewarming..." << std::endl;
    obs_builder_.ClearHistory();

    int valid_frames = 0;
    int total_attempts = 0;
    const int max_attempts = ObsBuilder::kHistoryLen * 10;

    while (g_running && running_ && valid_frames < ObsBuilder::kHistoryLen && total_attempts < max_attempts) {
        if (SensorsHealthy()) {
            const auto single_obs = obs_builder_.BuildSingle(
                imu_reader_->GetObservation(),
                {0.0f, 0.0f, 0.0f},
                motor_io_->GetJointObs());
            obs_builder_.Push(single_obs);
            valid_frames++;
            std::cout << "\r  Valid frame " << valid_frames << "/" << ObsBuilder::kHistoryLen << std::flush;
        } else {
            obs_builder_.ClearHistory();
            valid_frames = 0;
            if (total_attempts % 10 == 0) {
                std::cout << "\r  Waiting for valid sensors... (attempt " << total_attempts << ")" << std::flush;
            }
        }

        HoldOffsetsAndResetActions();
        total_attempts++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kControlHz));
    }

    if (valid_frames < ObsBuilder::kHistoryLen) {
        std::cerr << "\n[MinimalDog] Prewarm failed" << std::endl;
        return false;
    }

    std::cout << "\n[MinimalDog] Prewarm complete" << std::endl;
    return true;
}

void MinimalDog::Run(float duration_sec) {
    if (!Prewarm()) {
        return;
    }

    std::cout << "[MinimalDog] Starting main loop..." << std::endl;
    const auto t0 = std::chrono::steady_clock::now();
    uint64_t tick = 0;

    while (running_ && g_running) {
        const auto loop_start = std::chrono::steady_clock::now();

        if (duration_sec > 0.0f) {
            const auto elapsed = std::chrono::duration<double>(loop_start - t0).count();
            if (elapsed >= duration_sec) {
                break;
            }
        }

        if (!SensorsHealthy()) {
            if (tick % 50 == 0) {
                std::cerr << "\n[MinimalDog] Sensor stale, holding offsets" << std::endl;
            }
            HoldOffsetsAndResetActions();
            std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(1000 / kControlHz));
            continue;
        }

        const auto single_obs = obs_builder_.BuildSingle(
            imu_reader_->GetObservation(),
            GetCommand(),
            motor_io_->GetJointObs());
        obs_builder_.Push(single_obs);

        PolicyStatus policy_status = PolicyStatus::kOk;
        auto actions = policy_->Run(obs_builder_.FlattenHistory(), &policy_status);
        if (policy_status != PolicyStatus::kOk) {
            policy_fail_count_++;
            std::cerr << "\n[MinimalDog] Policy failure count=" << policy_fail_count_ << std::endl;
            HoldOffsetsAndResetActions();
            if (policy_fail_count_ >= 5) {
                running_ = false;
            }
            std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(1000 / kControlHz));
            continue;
        }
        policy_fail_count_ = 0;

        for (int i = 0; i < 12; ++i) {
            const float delta = std::clamp(actions[i] - prev_actions_[i], -kActionRateLimit, kActionRateLimit);
            actions[i] = prev_actions_[i] + delta;
        }

        if (!motor_io_->SendActions(actions)) {
            std::cerr << "\n[MinimalDog] SendActions failed, holding offsets" << std::endl;
            HoldOffsetsAndResetActions();
            running_ = false;
            break;
        }

        prev_actions_ = actions;

        if (tick % 10 == 0) {
            const auto imu_obs = imu_reader_->GetObservation();
            const auto cmd = GetCommand();
            const auto t_sec = std::chrono::duration<double>(loop_start - t0).count();
            std::cout << "\r[t=" << std::fixed << std::setprecision(1) << t_sec
                      << "s] cmd:[" << std::setprecision(2) << cmd[0] << "," << cmd[1] << "," << cmd[2]
                      << "] gyro:[" << imu_obs[0] << "," << imu_obs[1] << "," << imu_obs[2]
                      << "] grav:[" << imu_obs[3] << "," << imu_obs[4] << "," << imu_obs[5] << "]"
                      << std::flush;
        }

        tick++;
        std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(1000 / kControlHz));
    }

    std::cout << "\n[MinimalDog] Main loop ended" << std::endl;
}

void MinimalDog::Stop() {
    running_ = false;

    if (cmd_source_) {
        cmd_source_->Stop();
    }
    if (imu_reader_) {
        imu_reader_->Stop();
    }
    if (motor_io_ && motors_enabled_) {
        for (int i = 0; i < 5; ++i) {
            motor_io_->HoldOffsets();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::cout << "[MinimalDog] Stopped" << std::endl;
}

}  // namespace minimal

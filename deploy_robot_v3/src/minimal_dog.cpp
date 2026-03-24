#include "minimal_dog.hpp"
#include "minimal_dog_cbinds.h"  // C linkage declarations
#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <algorithm>
#include <cerrno>

// External signal flag from main.cpp
extern std::atomic<bool> g_running;

// IMU C SDK headers (from v2) - include AFTER C++ headers, without extern "C"
#include "serial.h"
#include "wit_c_sdk.h"

namespace minimal {

// Static member initialization
MinimalDog* MinimalDog::imu_instance_ = nullptr;
static volatile char s_cDataUpdate = 0;

// ============================================================================
// Constructor/Destructor
// ============================================================================

MinimalDog::MinimalDog()
    : imu_running_(false),
      running_(false) {

    std::memset(imu_data_.gyro, 0, sizeof(imu_data_.gyro));
    std::memset(imu_data_.quat, 0, sizeof(imu_data_.quat));
    imu_data_.valid.store(false);  // Atomic store
    imu_data_.imu_fd_ = -1;  // Initialize to invalid

    prev_actions_.resize(12, 0.0f);
}

MinimalDog::~MinimalDog() {
    Stop();
}

// ============================================================================
// Initialization
// ============================================================================

bool MinimalDog::Initialize() {
    std::cout << "[MinimalDog] Initializing..." << std::endl;

    // Initialize IMU first
    if (!InitIMU()) {
        std::cerr << "[MinimalDog] IMU initialization failed" << std::endl;
        return false;
    }

    // Initialize MotorIO
    motor_io_ = std::make_unique<MotorIO>();
    if (!motor_io_->Initialize()) {
        std::cerr << "[MinimalDog] MotorIO initialization failed" << std::endl;
        return false;
    }

    // Initialize Policy before enabling motors
    const std::string kPolicyEnginePath = POLICY_ENGINE_PATH;
    policy_ = std::make_unique<Policy>(kPolicyEnginePath);
    if (!policy_->IsValid()) {
        std::cerr << "[MinimalDog] Policy initialization failed" << std::endl;
        return false;
    }

    // Initialize ROS2 command source before enabling motors
    cmd_source_ = std::make_unique<ROS2CommandSource>();
    if (!cmd_source_->Initialize()) {
        std::cerr << "[MinimalDog] ROS2 command source initialization failed" << std::endl;
        return false;
    }

    // Now enable motors and move to offset position
    // This is done last so all control components are ready
    if (!motor_io_->EnableAndMoveToOffsets(3.0f)) {
        std::cerr << "[MinimalDog] Enable and move to offsets failed" << std::endl;
        return false;
    }

    motors_enabled_ = true;
    running_ = true;
    std::cout << "[MinimalDog] Initialization complete" << std::endl;
    return true;
}

// ============================================================================
// IMU
// ============================================================================

bool MinimalDog::InitIMU() {
    imu_instance_ = this;

    const char* kImuDevice = "/dev/ttyCH341USB0";

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitDelayMsRegister(minimal_dog_delay_ms);
    WitSerialWriteRegister(minimal_dog_serial_write);
    WitRegisterCallBack(minimal_dog_sensor_update);

    // Auto-scan baud rate
    int fd = -1;
    int s_iCurBaud = 9600;
    constexpr int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

    for (size_t i = 0; i < sizeof(c_uiBaud)/sizeof(int); i++) {
        if (fd >= 0) serial_close(fd);

        s_iCurBaud = c_uiBaud[i];
        fd = serial_open((unsigned char*)kImuDevice, s_iCurBaud);

        if (fd < 0) continue;

        int iRetry = 2;
        do {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            minimal_dog_delay_ms(200);

            unsigned char cBuff[1];
            while (serial_read_data(fd, cBuff, 1)) {
                WitSerialDataIn(cBuff[0]);
            }

            if (s_cDataUpdate != 0) {
                std::cout << "[MinimalDog] IMU connected at baud " << s_iCurBaud << std::endl;
                imu_data_.imu_fd_ = fd;  // Store fd for later use
                break;
            }
            iRetry--;
        } while (iRetry);

        if (s_cDataUpdate != 0) break;
    }

    if (s_cDataUpdate == 0) {
        std::cerr << "[MinimalDog] Cannot find IMU sensor" << std::endl;
        return false;
    }

    // Configure IMU to output gyro and quaternion
    if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_Q) != WIT_HAL_OK) {
        std::cerr << "[MinimalDog] Configure IMU failed" << std::endl;
        return false;
    }

    // Start update thread
    imu_running_ = true;
    imu_thread_ = std::thread(&MinimalDog::IMUUpdateLoop, this);

    // Wait for first valid data
    auto t0 = std::chrono::steady_clock::now();
    while (!imu_data_.valid.load()) {  // Atomic load
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (dt > 5000) {
            std::cerr << "[MinimalDog] IMU data timeout" << std::endl;
            return false;
        }
    }

    std::cout << "[MinimalDog] IMU initialized" << std::endl;
    return true;
}

void MinimalDog::IMUUpdateLoop() {
    // Stop() will close the fd to unblock this thread
    std::cout << "[MinimalDog] IMU update thread started, fd=" << imu_data_.imu_fd_ << std::endl;
    int read_count = 0;

    while (imu_running_ && imu_data_.imu_fd_ >= 0) {
        unsigned char cBuff[1];

        // CRITICAL: Use while loop to drain all available data, same as v2
        // If we only read 1 byte and sleep, the buffer will fill up and stop receiving
        while (serial_read_data(imu_data_.imu_fd_, cBuff, 1) > 0) {
            WitSerialDataIn(cBuff[0]);
            read_count++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200Hz
    }
    std::cout << "[MinimalDog] IMU update thread exiting, imu_running=" << imu_running_
              << ", fd=" << imu_data_.imu_fd_ << std::endl;
}

// Static callback for IMU data (called from wit_c_sdk)
void MinimalDog::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    s_cDataUpdate = 1;  // Signal successful data reception for auto-scan
    if (!imu_instance_) return;

    std::lock_guard<std::mutex> lock(imu_instance_->imu_mutex_);

    for (uint32_t i = 0; i < uiRegNum; i++) {
        switch (uiReg) {
            case AX: case AY: case AZ:
                // Accelerometer (not used)
                break;
            case GX: case GY: case GZ:
                for (int j = 0; j < 3; j++) {
                    imu_instance_->imu_data_.gyro[j] = sReg[GX + j] / 32768.0f * 2000.0f;
                }
                break;
            case Roll: case Pitch: case Yaw:
                // Angles (not used, we use quaternion)
                break;
            case 0x51: case 0x52: case 0x53: case 0x54:  // q0, q1, q2, q3
                for (int j = 0; j < 4; j++) {
                    imu_instance_->imu_data_.quat[j] = sReg[0x51 + j] / 32768.0f;
                }
                imu_instance_->imu_data_.valid.store(true);  // Atomic store
                imu_instance_->imu_data_.last_update = std::chrono::steady_clock::now();
                break;
        }
        uiReg++;
    }
}

// Static callback functions for IMU
void MinimalDog::Delayms(uint16_t ucMs) {
    usleep(ucMs * 1000);
}

void MinimalDog::SerialWriteRegister(uint8_t* p_ucData, uint32_t uiLen) {
    if (!imu_instance_) return;
    if (imu_instance_->imu_data_.imu_fd_ < 0) return;

    serial_write_data(imu_instance_->imu_data_.imu_fd_, p_ucData, uiLen);
}

}  // namespace minimal

// C linkage wrappers for wit_c_sdk
extern "C" {
    void minimal_dog_sensor_update(uint32_t uiReg, uint32_t uiRegNum) {
        if (minimal::MinimalDog::imu_instance_) {
            minimal::MinimalDog::imu_instance_->SensorDataUpdata(uiReg, uiRegNum);
        }
    }

    void minimal_dog_delay_ms(uint16_t ucMs) {
        minimal::MinimalDog::Delayms(ucMs);
    }

    void minimal_dog_serial_write(uint8_t* p_ucData, uint32_t uiLen) {
        minimal::MinimalDog::SerialWriteRegister(p_ucData, uiLen);
    }
}

namespace minimal {

std::vector<float> MinimalDog::ComputeGravityProjection(const float quat[4]) const {
    float qw = quat[0];  // w component
    float qx = quat[1];  // x component
    float qy = quat[2];  // y component
    float qz = quat[3];  // z component

    // Gravity vector in IMU frame
    float gx = 2 * (qx * qz - qw * qy);
    float gy = 2 * (qy * qz + qw * qx);
    float gz = 1 - 2 * (qx * qx + qy * qy);

    // Normalize
    float norm = std::sqrt(gx * gx + gy * gy + gz * gz);
    if (norm > 1e-6f) {
        gx /= norm;
        gy /= norm;
        gz /= norm;
    }

    // Map to robot frame: forward(-gy), left(gx), up(-gz)
    return {-gy, gx, -gz};
}

std::vector<float> MinimalDog::GetIMUObs() const {
    // Copy data under lock to avoid deadlock
    float gyro[3], quat[4];
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        std::memcpy(gyro, imu_data_.gyro, sizeof(gyro));
        std::memcpy(quat, imu_data_.quat, sizeof(quat));
    }

    // Gyro mapping: Y(forward), -X(left), Z(up) in rad/s
    std::vector<float> obs;
    obs.push_back(gyro[1] * M_PI / 180.0f);   // forward
    obs.push_back(-gyro[0] * M_PI / 180.0f);  // left
    obs.push_back(gyro[2] * M_PI / 180.0f);   // up

    // Gravity projection (no lock needed - using copied quat)
    auto grav = ComputeGravityProjection(quat);
    obs.insert(obs.end(), grav.begin(), grav.end());

    return obs;
}

bool MinimalDog::IsIMUFresh(int timeout_ms) const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!imu_data_.valid.load()) return false;  // Atomic load

    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - imu_data_.last_update).count();
    return dt <= timeout_ms;
}

// ============================================================================
// Command input
// ============================================================================

std::array<float, 3> MinimalDog::GetCommand() const {
    if (cmd_source_) {
        return cmd_source_->GetCommand();
    }
    return {0.0f, 0.0f, 0.0f};
}

// ============================================================================
// Observation Building
// ============================================================================

std::vector<float> MinimalDog::BuildSingleObs() {
    std::vector<float> obs;
    obs.reserve(kObsDim);

    // [0:3] Angular velocity (rad/s)
    auto imu_obs = GetIMUObs();
    obs.insert(obs.end(), imu_obs.begin(), imu_obs.begin() + 3);

    // [3:6] Projected gravity
    obs.insert(obs.end(), imu_obs.begin() + 3, imu_obs.end());

    // [6:9] Command
    auto cmd = GetCommand();
    obs.insert(obs.end(), cmd.begin(), cmd.end());  // std::array has begin()/end()

    // [9:21] Joint positions, [21:33] Joint velocities
    auto joint_obs = motor_io_->GetJointObs();
    obs.insert(obs.end(), joint_obs.begin(), joint_obs.end());

    // [33:45] Previous actions
    obs.insert(obs.end(), prev_actions_.begin(), prev_actions_.end());

    return obs;
}

std::vector<float> MinimalDog::FlattenHistory() const {
    std::vector<float> flat;
    flat.reserve(kPolicyInputDim);

    // Pad with zeros if history is not full
    int missing = kHistoryLen - obs_history_.size();
    for (int i = 0; i < missing; i++) {
        flat.insert(flat.end(), kObsDim, 0.0f);
    }

    // Add actual history
    for (const auto& frame : obs_history_) {
        flat.insert(flat.end(), frame.begin(), frame.end());
    }

    return flat;
}

// ============================================================================
// Prewarm
// ============================================================================

bool MinimalDog::Prewarm() {
    std::cout << "[MinimalDog] Prewarming: collecting " << kHistoryLen << " valid frames..." << std::endl;

    int valid_frames = 0;
    int total_attempts = 0;
    const int max_attempts = kHistoryLen * 10;

    while (valid_frames < kHistoryLen && total_attempts < max_attempts && running_ && g_running) {
        auto single_obs = BuildSingleObs();

        // Check if sensors are fresh
        if (IsIMUFresh(100) && motor_io_->AllMotorsHealthy(100)) {
            obs_history_.push_back(single_obs);
            if (obs_history_.size() > static_cast<size_t>(kHistoryLen)) {
                obs_history_.pop_front();
            }
            valid_frames++;
            std::cout << "\r  Valid frame " << valid_frames << "/" << kHistoryLen << std::flush;
        } else {
            // Invalid frame: just skip, don't reset
            // This makes prewarm more tolerant to transient sensor failures
            if (total_attempts % 10 == 0) {
                std::cout << "\r  Waiting for valid sensors... (attempt " << total_attempts << ")" << std::flush;
            }
        }

        // Hold at offset positions during prewarm for stability
        if (!motor_io_->HoldOffsets()) {
            std::cerr << "\n[MinimalDog] HoldOffsets failed during prewarm" << std::endl;
            return false;
        }

        total_attempts++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kControlHz));
    }

    if (valid_frames < kHistoryLen) {
        std::cerr << "\n[MinimalDog] Prewarm failed: only got " << valid_frames << " frames" << std::endl;
        return false;
    }

    std::cout << "\n[MinimalDog] Prewarm complete" << std::endl;
    return true;
}

// ============================================================================
// Run
// ============================================================================

void MinimalDog::Run(float duration_sec) {
    if (!Prewarm()) {
        std::cerr << "[MinimalDog] Prewarm failed, exiting" << std::endl;
        return;
    }

    std::cout << "[MinimalDog] Starting main loop..." << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    uint64_t tick = 0;

    while (running_ && g_running) {
        auto loop_start = std::chrono::steady_clock::now();

        // Cache health check results
        bool imu_fresh = IsIMUFresh(100);
        bool motors_healthy = motor_io_->AllMotorsHealthy(100);

        // Check duration
        if (duration_sec > 0) {
            auto elapsed = std::chrono::duration<double>(loop_start - t0).count();
            if (elapsed >= duration_sec) {
                std::cout << "[MinimalDog] Run duration reached" << std::endl;
                break;
            }
        }

        // Safety check: sensors must be fresh
        if (!imu_fresh || !motors_healthy) {
            // Only print every 50 ticks (1 second) to avoid spamming
            if (tick % 50 == 0) {
                auto now = std::chrono::steady_clock::now();
                std::lock_guard<std::mutex> lock(imu_mutex_);
                auto imu_age = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - imu_data_.last_update).count();
                std::cerr << "\n[MinimalDog] Sensor not fresh:"
                          << " imu=" << (imu_fresh ? "OK" : "STALE")
                          << " (age=" << imu_age << "ms)"
                          << ", motors=" << (motors_healthy ? "OK" : "STALE")
                          << std::endl;
            }

            motor_io_->HoldOffsets();
            auto next_time = loop_start + std::chrono::milliseconds(1000 / kControlHz);
            std::this_thread::sleep_until(next_time);
            continue;
        }

        // Build observation
        auto single_obs = BuildSingleObs();

        // Update history
        obs_history_.push_back(single_obs);
        if (obs_history_.size() > static_cast<size_t>(kHistoryLen)) {
            obs_history_.pop_front();
        }

        // Flatten to 450-dim
        auto obs_450 = FlattenHistory();

        // Run policy with status check
        PolicyStatus policy_status;
        auto actions = policy_->Run(obs_450, &policy_status);

        // Handle policy failure: hold current pose instead of sending zero actions
        if (policy_status != PolicyStatus::kOk) {
            policy_fail_count_++;
            std::cerr << "\n[MinimalDog] Policy inference failed (status="
                      << static_cast<int>(policy_status) << ", count=" << policy_fail_count_ << ")"
                      << std::endl;

            // Hold current pose on policy failure
            motor_io_->CaptureHoldPose();
            if (!motor_io_->HoldLatchedPose()) {
                std::cerr << "[MinimalDog] HoldLatchedPose failed" << std::endl;
            }

            if (policy_fail_count_ >= 5) {
                std::cerr << "[MinimalDog] Too many policy failures, stopping..." << std::endl;
                running_ = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kControlHz));
            continue;
        } else {
            policy_fail_count_ = 0;  // Reset on success
        }

        // Apply rate limit to actions for smoother motion
        for (int i = 0; i < 12; ++i) {
            float da = actions[i] - prev_actions_[i];
            da = std::clamp(da, -kActionRateLimit, kActionRateLimit);
            actions[i] = prev_actions_[i] + da;
        }

        // Send actions
        if (!motor_io_->SendActions(actions)) {
            std::cerr << "\n[MinimalDog] SendActions failed, stopping..." << std::endl;
            running_ = false;
            break;
        }

        // Store previous actions
        prev_actions_ = actions;

        // Print status every 10 ticks
        if (tick % 10 == 0) {
            auto imu_obs = GetIMUObs();
            auto cmd = GetCommand();  // Now returns std::array<float, 3>
            auto t_sec = std::chrono::duration<double>(loop_start - t0).count();

            std::cout << "\r[t=" << std::fixed << std::setprecision(1) << t_sec
                      << "s] cmd: [" << std::setprecision(2) << cmd[0] << "," << cmd[1] << "," << cmd[2] << "]"
                      << " | gyro: [" << imu_obs[0] << "," << imu_obs[1] << "," << imu_obs[2] << "]"
                      << " | grav: [" << imu_obs[3] << "," << imu_obs[4] << "," << imu_obs[5] << "]"
                      << " | imu: " << (imu_fresh ? "OK" : "!!")
                      << " | motors: " << (motors_healthy ? "OK" : "!!")
                      << std::flush;
        }

        tick++;

        // Sleep until next control cycle
        auto next_time = loop_start + std::chrono::milliseconds(1000 / kControlHz);
        std::this_thread::sleep_until(next_time);
    }

    std::cout << "\n[MinimalDog] Main loop ended" << std::endl;
}

void MinimalDog::Stop() {
    // Step 1: Stop main loop and command source first
    running_ = false;

    // Stop ROS2 command source (stops spin thread and subscription)
    if (cmd_source_) {
        cmd_source_->Stop();
    }

    // Step 2: Stop IMU thread
    // Close fd FIRST to unblock any blocking read in the thread
    imu_running_ = false;
    int fd_to_close = imu_data_.imu_fd_;
    imu_data_.imu_fd_ = -1;  // Signal thread to stop using fd
    if (fd_to_close >= 0) {
        serial_close(fd_to_close);  // This will unblock any blocking read
    }
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }

    // Step 3: Send final hold commands only if motors were enabled
    // This avoids sending commands on failed initialization path
    if (motor_io_ && motors_enabled_) {
        motor_io_->CaptureHoldPose();
        for (int i = 0; i < 5; i++) {
            motor_io_->HoldLatchedPose();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::cout << "[MinimalDog] Stopped" << std::endl;
}

}  // namespace minimal

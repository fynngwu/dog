#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "can_interface.hpp"

/** Latest decoded motor telemetry. */
struct motor_state {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
};

/** MIT controller parameters stored per motor. */
struct MIT_params {
    float kp = 25.0f;
    float kd = 0.5f;
    float vel_limit = 44.0f;
    float torque_limit = 17.0f;
};

/** Describes a single motor fault for diagnostics. */
struct MotorFault {
    bool has_fault = false;
    int motor_index = -1;
    int motor_id = -1;
    std::string joint_name;
    std::string code;         // "no_feedback", "send_failed", "enable_failed"
    std::string message;
    int feedback_age_ms = -1;
};

/** Last commanded MIT tuple, useful for telemetry and debugging. */
struct motor_command {
    float position = 0.0f;
    float velocity = 0.0f;
    float kp = 25.0f;
    float kd = 0.5f;
    float torque = 0.0f;
};

/**
 * @brief Robstride CAN controller compatible with the reference implementation.
 *
 * The class binds multiple SocketCAN interfaces, tracks the last received state
 * for every motor, and serializes outgoing MIT frames using the same 29-bit CAN
 * ID layout as the reference daemon.
 */
class RobstrideController {
public:
    /** Static configuration attached to one motor instance. */
    struct MotorInfo {
        int motor_id = 0;
        int host_id = 0xFD;
        float max_torque = 17.0f;
        float max_speed = 44.0f;
        float max_kp = 500.0f;
        float max_kd = 5.0f;
    };

    RobstrideController();
    ~RobstrideController();

    int BindCAN(std::shared_ptr<CANInterface> can_interface);
    int BindMotor(const char* can_if, std::unique_ptr<struct MotorInfo> motor_info);

    struct motor_state GetMotorState(int motor_idx);
    struct motor_command GetLastCommand(int motor_idx);
    void GetAllMotorStates(const std::vector<int>& motor_indices,
                           std::vector<float>& positions,
                           std::vector<float>& velocities,
                           std::vector<float>& torques);
    bool IsMotorOnline(int motor_idx);
    int64_t GetLastOnlineAgeMs(int motor_idx);
    bool AllMotorsOnlineFresh(const std::vector<int>& motor_indices, int max_age_ms = 100);

    /** Return static motor info (motor_id, max limits) for a controller-level motor index. */
    struct MotorInfo GetMotorInfo(int motor_idx);

    /** Check a list of controller-level motor indices for offline/stale motors. */
    void GetOfflineMotors(const std::vector<int>& indices, int threshold_ms,
                          std::vector<MotorFault>& faults);

    int SetMITParams(int motor_idx, struct MIT_params mit_params);
    MIT_params GetMITParams(int motor_idx);

    int SendMITCommand(int motor_idx, float pos);
    int SendMITCommand(int motor_idx, float pos, float vel, float kp, float kd, float torque);

    int EnableMotor(int motor_idx);
    int DisableMotor(int motor_idx);
    int EnableAutoReport(int motor_idx);
    int DisableAutoReport(int motor_idx);
    int SetZero(int motor_idx);

    void HandleCANMessage(const struct device* dev, struct can_frame* frame);

private:
    /** Runtime state associated with one bound motor. */
    struct MotorData {
        struct MotorInfo motor_info;
        std::shared_ptr<CANInterface> can_iface;
        int motor_id = 0;
        int host_id = 0xFD;
        bool enabled = false;
        bool online = false;
        std::chrono::steady_clock::time_point last_online_time;
        struct MIT_params mit_params;
        struct motor_state state;
        struct motor_command last_command;
        int offline_count = 0;
    };

    float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);

    std::vector<std::shared_ptr<CANInterface>> can_interfaces_;
    std::vector<struct MotorData> motor_data_;
    CANInterface::can_rx_callback_t can_rx_callback_ = nullptr;

    std::thread control_thread_;
    std::atomic<bool> running_{false};
    std::recursive_mutex motor_data_mutex_;
};

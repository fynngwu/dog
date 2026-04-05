#include "robstride.hpp"

#include <climits>
#include <cmath>
#include <cstring>

namespace {
constexpr uint8_t kCommMIT = 0x01;
constexpr uint8_t kCommFeedback = 0x02;
constexpr uint8_t kCommEnable = 0x03;
constexpr uint8_t kCommStop = 0x04;
constexpr uint8_t kCommSetZero = 0x06;
constexpr uint8_t kCommReport = 0x18;

constexpr float kPMax = 12.57f;
constexpr float kVMax = 44.0f;
constexpr float kTMax = 17.0f;
constexpr float kKpMin = 0.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMin = 0.0f;
constexpr float kKdMax = 5.0f;

static inline uint32_t MakeSendCanID(uint8_t motor_id, uint8_t host_id, uint8_t msg_type) {
    uint32_t id = 0;
    id |= (motor_id & 0xFFU);
    id |= ((host_id & 0xFFU) << 8U);
    id |= (static_cast<uint32_t>(msg_type) << 24U);
    return id;
}

static inline uint8_t ParseRxMotorID(uint32_t can_id) {
    return static_cast<uint8_t>((can_id >> 8U) & 0xFFU);
}

static inline uint8_t ParseRxMsgType(uint32_t can_id) {
    return static_cast<uint8_t>((can_id >> 24U) & 0x1FU);
}

static void robstride_can_rx_callback_wrapper(const struct device* dev,
                                              struct can_frame* frame,
                                              void* user_data) {
    auto* controller = static_cast<RobstrideController*>(user_data);
    if (controller != nullptr) {
        controller->HandleCANMessage(dev, frame);
    }
}
}  // namespace

float RobstrideController::uint16_to_float(uint16_t x, float x_min, float x_max, int bits) {
    const float span = x_max - x_min;
    if (bits <= 0) {
        return 0.0f;
    }
    return (static_cast<float>(x) * span / static_cast<float>((1 << bits) - 1)) + x_min;
}

int RobstrideController::float_to_uint(float x, float x_min, float x_max, int bits) {
    const float clamped = std::max(x_min, std::min(x_max, x));
    if (bits <= 0) {
        return 0;
    }
    return static_cast<int>((clamped - x_min) * static_cast<float>((1 << bits) - 1) / (x_max - x_min));
}

void RobstrideController::HandleCANMessage(const struct device* dev, struct can_frame* frame) {
    (void)dev;
    if (frame == nullptr) {
        return;
    }

    uint32_t id = frame->can_id;
    if ((frame->can_id & CAN_EFF_FLAG) != 0) {
        id = frame->can_id & CAN_EFF_MASK;
    }

    const uint8_t motor_id = ParseRxMotorID(id);
    const uint8_t msg_type = ParseRxMsgType(id);
    if (msg_type != kCommFeedback && msg_type != kCommReport) {
        return;
    }

    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    for (auto& motor : motor_data_) {
        if (motor.motor_id != motor_id) {
            continue;
        }
        motor.online = true;
        motor.last_online_time = std::chrono::steady_clock::now();
        if (frame->can_dlc >= 6) {
            const uint16_t raw_pos = static_cast<uint16_t>((frame->data[0] << 8) | frame->data[1]);
            const uint16_t raw_vel = static_cast<uint16_t>((frame->data[2] << 8) | frame->data[3]);
            const uint16_t raw_tor = static_cast<uint16_t>((frame->data[4] << 8) | frame->data[5]);
            const float v_max = motor.motor_info.max_speed > 0.0f ? motor.motor_info.max_speed : kVMax;
            const float t_max = motor.motor_info.max_torque > 0.0f ? motor.motor_info.max_torque : kTMax;
            motor.state.position = uint16_to_float(raw_pos, -kPMax, kPMax, 16);
            motor.state.velocity = uint16_to_float(raw_vel, -v_max, v_max, 16);
            motor.state.torque = uint16_to_float(raw_tor, -t_max, t_max, 16);
        }
        break;
    }
}

RobstrideController::RobstrideController() {
    can_rx_callback_ = robstride_can_rx_callback_wrapper;
    running_ = true;
    control_thread_ = std::thread([this]() {
        while (running_) {
            const auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
                for (size_t i = 0; i < motor_data_.size(); ++i) {
                    auto& motor = motor_data_[i];
                    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - motor.last_online_time).count();
                    if (age_ms > 500) {
                        motor.online = false;
                    }
                    if (!motor.online && motor.enabled && motor.offline_count++ >= 10) {
                        motor.offline_count = 0;
                        EnableMotor(static_cast<int>(i));
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
}

RobstrideController::~RobstrideController() {
    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

int RobstrideController::BindCAN(std::shared_ptr<CANInterface> can_interface) {
    if (!can_interface) {
        return -1;
    }
    can_interfaces_.push_back(std::move(can_interface));
    return 0;
}

int RobstrideController::BindMotor(const char* can_if, std::unique_ptr<struct MotorInfo> motor_info) {
    if (can_if == nullptr || !motor_info) {
        return -1;
    }

    std::shared_ptr<CANInterface> target_iface;
    for (const auto& iface : can_interfaces_) {
        if (iface && std::string(iface->GetName()) == std::string(can_if)) {
            target_iface = iface;
            break;
        }
    }
    if (!target_iface) {
        return -1;
    }

    MotorData data;
    data.motor_info = *motor_info;
    data.can_iface = target_iface;
    data.motor_id = motor_info->motor_id;
    data.host_id = motor_info->host_id;
    data.enabled = false;
    data.online = false;
    data.last_online_time = std::chrono::steady_clock::now();
    data.mit_params = {25.0f, 0.5f, motor_info->max_speed, motor_info->max_torque};
    data.state = {0.0f, 0.0f, 0.0f};
    data.last_command = {0.0f, 0.0f, data.mit_params.kp, data.mit_params.kd, 0.0f};

    CANInterface::can_filter filter;
    uint32_t filter_id = 0;
    filter_id |= static_cast<uint32_t>(motor_info->host_id & 0xFF);
    filter_id |= static_cast<uint32_t>((motor_info->motor_id & 0xFF) << 8);
    filter.id = filter_id;
    filter.mask = 0x0000FFFFU;
    filter.flags = CAN_FILTER_IDE;
    target_iface->SetFilter(filter, can_rx_callback_, this);

    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    motor_data_.push_back(data);
    return static_cast<int>(motor_data_.size()) - 1;
}

struct motor_state RobstrideController::GetMotorState(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx >= 0 && static_cast<size_t>(motor_idx) < motor_data_.size()) {
        return motor_data_[motor_idx].state;
    }
    return {};
}

struct motor_command RobstrideController::GetLastCommand(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx >= 0 && static_cast<size_t>(motor_idx) < motor_data_.size()) {
        return motor_data_[motor_idx].last_command;
    }
    return {};
}

void RobstrideController::GetAllMotorStates(const std::vector<int>& motor_indices,
                                            std::vector<float>& positions,
                                            std::vector<float>& velocities,
                                            std::vector<float>& torques) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    positions.resize(motor_indices.size(), 0.0f);
    velocities.resize(motor_indices.size(), 0.0f);
    torques.resize(motor_indices.size(), 0.0f);
    for (size_t i = 0; i < motor_indices.size(); ++i) {
        const int idx = motor_indices[i];
        if (idx < 0 || static_cast<size_t>(idx) >= motor_data_.size()) {
            continue;
        }
        positions[i] = motor_data_[idx].state.position;
        velocities[i] = motor_data_[idx].state.velocity;
        torques[i] = motor_data_[idx].state.torque;
    }
}

bool RobstrideController::IsMotorOnline(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    return motor_idx >= 0 && static_cast<size_t>(motor_idx) < motor_data_.size() && motor_data_[motor_idx].online;
}

int64_t RobstrideController::GetLastOnlineAgeMs(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return INT64_MAX;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - motor_data_[motor_idx].last_online_time).count();
}

bool RobstrideController::AllMotorsOnlineFresh(const std::vector<int>& motor_indices, int max_age_ms) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    for (const int idx : motor_indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= motor_data_.size()) {
            return false;
        }
        const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - motor_data_[idx].last_online_time).count();
        if (!motor_data_[idx].online || age_ms > max_age_ms) {
            return false;
        }
    }
    return true;
}

struct RobstrideController::MotorInfo RobstrideController::GetMotorInfo(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return {};
    }
    return motor_data_[motor_idx].motor_info;
}

void RobstrideController::GetOfflineMotors(const std::vector<int>& indices, int threshold_ms,
                                           std::vector<MotorFault>& faults) {
    faults.clear();
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    const auto now = std::chrono::steady_clock::now();
    for (const int idx : indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= motor_data_.size()) {
            continue;
        }
        auto& motor = motor_data_[idx];
        const auto age_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - motor.last_online_time).count());
        if (!motor.online || age_ms > threshold_ms) {
            MotorFault f;
            f.has_fault = true;
            f.motor_index = idx;
            f.motor_id = motor.motor_id;
            f.code = "no_feedback";
            f.feedback_age_ms = age_ms;
            f.message = "motor_id=" + std::to_string(motor.motor_id) + " offline age=" + std::to_string(age_ms) + "ms";
            faults.push_back(std::move(f));
        }
    }
}

int RobstrideController::SetMITParams(int motor_idx, struct MIT_params mit_params) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return -1;
    }
    motor_data_[motor_idx].mit_params = mit_params;
    return 0;
}

MIT_params RobstrideController::GetMITParams(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx >= 0 && static_cast<size_t>(motor_idx) < motor_data_.size()) {
        return motor_data_[motor_idx].mit_params;
    }
    return {};
}

int RobstrideController::SendMITCommand(int motor_idx, float pos) {
    MIT_params params = GetMITParams(motor_idx);
    return SendMITCommand(motor_idx, pos, 0.0f, params.kp, params.kd, 0.0f);
}

int RobstrideController::SendMITCommand(int motor_idx, float pos, float vel, float kp, float kd, float torque) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return -1;
    }

    auto& motor = motor_data_[motor_idx];
    if (!motor.can_iface) {
        return -1;
    }

    const float v_max = motor.motor_info.max_speed > 0.0f ? motor.motor_info.max_speed : kVMax;
    const float t_max = motor.motor_info.max_torque > 0.0f ? motor.motor_info.max_torque : kTMax;

    struct can_frame frame{};
    const uint16_t tor_uint = static_cast<uint16_t>(float_to_uint(torque, -t_max, t_max, 16));

    uint32_t id = 0;
    id |= static_cast<uint32_t>(motor.motor_id & 0xFF);
    id |= static_cast<uint32_t>((tor_uint & 0xFF) << 8);
    id |= static_cast<uint32_t>(((tor_uint >> 8) & 0xFF) << 16);
    id |= static_cast<uint32_t>(kCommMIT) << 24;
    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = 8;

    const uint16_t pos_uint = static_cast<uint16_t>(float_to_uint(pos, -kPMax, kPMax, 16));
    const uint16_t vel_uint = static_cast<uint16_t>(float_to_uint(vel, -v_max, v_max, 16));
    const uint16_t kp_uint = static_cast<uint16_t>(float_to_uint(kp, kKpMin, kKpMax, 16));
    const uint16_t kd_uint = static_cast<uint16_t>(float_to_uint(kd, kKdMin, kKdMax, 16));

    frame.data[0] = static_cast<uint8_t>((pos_uint >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>(pos_uint & 0xFF);
    frame.data[2] = static_cast<uint8_t>((vel_uint >> 8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(vel_uint & 0xFF);
    frame.data[4] = static_cast<uint8_t>((kp_uint >> 8) & 0xFF);
    frame.data[5] = static_cast<uint8_t>(kp_uint & 0xFF);
    frame.data[6] = static_cast<uint8_t>((kd_uint >> 8) & 0xFF);
    frame.data[7] = static_cast<uint8_t>(kd_uint & 0xFF);

    motor.last_command = {pos, vel, kp, kd, torque};
    return motor.can_iface->SendMessage(&frame);
}

int RobstrideController::EnableMotor(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return -1;
    }
    auto& motor = motor_data_[motor_idx];
    motor.enabled = true;

    struct can_frame frame{};
    frame.can_id = MakeSendCanID(static_cast<uint8_t>(motor.motor_id),
                                 static_cast<uint8_t>(motor.host_id),
                                 kCommEnable) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    return motor.can_iface ? motor.can_iface->SendMessage(&frame) : -1;
}

int RobstrideController::DisableMotor(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return -1;
    }
    auto& motor = motor_data_[motor_idx];
    motor.enabled = false;

    struct can_frame frame{};
    frame.can_id = MakeSendCanID(static_cast<uint8_t>(motor.motor_id),
                                 static_cast<uint8_t>(motor.host_id),
                                 kCommStop) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    return motor.can_iface ? motor.can_iface->SendMessage(&frame) : -1;
}

int RobstrideController::EnableAutoReport(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return -1;
    }
    auto& motor = motor_data_[motor_idx];

    struct can_frame frame{};
    frame.can_id = MakeSendCanID(static_cast<uint8_t>(motor.motor_id),
                                 static_cast<uint8_t>(motor.host_id),
                                 kCommReport) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;
    frame.data[1] = 0x02;
    frame.data[2] = 0x03;
    frame.data[3] = 0x04;
    frame.data[4] = 0x05;
    frame.data[5] = 0x06;
    frame.data[6] = 0x01;
    frame.data[7] = 0x00;
    return motor.can_iface ? motor.can_iface->SendMessage(&frame) : -1;
}

int RobstrideController::DisableAutoReport(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return -1;
    }
    auto& motor = motor_data_[motor_idx];

    struct can_frame frame{};
    frame.can_id = MakeSendCanID(static_cast<uint8_t>(motor.motor_id),
                                 static_cast<uint8_t>(motor.host_id),
                                 kCommReport) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;
    frame.data[1] = 0x02;
    frame.data[2] = 0x03;
    frame.data[3] = 0x04;
    frame.data[4] = 0x05;
    frame.data[5] = 0x06;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return motor.can_iface ? motor.can_iface->SendMessage(&frame) : -1;
}

int RobstrideController::SetZero(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex_);
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= motor_data_.size()) {
        return -1;
    }
    auto& motor = motor_data_[motor_idx];

    struct can_frame frame{};
    frame.can_id = MakeSendCanID(static_cast<uint8_t>(motor.motor_id),
                                 static_cast<uint8_t>(motor.host_id),
                                 kCommSetZero) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;
    return motor.can_iface ? motor.can_iface->SendMessage(&frame) : -1;
}

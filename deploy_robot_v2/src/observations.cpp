#include "observations.hpp"
#include "input/input_source.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <cmath>
#include <fcntl.h>

#include "serial.h"
#include "wit_c_sdk.h"

/*
    Observations:
    Angular Velocity: 3
    Projected Gravity: 3
    Commands: 3
    Joint Positions: 12
    Joint Velocities: 12
    Previous Actions: 12

    Total: 45
*/

IMUComponent* IMUComponent::instance_ = nullptr;
int IMUComponent::fd = -1;
int IMUComponent::s_iCurBaud = 9600;

static volatile char s_cDataUpdate = 0;

IMUComponent::IMUComponent(const char* dev) : dev_path(dev), running_(true), data_valid_(false) {
    instance_ = this;

    std::memset(acc, 0, sizeof(acc));
    std::memset(gyro, 0, sizeof(gyro));
    std::memset(angle, 0, sizeof(angle));
    std::memset(quaternion, 0, sizeof(quaternion));
    last_update_time_ = std::chrono::steady_clock::now();

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitDelayMsRegister(Delayms);
    WitSerialWriteRegister(SerialWriteRegister);
    WitRegisterCallBack(SensorDataUpdata);

    AutoScanSensor();

    if (ConfigureSensorOutputs() != 0) {
        std::cerr << "Configure IMU failed" << std::endl;
    }

    update_thread_ = std::thread(&IMUComponent::UpdateLoop, this);
}

IMUComponent::~IMUComponent() {
    running_ = false;
    if (update_thread_.joinable()) {
        update_thread_.join();
    }

    if (fd >= 0) {
        serial_close(fd);
    }
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

void IMUComponent::UpdateLoop() {
    while (running_) {
        Update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200Hz, 足够 50Hz 控制
    }
}

void IMUComponent::Update() {
    if (fd < 0) return;
    
    char cBuff[1];
    while(serial_read_data(fd, (unsigned char*)cBuff, 1)) {
        WitSerialDataIn(cBuff[0]);
    }
}

std::vector<float> IMUComponent::GetObs() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<float> obs;
    obs.reserve(7);

    // 如果以 IMU 的 Y 轴为前向、-X 轴为左向、Z 轴为上向
    obs.push_back(gyro[1] * M_PI / 180.0f);   // 传给 policy 的前向: IMU_Y
    obs.push_back(-gyro[0] * M_PI / 180.0f);  // 传给 policy 的左向: -IMU_X
    obs.push_back(gyro[2] * M_PI / 180.0f);   // 传给 policy 的上向: IMU_Z

    float quat0 = quaternion[0];
    float quat1 = quaternion[1];
    float quat2 = quaternion[2];
    float quat3 = quaternion[3];

    float gx = 2 * (quat1 * quat3 - quat0 * quat2); // IMU X轴投影重力 (物理右向)
    float gy = 2 * (quat2 * quat3 + quat0 * quat1); // IMU Y轴投影重力 (物理前向)
    float gz = 1 - 2 * (quat1 * quat1 + quat2 * quat2);

    float norm = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (norm > 1e-6f) {
        gx /= norm;
        gy /= norm;
        gz /= norm;
    }
    
    // 同样，把前、左、上的重力投影传给 Policy
    // 原来是: -gx (前), -gy(左), -gz(上)
    obs.push_back(-gy);  // 新的前向是 Y    -> 传入 -gy
    obs.push_back(gx);   // 新的左向是 -X   -> 传入 -(-gx) = gx
    obs.push_back(-gz);  // Z 轴不变        -> 传入 -gz
    
    return obs;
}

bool IMUComponent::IsReady() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return data_valid_;
}

bool IMUComponent::IsFresh(int timeout_ms) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!data_valid_) return false;
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_update_time_);
    return duration.count() <= timeout_ms;
}

void IMUComponent::PrintDebug() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::cout << "[IMU Debug] raw gyro (deg/s): [" 
              << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << "]"
              << " | mapped (rad/s): ["
              << gyro[1] * M_PI / 180.0f << ", "  // Y -> forward
              << -gyro[0] * M_PI / 180.0f << ", " // -X -> left
              << gyro[2] * M_PI / 180.0f << "]"   // Z -> up
              << std::endl;
    
    float gx = 2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
    float gy = 2 * (quaternion[2] * quaternion[3] + quaternion[0] * quaternion[1]);
    float gz = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
    float norm = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (norm > 1e-6f) { gx /= norm; gy /= norm; gz /= norm; }
    
    std::cout << "[IMU Debug] raw grav: [" << gx << ", " << gy << ", " << gz << "]"
              << " | mapped: [" << -gy << ", " << gx << ", " << -gz << "]"
              << " | valid: " << (data_valid_ ? "Y" : "N")
              << std::endl;
}

void IMUComponent::AutoScanSensor() {
    int i, iRetry;
    char cBuff[1];
    
    for(i = 0; i < sizeof(c_uiBaud)/sizeof(int); i++) {
        if(fd >= 0) serial_close(fd);
        
        s_iCurBaud = c_uiBaud[i];
        fd = serial_open((unsigned char*)dev_path, s_iCurBaud);
        
        if(fd < 0) continue;

        iRetry = 2;
        do {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            Delayms(200);
            
            while(serial_read_data(fd, (unsigned char*)cBuff, 1)) {
                WitSerialDataIn(cBuff[0]);
            }
            
            if(s_cDataUpdate != 0) {
                std::cout << "IMU Connected at baud " << s_iCurBaud << std::endl;
                return;
            }
            iRetry--;
        } while(iRetry);
    }
    std::cerr << "Can not find IMU sensor" << std::endl;
}

int IMUComponent::ConfigureSensorOutputs() {
    // 设置回传内容：加速度、角速度、四元数
    int32_t ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_Q);
    if(ret != WIT_HAL_OK) {
        return -1;
    }
    return 0;
}

void IMUComponent::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    s_cDataUpdate = 1; // 标记收到数据，用于 AutoScan

    if (!instance_) return;

    std::lock_guard<std::mutex> lock(instance_->data_mutex_);

    for(int i = 0; i < uiRegNum; i++) {
        switch(uiReg) {
            case AX: case AY: case AZ:
                for(int j=0; j<3; j++) instance_->acc[j] = sReg[AX+j] / 32768.0f * 16.0f;
                break;
            case GX: case GY: case GZ:
                for(int j=0; j<3; j++) instance_->gyro[j] = sReg[GX+j] / 32768.0f * 2000.0f;
                break;
            case Roll: case Pitch: case Yaw:
                for(int j=0; j<3; j++) instance_->angle[j] = sReg[Roll+j] / 32768.0f * 180.0f;
                break;
            case q0: case q1: case q2: case q3:
                for(int j=0; j<4; j++) instance_->quaternion[j] = sReg[q0+j] / 32768.0f;
                // 收到四元数数据，标记为有效并更新时间
                instance_->data_valid_ = true;
                instance_->last_update_time_ = std::chrono::steady_clock::now();
                break;
        }
        uiReg++;
    }
}

void IMUComponent::Delayms(uint16_t ucMs) {
    usleep(ucMs * 1000);
}

void IMUComponent::SerialWriteRegister(uint8_t *p_ucData, uint32_t uiLen) {
    if (fd >= 0) {
        serial_write_data(fd, p_ucData, uiLen);
    }
}

std::vector<float> JointComponent::GetObs() const {
    std::vector<float> obs(joint_count * 2);
    if (motor_indices.size() != (size_t)joint_count) {
        std::cerr << "JointComponent: controllers size mismatch" << std::endl;
        return obs;
    }
    int idx = 0;
    for (int motor_idx : motor_indices) {
        auto state = controller->GetMotorState(motor_idx);

        // obs[idx] = state.position;


        // For knee joints (indices 8-11) the motor has a gear reduction.
        // Convert motor shaft angle to joint angle by dividing by the gear ratio.
        float pos = state.position;
        float vel = state.velocity;
         // Subtract joint offsets to get observations in relative coordinates
        if (idx >= 8 && idx <= 11) {
            obs[idx] = (pos - offsets[idx]) / 1.667f;
            obs[joint_count + idx] = vel / 1.667f;

        }
        else{
            obs[idx] = pos - offsets[idx];
            obs[joint_count + idx] = vel;
        }
        idx++;
    }
    return obs;
}

void JointComponent::PrintDebug() const {
    std::cout << "[Joint Debug] joint_count=" << joint_count << std::endl;
    for (size_t i = 0; i < motor_indices.size() && i < 12; ++i) {
        auto state = controller->GetMotorState(motor_indices[i]);
        float pos_obs = state.position - offsets[i];
        float vel_obs = state.velocity;
        
        // 膝关节特殊处理
        if (i >= 8 && i <= 11) {
            pos_obs /= 1.667f;
            vel_obs /= 1.667f;
            std::cout << "  [" << i << "] KNEE motor_pos=" << state.position 
                      << " offset=" << offsets[i]
                      << " obs_pos=" << pos_obs 
                      << " (ratio=1.667)" << std::endl;
        } else {
            std::cout << "  [" << i << "] motor_pos=" << state.position 
                      << " offset=" << offsets[i]
                      << " obs_pos=" << pos_obs << std::endl;
        }
    }
}

// Implementations for ActionComponent, CommandComponent, RoboObsFrame, RoboObs

Gamepad::Gamepad(const char* dev) : fd(-1), running(true), connected_(false) {
    fd = open(dev, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Failed to open gamepad device: " << dev << std::endl;
        running = false;
        return;
    }
    connected_ = true;
    last_event_time_ = std::chrono::steady_clock::now();
    std::memset(axes, 0, sizeof(axes));
    read_thread = std::thread(&Gamepad::ReadLoop, this);
}

Gamepad::~Gamepad() {
    running = false;
    if (read_thread.joinable()) {
        read_thread.join();
    }
    if (fd >= 0) {
        close(fd);
    }
}

float Gamepad::GetAxis(int axis) const {
    if (axis < 0 || axis >= JS_AXIS_LIMIT) return 0.0f;
    std::lock_guard<std::mutex> lock(data_mutex);
    return axes[axis];
}

bool Gamepad::IsConnected() const {
    return connected_;
}

bool Gamepad::IsFresh(int timeout_ms) const {
    if (!connected_) return false;
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_event_time_).count();
    return dt <= timeout_ms;
}

void Gamepad::ReadLoop() {
    struct js_event event;
    while (running) {
        ssize_t bytes = read(fd, &event, sizeof(event));
        if (bytes == sizeof(event)) {
            if (event.type & JS_EVENT_AXIS) {
                if (event.number < JS_AXIS_LIMIT) {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    axes[event.number] = (float)event.value / 32767.0f;
                }
            }
            last_event_time_ = std::chrono::steady_clock::now();
        } else if (bytes < 0) {
            // 设备断开或错误
            if (errno == ENODEV || errno == EIO) {
                std::cerr << "[Gamepad] Device disconnected or I/O error" << std::endl;
                connected_ = false;
                // 清零轴值
                std::lock_guard<std::mutex> lock(data_mutex);
                std::memset(axes, 0, sizeof(axes));
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

std::vector<float> ActionComponent::GetObs() const {
    return prev_actions;
}

void ActionComponent::SetAction(const std::vector<float>& action) {
    prev_actions = action;
}

// CommandComponent 实现

CommandComponent::CommandComponent(int command_dim, std::shared_ptr<ICommandSource> input)
    : command(command_dim, 0.0f), input_source_(std::move(input)) {
}

CommandComponent::CommandComponent(int command_dim, std::shared_ptr<Gamepad> gamepad)
    : command(command_dim, 0.0f), gamepad_(std::move(gamepad)) {
}

std::vector<float> CommandComponent::GetObs() const {
    // 强制归零模式（prewarm 阶段）
    if (force_zero_) {
        return {0.0f, 0.0f, 0.0f};
    }

    // 优先使用 ICommandSource 接口
    if (input_source_ && input_source_->IsReady()) {
        return input_source_->GetCommand();
    }

    // 向后兼容：直接使用 Gamepad（使用新鲜度检查）
    if (gamepad_ && gamepad_->IsFresh(500)) {
        return {
            -gamepad_->GetAxis(1) * 0.5f,
            -gamepad_->GetAxis(0) * 0.0f,
            -gamepad_->GetAxis(3) * 0.5f
        };
    }

    return command;
}

void CommandComponent::SetCommand(const std::vector<float>& cmd) {
    command = cmd;
}

void CommandComponent::Update() {
    // ICommandSource 可能需要更新（键盘是线程驱动，不需要）
    if (input_source_) {
        input_source_->Update();
    }
    // Gamepad 在自己的线程中更新
}

RoboObsFrame::RoboObsFrame() : timestamp(0) {}

void RoboObsFrame::AddComponent(std::shared_ptr<ObsComponent> component) {
    components.push_back(component);
}

std::vector<float> RoboObsFrame::GetObs() const {
    std::vector<float> obs;
    for (const auto& comp : components) {
        auto sub_obs = comp->GetObs();
        obs.insert(obs.end(), sub_obs.begin(), sub_obs.end());
    }
    return obs;
}

RoboObs::RoboObs(int history_length) : obs_dim(0), history_length(history_length) {}

void RoboObs::AddComponent(std::shared_ptr<ObsComponent> component) {
    frame.AddComponent(component);
}

void RoboObs::UpdateObs() {
    for (const auto& component : frame.components) {
        component->Update();
    }
    auto current_obs = frame.GetObs();
    obs_dim = current_obs.size();

    last_single_obs_ = std::move(current_obs);
    history.push_back(last_single_obs_);

    while (history.size() > (size_t)history_length) {
        history.pop_front();
    }
}

std::vector<float> RoboObs::GetWholeObs() const {
    std::vector<float> whole_obs;
    int missing = history_length - history.size();

    if (missing > 0 && obs_dim > 0) {
        std::vector<float> padding(missing * obs_dim, 0.0f);
        whole_obs.insert(whole_obs.end(), padding.begin(), padding.end());
    }

    for (const auto& obs : history) {
        whole_obs.insert(whole_obs.end(), obs.begin(), obs.end());
    }
    return whole_obs;
}

const std::vector<float>& RoboObs::GetSingleObs() const {
    if (history.empty()) return last_single_obs_;  // 始终返回有效缓存
    return history.back();
}

void RoboObs::ClearHistory() {
    history.clear();
}


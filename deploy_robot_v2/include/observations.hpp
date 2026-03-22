#pragma once
#include <ctime>
#include <cstdint>
#include <vector>
#include <memory>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <linux/joystick.h>
#include "robstride.hpp"

#define JS_AXIS_LIMIT 64

class ObsComponent {
public:
    virtual ~ObsComponent() = default;
    virtual std::vector<float> GetObs() const = 0;
    virtual void Update() = 0;
};

class Gamepad {
public:
    Gamepad(const char* dev = "/dev/input/js0");
    ~Gamepad();
    float GetAxis(int axis) const;
    bool IsConnected() const;
    bool IsFresh(int timeout_ms = 500) const;

private:
    void ReadLoop();
    int fd;
    std::thread read_thread;
    std::atomic<bool> running;
    std::atomic<bool> connected_{false};
    mutable std::mutex data_mutex;
    float axes[JS_AXIS_LIMIT]; // From linux/joystick.h
    std::chrono::steady_clock::time_point last_event_time_;
};

class RoboObsFrame {
public:
    std::vector<std::shared_ptr<ObsComponent>> components;

    RoboObsFrame();
    void AddComponent(std::shared_ptr<ObsComponent> component);
    std::vector<float> GetObs() const;

private:
    uint64_t timestamp;
};

class RoboObs {
public:
    int obs_dim;

    int history_length;
    std::deque<std::vector<float>> history;
    RoboObsFrame frame;

    RoboObs(int history_length);
    void AddComponent(std::shared_ptr<ObsComponent> component);
    void UpdateObs();
    std::vector<float> GetWholeObs() const;
    const std::vector<float>& GetSingleObs() const;

    // History 状态查询接口
    size_t ValidFrames() const { return history.size(); }
    bool IsHistoryFull() const { return history.size() == static_cast<size_t>(history_length); }
    void ClearHistory();

private:
    std::vector<float> last_single_obs_;  // 缓存最新单帧观测
};

class IMUComponent : public ObsComponent {
public:
    IMUComponent(const char* dev);
    ~IMUComponent();
    std::vector<float> GetObs() const override;

    void Update() override;

    // 健康状态接口
    bool IsReady() const;      // 是否已收到过有效数据
    bool IsFresh(int timeout_ms = 100) const;  // 最近是否更新过

    // 调试接口：打印 IMU 原始数据和映射后数据
    void PrintDebug() const;

private:
    const char* dev_path;
    static IMUComponent* instance_;

    std::thread update_thread_;
    std::atomic<bool> running_;
    mutable std::mutex data_mutex_;
    void UpdateLoop();

    float acc[3];
    float gyro[3];
    float angle[3];
    float quaternion[4];
    bool data_valid_;  // 是否收到过有效数据
    std::chrono::steady_clock::time_point last_update_time_;  // 最后更新时间

    void AutoScanSensor();
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t ucMs);
    int ConfigureSensorOutputs(void);
    static void SerialWriteRegister(uint8_t *p_ucData, uint32_t uiLen);

    static int fd;
    static int s_iCurBaud;
    static constexpr int c_uiBaud[] = {2400 , 4800 , 9600 , 19200 ,
        38400 , 57600 , 115200 , 230400 , 460800 , 921600};
};

class JointComponent : public ObsComponent {
public:
    int joint_count;
    JointComponent(int joint_count, std::shared_ptr<RobstrideController> controller, const std::vector<int>& motor_indices, const std::vector<float>& offsets) :
        joint_count(joint_count), controller(controller), motor_indices(motor_indices), offsets(offsets) {}
    std::vector<float> GetObs() const override;
    void Update() override {}

    // 调试接口：打印关节映射信息
    void PrintDebug() const;

private:
    std::shared_ptr<RobstrideController> controller;
    std::vector<int> motor_indices;
    std::vector<float> offsets;
};

class ActionComponent : public ObsComponent {
public:
    ActionComponent(int action_dim) : prev_actions(action_dim) {}
    ~ActionComponent() = default;
    std::vector<float> GetObs() const override;
    void SetAction(const std::vector<float>& action);
    void Update() override {}

private:
    std::vector<float> prev_actions;
};

// 前向声明
class ICommandSource;

class CommandComponent : public ObsComponent {
public:
    /**
     * @brief 使用 ICommandSource 接口构造（推荐）
     */
    CommandComponent(int command_dim, std::shared_ptr<ICommandSource> input);

    /**
     * @brief 使用 Gamepad 构造（向后兼容）
     * @deprecated 建议使用 ICommandSource 接口
     */
    CommandComponent(int command_dim, std::shared_ptr<Gamepad> gamepad = nullptr);

    ~CommandComponent() = default;
    std::vector<float> GetObs() const override;
    void SetCommand(const std::vector<float>& command);
    void Update() override;

    /**
     * @brief 设置强制归零模式
     * @param force_zero true 时命令强制返回 [0,0,0]
     *
     * 用于 prewarm 阶段防止用户输入污染 history
     */
    void SetForceZero(bool force_zero) { force_zero_ = force_zero; }

private:
    mutable std::vector<float> command;
    std::shared_ptr<ICommandSource> input_source_;
    std::shared_ptr<Gamepad> gamepad_;  // 向后兼容
    bool force_zero_ = false;
};

class RSDepthComponent : public ObsComponent {
public:
    RSDepthComponent(const char* dev, int width, int height);
    ~RSDepthComponent();
    std::vector<float> GetObs() const override;
    void Update() override {}
private:
    const char* dev_path;
    int width;
    int height;
    static RSDepthComponent* instance_;
    std::thread update_thread_;
    std::atomic<bool> running_;
    mutable std::mutex data_mutex_;
    void UpdateLoop();
};
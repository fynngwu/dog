#ifndef OBSERVATIONS_HPP
#define OBSERVATIONS_HPP

#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <deque>
#include <string>
#include <cstdint>

// Forward declarations
class Gamepad;
class RobstrideController;

// ============================================================================
// Base Component
// ============================================================================
class ObsComponent {
public:
    virtual ~ObsComponent() = default;
    virtual void Update() {}
    virtual std::vector<float> GetObs() const = 0;
};

// ============================================================================
// IMU Component (WIT IMU sensor)
// ============================================================================
class IMUComponent : public ObsComponent {
public:
    static IMUComponent* instance_;
    static int fd;
    static int s_iCurBaud;

    explicit IMUComponent(const char* dev_path);
    ~IMUComponent() override;

    void Update() override;
    std::vector<float> GetObs() const override;
    void UpdateLoop();

private:
    std::string dev_path;
    bool running_;
    std::thread update_thread_;

    float acc[3];
    float gyro[3];
    float angle[3];
    float quaternion[4];
    mutable std::mutex data_mutex_;

    void AutoScanSensor();
    int ConfigureSensorOutputs();

    // Sensor callbacks
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t ucMs);
    static void SerialWriteRegister(uint8_t* p_ucData, uint32_t uiLen);

    static constexpr int c_uiBaud[7] = {9600, 115200, 230400, 460800, 921600, 1000000, 2000000};

    // Registry addresses
    enum Register {
        AX = 0x00, AY = 0x01, AZ = 0x02,
        GX = 0x03, GY = 0x04, GZ = 0x05,
        Roll = 0x55, Pitch = 0x56, Yaw = 0x57,
        q0 = 0x59, q1 = 0x5A, q2 = 0x5B, q3 = 0x5C,
    };

    enum SensorContent {
        RSW_ACC = 0x01, RSW_GYRO = 0x02, RSW_Q = 0x04
    };

    enum RetCode {
        WIT_HAL_OK = 0,
    };
};

// ============================================================================
// Joint Component
// ============================================================================
class JointComponent : public ObsComponent {
public:
    static constexpr int joint_count = 12;

    JointComponent(int count,
                    std::shared_ptr<RobstrideController> controller,
                    const std::vector<int>& motor_indices,
                    const std::vector<float>& offsets);

    std::vector<float> GetObs() const override;

private:
    int count_;
    std::shared_ptr<RobstrideController> controller;
    std::vector<int> motor_indices;
    std::vector<float> offsets;
};

// ============================================================================
// Action Component
// ============================================================================
class ActionComponent : public ObsComponent {
public:
    explicit ActionComponent(int dim);

    std::vector<float> GetObs() const override;
    void SetAction(const std::vector<float>& action);

private:
    std::vector<float> prev_actions;
};

// ============================================================================
// Command Component
// ============================================================================
class CommandComponent : public ObsComponent {
public:
    CommandComponent(int dim, std::shared_ptr<Gamepad> gamepad);

    std::vector<float> GetObs() const override;
    void SetCommand(const std::vector<float>& cmd);
    void Update() override;

private:
    mutable std::vector<float> command;
    std::shared_ptr<Gamepad> gamepad;
};

// ============================================================================
// Gamepad (Joystick) Component
// ============================================================================
class Gamepad {
public:
    explicit Gamepad(const char* dev);
    ~Gamepad();

    float GetAxis(int axis) const;
    bool IsConnected() const;

private:
    int fd;
    bool running;
    float axes[32];  // JS_AXIS_LIMIT
    mutable std::mutex data_mutex;
    std::thread read_thread;

    static constexpr int JS_AXIS_LIMIT = 32;

    void ReadLoop();
};

// ============================================================================
// RoboObs Frame & History
// ============================================================================
class RoboObsFrame {
public:
    RoboObsFrame();

    void AddComponent(std::shared_ptr<ObsComponent> component);
    std::vector<float> GetObs() const;

    // Allow RoboObs to access private members
    friend class RoboObs;

private:
    std::vector<std::shared_ptr<ObsComponent>> components;
};

class RoboObs {
public:
    explicit RoboObs(int history_length);

    void AddComponent(std::shared_ptr<ObsComponent> component);
    void UpdateObs();
    std::vector<float> GetWholeObs() const;
    std::vector<float> GetSingleObs() const;

    int GetObsDim() const { return obs_dim; }

private:
    RoboObsFrame frame;
    std::deque<std::vector<float>> history;
    int history_length;
    int obs_dim;
};

#endif // OBSERVATIONS_HPP

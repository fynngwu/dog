#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace minimal {

class IMUReader {
public:
    IMUReader();
    ~IMUReader();

    bool Initialize(const std::string& device = "/dev/ttyCH341USB0");
    void Stop();

    bool IsReady() const;
    bool IsFresh(int timeout_ms = 100) const;
    std::vector<float> GetObservation() const;
    void PrintDebug() const;

    static IMUReader* instance_;
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t ucMs);
    static void SerialWriteRegister(uint8_t* p_ucData, uint32_t uiLen);

private:
    struct Data {
        float gyro[3];
        float quat[4];
        std::atomic<bool> valid;
        std::chrono::steady_clock::time_point last_update;
        int fd;
    };

    void UpdateLoop();
    std::vector<float> ComputeGravityProjection(const float quat[4]) const;

    std::thread thread_;
    std::atomic<bool> running_;
    mutable std::mutex mutex_;
    Data data_;
};

}  // namespace minimal

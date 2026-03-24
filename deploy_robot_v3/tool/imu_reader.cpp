#include "imu_reader.hpp"

extern std::atomic<bool> g_running;

#include <cmath>
#include <cstring>
#include <iostream>
#include <unistd.h>

#include "serial.h"
#include "wit_c_sdk.h"

extern "C" void imu_reader_sensor_update(uint32_t uiReg, uint32_t uiRegNum);
extern "C" void imu_reader_delay_ms(uint16_t ucMs);
extern "C" void imu_reader_serial_write(uint8_t* p_ucData, uint32_t uiLen);

namespace minimal {

IMUReader* IMUReader::instance_ = nullptr;
static volatile char s_cDataUpdate = 0;

IMUReader::IMUReader() : running_(false) {
    std::memset(data_.gyro, 0, sizeof(data_.gyro));
    std::memset(data_.quat, 0, sizeof(data_.quat));
    data_.valid.store(false);
    data_.last_update = std::chrono::steady_clock::now();
    data_.fd = -1;
}

IMUReader::~IMUReader() {
    Stop();
}

bool IMUReader::Initialize(const std::string& device) {
    instance_ = this;
    data_.valid.store(false);

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitDelayMsRegister(imu_reader_delay_ms);
    WitSerialWriteRegister(imu_reader_serial_write);
    WitRegisterCallBack(imu_reader_sensor_update);

    constexpr int kBauds[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    int fd = -1;
    for (int baud : kBauds) {
        if (!g_running) {
            break;
        }
        if (fd >= 0) {
            serial_close(fd);
        }

        fd = serial_open(reinterpret_cast<unsigned char*>(const_cast<char*>(device.c_str())), baud);
        if (fd < 0) {
            continue;
        }

        int retry = 2;
        do {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            imu_reader_delay_ms(200);

            unsigned char buff[1];
            while (serial_read_data(fd, buff, 1) > 0) {
                WitSerialDataIn(buff[0]);
            }

            if (s_cDataUpdate != 0) {
                data_.fd = fd;
                std::cout << "[IMUReader] Connected at baud " << baud << std::endl;
                break;
            }
            retry--;
        } while (retry > 0 && g_running);

        if (s_cDataUpdate != 0) {
            break;
        }
    }

    if (data_.fd < 0 || s_cDataUpdate == 0) {
        std::cerr << "[IMUReader] Cannot find IMU sensor" << std::endl;
        return false;
    }

    if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_Q) != WIT_HAL_OK) {
        std::cerr << "[IMUReader] Configure IMU failed" << std::endl;
        return false;
    }

    running_ = true;
    thread_ = std::thread(&IMUReader::UpdateLoop, this);

    const auto start = std::chrono::steady_clock::now();
    while (!data_.valid.load() && g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > 5000) {
            std::cerr << "[IMUReader] IMU data timeout" << std::endl;
            Stop();
            return false;
        }
    }

    if (!g_running) {
        Stop();
        return false;
    }

    std::cout << "[IMUReader] Initialized" << std::endl;
    return true;
}

void IMUReader::Stop() {
    running_ = false;
    const int fd = data_.fd;
    data_.fd = -1;
    if (fd >= 0) {
        serial_close(fd);
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

void IMUReader::UpdateLoop() {
    while (running_ && data_.fd >= 0) {
        unsigned char buff[1];
        while (serial_read_data(data_.fd, buff, 1) > 0) {
            WitSerialDataIn(buff[0]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

bool IMUReader::IsFresh(int timeout_ms) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!data_.valid.load()) {
        return false;
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - data_.last_update).count();
    return age <= timeout_ms;
}

bool IMUReader::IsReady() const {
    return data_.valid.load();
}

std::vector<float> IMUReader::ComputeGravityProjection(const float quat[4]) const {
    const float qw = quat[0];
    const float qx = quat[1];
    const float qy = quat[2];
    const float qz = quat[3];

    float gx = 2.0f * (qx * qz - qw * qy);
    float gy = 2.0f * (qy * qz + qw * qx);
    float gz = 1.0f - 2.0f * (qx * qx + qy * qy);

    const float norm = std::sqrt(gx * gx + gy * gy + gz * gz);
    if (norm > 1e-6f) {
        gx /= norm;
        gy /= norm;
        gz /= norm;
    }

    return {-gy, gx, -gz};
}

std::vector<float> IMUReader::GetObservation() const {
    float gyro[3];
    float quat[4];
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::memcpy(gyro, data_.gyro, sizeof(gyro));
        std::memcpy(quat, data_.quat, sizeof(quat));
    }

    std::vector<float> obs;
    obs.reserve(6);
    obs.push_back(gyro[1] * static_cast<float>(M_PI) / 180.0f);
    obs.push_back(-gyro[0] * static_cast<float>(M_PI) / 180.0f);
    obs.push_back(gyro[2] * static_cast<float>(M_PI) / 180.0f);

    const auto grav = ComputeGravityProjection(quat);
    obs.insert(obs.end(), grav.begin(), grav.end());
    return obs;
}

void IMUReader::PrintDebug() const {
    const auto obs = GetObservation();
    std::cout << "[IMUReader] gyro: [" << obs[0] << ", " << obs[1] << ", " << obs[2]
              << "] grav: [" << obs[3] << ", " << obs[4] << ", " << obs[5] << "]"
              << " fresh=" << (IsFresh(100) ? "Y" : "N") << std::endl;
}

void IMUReader::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    s_cDataUpdate = 1;
    if (!instance_) {
        return;
    }

    std::lock_guard<std::mutex> lock(instance_->mutex_);
    for (uint32_t i = 0; i < uiRegNum; ++i) {
        switch (uiReg) {
            case GX:
            case GY:
            case GZ:
                for (int j = 0; j < 3; ++j) {
                    instance_->data_.gyro[j] = sReg[GX + j] / 32768.0f * 2000.0f;
                }
                break;
            case q0:
            case q1:
            case q2:
            case q3:
                for (int j = 0; j < 4; ++j) {
                    instance_->data_.quat[j] = sReg[q0 + j] / 32768.0f;
                }
                instance_->data_.valid.store(true);
                instance_->data_.last_update = std::chrono::steady_clock::now();
                break;
            default:
                break;
        }
        uiReg++;
    }
}

void IMUReader::Delayms(uint16_t ucMs) {
    usleep(ucMs * 1000);
}

void IMUReader::SerialWriteRegister(uint8_t* p_ucData, uint32_t uiLen) {
    if (!instance_ || instance_->data_.fd < 0) {
        return;
    }
    serial_write_data(instance_->data_.fd, p_ucData, uiLen);
}

}  // namespace minimal

extern "C" void imu_reader_sensor_update(uint32_t uiReg, uint32_t uiRegNum) {
    minimal::IMUReader::SensorDataUpdata(uiReg, uiRegNum);
}

extern "C" void imu_reader_delay_ms(uint16_t ucMs) {
    minimal::IMUReader::Delayms(ucMs);
}

extern "C" void imu_reader_serial_write(uint8_t* p_ucData, uint32_t uiLen) {
    minimal::IMUReader::SerialWriteRegister(p_ucData, uiLen);
}

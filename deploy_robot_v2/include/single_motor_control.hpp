#pragma once

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <iostream>

#include "robstride.hpp"
#include "can_interface.hpp"

class SingleMotorControl {
public:
    SingleMotorControl() {
        rs_controller_ = std::make_shared<RobstrideController>();
        motor_indices_.assign(12, -1);
    }
    ~SingleMotorControl() = default;

    // 初始化 CAN 与 controller
    bool Init() {
        // create CANs and bind
        for (int i = 0; i < 4; ++i) {
            std::string can_name = std::string("candle") + std::to_string(i);
            auto can = std::make_shared<CANInterface>(can_name.c_str());
            cans_.push_back(can);
            rs_controller_->BindCAN(can);
        }
        return true;
    }

    // Bind physical motor id (like 1,2,...15). Returns internal index or -1 on fail
    int BindMotorByPhysicalId(int physical_id) {
        for (size_t pos = 0; pos < motor_ids.size(); ++pos) {
            if (motor_ids[pos] == physical_id) {
                int group = pos % 4; // which CAN
                std::string can_if_str = std::string("candle") + std::to_string(group);

                std::unique_ptr<RobstrideController::MotorInfo> motor_info = std::make_unique<RobstrideController::MotorInfo>();
                motor_info->motor_id = physical_id;
                motor_info->host_id = 0xFD;
                int idx = rs_controller_->BindMotor(can_if_str.c_str(), std::move(motor_info));
                if (idx < 0) return -1;
                rs_controller_->EnableMotor(idx);
                rs_controller_->SetMITParams(idx, {MIT_KP, MIT_KD, MIT_VEL_LIMIT, MIT_TORQUE_LIMIT});
                return idx;
            }
        }
        return -1;
    }

    // Startup: bind all motors and move them to joint_offsets (like main)
    bool StartupAll() {
        // Bind all motors and store internal indices
        for (size_t pos = 0; pos < motor_ids.size(); ++pos) {
            int phys = motor_ids[pos];
            int idx = BindMotorByPhysicalId(phys);
            if (idx < 0) {
                std::cerr << "Failed to bind motor " << phys << std::endl;
                return false;
            }
            motor_indices_[pos] = idx;
        }

        // Interpolate to joint_offsets
        unsigned int interval = 10;
        for (unsigned int step = 1; step <= interval; ++step) {
            float factor = float(step) / float(interval);
            for (size_t pos = 0; pos < motor_ids.size(); ++pos) {
                int idx = motor_indices_[pos];
                if (idx < 0) continue;
                float target = joint_offsets[pos] * factor;
                rs_controller_->SendMITCommand(idx, target);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        return true;
    }

    // Send angle for single physical motor id; applies soft-limits based on XML and joint_offsets
    bool SendAngle(int physical_id, float angle_rad) {
        int pos = find_motor_pos(physical_id);
        if (pos < 0) return false;

        // Treat angle_rad as the desired offset from zero; add joint_offsets to get absolute target
        float desired = angle_rad + joint_offsets[pos];

        // compute soft limits based on joint_offsets and xml ranges
        float lower = joint_offsets[pos] + xml_min[pos];
        float upper = joint_offsets[pos] + xml_max[pos];

        // clip the absolute desired angle into soft limits
        float clipped = desired;
        if (clipped < lower) clipped = lower;
        if (clipped > upper) clipped = upper;

        int idx = motor_indices_[pos];
        if (idx < 0) {
            // try to bind on demand
            idx = BindMotorByPhysicalId(physical_id);
            if (idx < 0) return false;
            motor_indices_[pos] = idx;
        }

        rs_controller_->SendMITCommand(idx, clipped);
        return true;
    }

private:
    std::shared_ptr<RobstrideController> rs_controller_;
    std::vector<std::shared_ptr<CANInterface>> cans_;
    std::vector<int> motor_indices_;

    // helper: find motor position in the 12-array; -1 if not found
    int find_motor_pos(int physical_id) {
        for (size_t i = 0; i < motor_ids.size(); ++i) if (motor_ids[i] == physical_id) return (int)i;
        return -1;
    }

    // static configuration copied from main.cpp
    static constexpr float HIPA_OFFSET = 0.37f;
    static constexpr float HIPF_OFFSET = 0.13f;
    static constexpr float KNEE_OFFSET = 1.06f * 1.667f;
    static const std::vector<float> joint_offsets;
    static const std::vector<char> can_ids;
    static const std::vector<int> motor_ids;
    static constexpr float MIT_KP = 5.0f;
    static constexpr float MIT_KD = 0.5f;
    static constexpr float MIT_VEL_LIMIT = 12.57f;
    static constexpr float MIT_TORQUE_LIMIT = 44.0f;
    static const std::vector<float> xml_min;
    static const std::vector<float> xml_max;
};

// Definitions of static members
const std::vector<float> SingleMotorControl::joint_offsets = {
    HIPA_OFFSET, -HIPA_OFFSET, -HIPA_OFFSET, HIPA_OFFSET,
    HIPF_OFFSET, HIPF_OFFSET, -HIPF_OFFSET, -HIPF_OFFSET,
    KNEE_OFFSET, KNEE_OFFSET, -KNEE_OFFSET, -KNEE_OFFSET,
};
const std::vector<char> SingleMotorControl::can_ids = {'0','1','2','3'};
const std::vector<int> SingleMotorControl::motor_ids = {
    1, 5, 9, 13,
    2, 6, 10, 14,
    3, 7, 11, 15,
};
const std::vector<float> SingleMotorControl::xml_min = {
    -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,
    -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,
    -1.2217299f * 1.667f, -1.2217299f * 1.667f, -0.3f * 1.667f, -0.3f * 1.667f
};
const std::vector<float> SingleMotorControl::xml_max = {
    0.7853982f, 0.7853982f, 0.7853982f, 0.7853982f,
    0.8726683f, 0.8726683f, 1.2217342f, 1.2217305f,
    0.3f * 1.667f, 0.3f * 1.667f, 1.2217287f * 1.667f, 1.2217287f * 1.667f
};


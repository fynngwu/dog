#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <ctime>
#include <cstdlib>

#include "observations.hpp"
#include "stubs/tensorrt_inference.hpp"
#include "stubs/robstride.hpp"
#include "debug_utils/data_logger.hpp"

// HipA HipF Knee
// LF LR RF RR
constexpr float HIPA_OFFSET = 0.37;
constexpr float HIPF_OFFSET = 0.13;
constexpr float KNEE_OFFSET = 1.06 * 1.667;
const std::vector<float> joint_offsets = {
    HIPA_OFFSET, -HIPA_OFFSET, -HIPA_OFFSET, HIPA_OFFSET,
    HIPF_OFFSET, HIPF_OFFSET, -HIPF_OFFSET, -HIPF_OFFSET,
    KNEE_OFFSET, KNEE_OFFSET, -KNEE_OFFSET, -KNEE_OFFSET,
};

const std::vector<char> can_ids = {
    '0', '1', '2', '3',
};

const std::vector<int> motor_ids = {
    1, 5, 9, 13,
    2, 6, 10, 14,
    3, 7, 11, 15,
};


constexpr float MIT_KP = 40.0f;
constexpr float MIT_KD = 0.5f;
constexpr float MIT_ANG_LIMIT = 12.57f;
constexpr float MIT_VEL_LIMIT = 44.0f;
constexpr float MIT_TORQUE_LIMIT = 17.0f;
#include <algorithm>

/**
 * @brief 对 shared_ptr 指向的 vector 进行限幅处理
 * 
 * @param data 需要处理的数据指针 (in-place 修改)
 * @param upper_limit 上限 vector
 * @param lower_limit 下限 vector
 */
void clip_vector(std::shared_ptr<std::vector<float>> data, 
                 const std::vector<float>& upper_limit, 
                 const std::vector<float>& lower_limit) {
    // 基础检查：指针不能为空且大小必须匹配
    if (!data || data->empty()) return;
    
    size_t n = data->size();
    if (n != upper_limit.size() || n != lower_limit.size()) {
        // 如果维度不匹配，可以根据需要抛出异常或直接返回
        return;
    }

    // 遍历并限制每个元素的值
    for (size_t i = 0; i < n; ++i) {
        (*data)[i] = std::clamp((*data)[i], lower_limit[i], upper_limit[i]);
    }
}

/**
  * @brief 插值函数
  */
void startup(std::shared_ptr<RobstrideController> rs_controller, std::vector<int>& motor_indices){
    
    for (int j = 0; j < 4; ++j) {
        const char *can_if = "candle";
        std::string can_if_str = std::string(can_if) + can_ids[j];
        for(int i = 0; i < 3; ++i) {
            std::unique_ptr<RobstrideController::MotorInfo> motor_info = std::make_unique<RobstrideController::MotorInfo>();
            motor_info->motor_id = motor_ids[j + i * 4];
            motor_info->host_id = 0xFD;

            int idx = rs_controller->BindMotor(can_if_str.c_str(), std::move(motor_info));

            motor_indices[j + i * 4] = idx;
            rs_controller->EnableMotor(idx); // Enable motor to receive feedback
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            rs_controller->EnableAutoReport(idx); // Enable motor to receive feedback
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            rs_controller->EnableAutoReport(idx); // Enable motor to receive feedback
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            // rs_controller->SetZero(idx);
            // std::this_thread::sleep_for(std::chrono::milliseconds(2));
            rs_controller->SetMITParams(idx, {MIT_KP, MIT_KD, MIT_VEL_LIMIT, MIT_TORQUE_LIMIT});
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    
    unsigned int interval = 20; 
    for (unsigned int step = 1; step <= interval; ++step){
        for (int j = 0; j < 4; ++j) {
            const char *can_if = "candle";
            std::string can_if_str = std::string(can_if) + can_ids[j];
            for(int i = 0; i < 3; ++i) {
                int global_motor_idx = j + i * 4;
                float factor = (float)step / (float)interval;
                rs_controller->SendMITCommand(motor_indices[global_motor_idx], joint_offsets[global_motor_idx] * factor);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }   
// send:
//         for (int i = 0; i < 12; i++) {
//             rs_controller->EnableMotor(motor_indices[i]);
//             std::this_thread::sleep_for(std::chrono::milliseconds(20));
//             rs_controller->SetMITParams(motor_indices[i], {MIT_KP, MIT_KD, MIT_VEL_LIMIT, MIT_TORQUE_LIMIT});
//             std::this_thread::sleep_for(std::chrono::milliseconds(20));
//             rs_controller->SendMITCommand(motor_indices[i], joint_offsets[i] * step / interval);
//             std::this_thread::sleep_for(std::chrono::milliseconds(20));
//         }
//         if (send_times < 3) goto send;
    }
}

int main() {

    // auto imu_component = std::make_shared<IMUComponent>("/dev/ttyCH341USB0");
    // RoboObs obs(10);
    // obs.AddComponent(imu_component);

    // while(1){
    //     static auto last_time = std::chrono::steady_clock::now();

    //     obs.UpdateObs();
    //     std::vector<float> obs_vec = obs.GetWholeObs();
    //     std::cout << "Single Obs: ";
    //     std::vector<float> single_obs_vec = obs.GetSingleObs();
    //     for(float obs : single_obs_vec) {
    //         std::cout << std::fixed << std::setprecision(2) << obs << " ";
    //     }
    //     std::cout << std::endl;

    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // }

    auto rs_controller = std::make_shared<RobstrideController>();
    std::shared_ptr<CANInterface> can0_interface = std::make_shared<CANInterface>("candle0");
    std::shared_ptr<CANInterface> can1_interface = std::make_shared<CANInterface>("candle1");
    std::shared_ptr<CANInterface> can2_interface = std::make_shared<CANInterface>("candle2");
    std::shared_ptr<CANInterface> can3_interface = std::make_shared<CANInterface>("candle3");
    
    rs_controller->BindCAN(can0_interface);
    rs_controller->BindCAN(can1_interface);
    rs_controller->BindCAN(can2_interface);
    rs_controller->BindCAN(can3_interface);
    // Bind motors 0-11
    std::vector<int> motor_indices(12);

    startup(rs_controller, motor_indices);

    std::cout << "Press any key to continue..." << std::endl;
    std::cin.get();

    auto imu_component = std::make_shared<IMUComponent>("/dev/ttyCH341USB0");
    auto gamepad = std::make_shared<Gamepad>("/dev/input/js0");
    auto command_component = std::make_shared<CommandComponent>(3, gamepad);
    auto joint_component = std::make_shared<JointComponent>(12, 
            rs_controller, motor_indices, joint_offsets);
    auto action_component = std::make_shared<ActionComponent>(12);

    RoboObs obs(10);
    obs.AddComponent(imu_component);
    obs.AddComponent(command_component);
    obs.AddComponent(joint_component);
    obs.AddComponent(action_component);

    InferenceEngine inference_engine("/home/ares/pure_cpp/policy.engine", 450, 12);

    std::vector<float> action_vec(12, 0.0f);

    // === Data Logger for debugging ===
    // 日志文件路径，可以改为绝对路径
    const std::string log_filepath = "/home/wufy/project2026/dog/debug_utils/deploy_log.bin";
    const int obs_dim = 450;   // 10 frames * 45 single obs
    const int action_dim = 12;
    DataLogger logger(log_filepath, obs_dim, action_dim);
    if (!logger.IsOpen()) {
        std::cerr << "Warning: DataLogger failed to open, logging disabled" << std::endl;
    } else {
        std::cout << "DataLogger initialized: " << log_filepath << std::endl;
    }
    // 每隔 N 帧记录一次 (避免文件过大，默认 50ms/帧 = 20fps, 每秒1帧记录)
    constexpr int LOG_INTERVAL = 20;
    int log_counter = 0;

    while (1) {
        static auto last_time = std::chrono::steady_clock::now();

        obs.UpdateObs();
        std::vector<float> obs_vec = obs.GetWholeObs();
        inference_engine.infer(obs_vec, action_vec);
        std::cout << "Single Obs: ";
        std::vector<float> single_obs_vec = obs.GetSingleObs();
        for(float obs : single_obs_vec) {
            std::cout << std::fixed << std::setprecision(2) << obs << " ";
        }
        std::cout << std::endl;
        std::cout << "Action: ";
        for(float action : action_vec) {
            std::cout << std::fixed << std::setprecision(2) << action << " ";
        }
        std::cout << std::endl;
        action_component->SetAction(action_vec);

        // === 记录日志 (按间隔) ===
        if (logger.IsOpen() && ++log_counter >= LOG_INTERVAL) {
            logger.Log(obs_vec, action_vec);
            log_counter = 0;
            static int log_frames = 0;
            if (log_frames % 100 == 0) {
                std::cout << "[Logger] Recorded frames: " << logger.GetFrameCount() << std::endl;
            }
            log_frames++;
        }

        // === 使用 single_motor 的限位逻辑：把网络输出视为相对于零位的偏移，
        //      加上 joint_offsets 得到绝对目标角度，再按 XML 范围裁剪后发送 ===
        float scale = 0.25f; // 网络输出到角度的缩放因子（按需调整）

        const std::vector<float> xml_min = {
            -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,
            -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,
            -1.2217299f * 1.667f, -1.2217299f * 1.667f, -0.6, -0.6
        };
        const std::vector<float> xml_max = {
            0.7853982f, 0.7853982f, 0.7853982f, 0.7853982f,
            0.8726683f, 0.8726683f, 1.2217342f, 1.2217305f,
            0.6, 0.6, 1.2217287f * 1.667f, 1.2217287f * 1.667f
        };

        std::cout << "Real Action in Degree:";
        for (int i = 0; i < 12; ++i) {
            // Apply gear correction for knee joints on the action value, without modifying action_vec
            float act = action_vec[i];

            // if (i == 2) {
            //     act = 1.0f; // 给第 0 号电机一个固定动作
            // } else {
            //     act = 0.0f; // 让其他 11 个电机全部强制归零
            // }

            if (i >= 8 && i <= 11) act = act * 1.667f;

            // desired: absolute angle = action * scale + joint offset
            float desired = act * scale + joint_offsets[i];
            float lower = joint_offsets[i] + xml_min[i];
            float upper = joint_offsets[i] + xml_max[i];
            float clipped = std::clamp(desired, lower, upper);
            // rs_controller->SendMITCommand(motor_indices[i], clipped);
            std::cout << std::fixed << std::setprecision(2) << clipped << " ";

        }
        std::cout << std::endl;

        // === 原始发送循环（已注释保留） ===
        // for (int i = 0; i < 12; i++) {
        //     rs_controller->SendMITCommand(motor_indices[i], action_vec[i] * 0.25 + joint_offsets[i]);
        //     // rs_controller->SendMITCommand(motor_indices[i], joint_offsets[i]);
        // }

        auto now = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        // std::cout << "两次循环间隔: " << duration_ms << " ms" << std::endl;
        last_time = now;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
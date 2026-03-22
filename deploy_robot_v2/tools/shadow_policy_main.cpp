/**
 * @file shadow_policy_main.cpp
 * @brief 模式3：Shadow 模式测试
 *
 * 流程：
 * 1. 上电并平滑到 offset
 * 2. 构建 IMU + command + joint + prev_action
 * 3. 预热 history 10 帧有效观测（IMU 和电机都就绪，command 强制为 0）
 * 4. 每帧跑 inference
 * 5. 记录 obs / action / clipped_target / dt / inference_time
 * 6. 电机 hold 在 offset，不发送 policy 输出
 *
 * 用途：排查"是不是 policy 推理/历史预热有问题"，先不发 policy 输出到电机
 *
 * 注意：此模式会发送 MIT 命令让电机 hold 在 offset，不是完全不发控制
 *
 * 安全机制：
 * - 运行期检查电机 online/fresh，不通过就继续 hold
 * - Policy 加载/推理失败时安全降级
 * - prewarm 只计入有效帧，command 强制归零
 * - 键盘输入有 deadman timeout (200ms)
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>
#include <cmath>
#include <exception>
#include <atomic>

#include "robstride.hpp"
#include "can_interface.hpp"
#include "robot_config.hpp"
#include "robot_bringup.hpp"
#include "observations.hpp"
#include "policy_runner.hpp"
#include "session_logger.hpp"
#include "input/input_source.hpp"
#include "input/gamepad_command_source.hpp"
#include "input/keyboard_command_source.hpp"

// 使用 std::atomic 保证线程安全的信号处理
std::atomic<bool> g_running{true};
std::atomic<bool> g_policy_error{false};
std::atomic<bool> g_motor_offline{false};

void signal_handler(int) {
    // 只置位，不做 I/O（async-signal-safe）
    g_running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    // 解析输入源参数
    std::string input_mode = "auto";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_mode = argv[++i];
        }
    }

    std::cout << "=== Shadow Policy Tool ===" << std::endl;
    std::cout << "Purpose: Test policy inference WITHOUT sending policy output to motors" << std::endl;
    std::cout << "NOTE: Motors will hold at offset position during the test" << std::endl;
    std::cout << "Input: " << input_mode << std::endl;
    std::cout << std::endl;

    // 创建日志目录：shadow_{YYYYMMDD_HHMMSS}
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << cfg::kLogDir << "/shadow_"
       << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    std::string log_dir = ss.str();

    SessionLogger logger;
    if (logger.Open(log_dir)) {
        logger.LogMeta(R"({"mode": "shadow", "input": ")" + input_mode + R"(", "policy_input_dim": 450, "history_len": 10, "control_hz": 50})");
    }
    logger.LogEvent("Shadow mode started");

    // 创建控制器和启动器
    auto controller = std::make_shared<RobstrideController>();
    RobotBringup bringup(controller);

    // 绑定 CAN
    logger.LogEvent("Binding CAN interfaces");
    if (!bringup.BindAllCAN()) {
        logger.LogEvent("ERROR: Failed to bind CAN");
        return 1;
    }

    // 绑定电机
    logger.LogEvent("Binding motors");
    std::vector<int> motor_indices;
    if (!bringup.BindAllMotors(motor_indices)) {
        logger.LogEvent("ERROR: Failed to bind motors");
        return 1;
    }

    // 使能电机
    logger.LogEvent("Enabling motors");
    if (!bringup.EnableAll(motor_indices)) {
        logger.LogEvent("ERROR: Failed to enable motors");
        return 1;
    }

    // 开启自动上报
    logger.LogEvent("Enabling auto report");
    if (!bringup.EnableAutoReportAll(motor_indices)) {
        logger.LogEvent("ERROR: Failed to enable auto report");
        return 1;
    }

    // 等待反馈
    logger.LogEvent("Waiting for feedback");
    if (!bringup.WaitForFeedback(motor_indices, 3000)) {
        logger.LogEvent("ERROR: Feedback timeout");
        return 1;
    }

    // 平滑移动到 offset
    logger.LogEvent("Moving to offset pose");
    if (!bringup.MoveCurrentToOffsets(motor_indices)) {
        logger.LogEvent("ERROR: Failed to move to offsets");
        return 1;
    }

    // 创建观测组件
    auto imu_component = std::make_shared<IMUComponent>(cfg::kImuDevice.c_str());

    // 创建输入源
    auto input_source = CreateInputSource(input_mode);
    logger.LogEvent(std::string("Input source: ") + input_source->Name());

    auto command_component = std::make_shared<CommandComponent>(3, input_source);
    auto joint_component = std::make_shared<JointComponent>(12, controller, motor_indices, cfg::kJointOffsets);
    auto action_component = std::make_shared<ActionComponent>(12);

    RoboObs obs(cfg::kHistoryLen);
    obs.AddComponent(imu_component);
    obs.AddComponent(command_component);
    obs.AddComponent(joint_component);
    obs.AddComponent(action_component);

    // 创建 Policy Runner - 加异常保护
    std::unique_ptr<PolicyRunner> policy;
    logger.LogEvent("Loading policy engine");
    try {
        policy = std::make_unique<PolicyRunner>(cfg::kPolicyEnginePath);
        std::cout << "[Policy] Engine loaded successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Policy] ERROR: Failed to load policy engine: " << e.what() << std::endl;
        logger.LogEvent(std::string("ERROR: Policy load failed: ") + e.what());
        g_policy_error = true;
        std::cerr << "[Policy] Cannot run shadow test without policy. Exiting." << std::endl;
        return 1;
    }

    // 初始化 action 为 0
    std::vector<float> zero_action(12, 0.0f);
    action_component->SetAction(zero_action);

    // ========== 预热 history（只计入有效帧，command 强制为 0）==========
    logger.LogEvent("Prewarming history with valid frames");
    std::cout << "[Prewarm] Filling history with valid observations..." << std::endl;
    std::cout << "  (IMU and motors must be fresh for each frame)" << std::endl;
    std::cout << "  (Command forced to zero during prewarm)" << std::endl;

    // prewarm 阶段强制 command 为 0
    command_component->SetForceZero(true);

    int valid_frames = 0;
    int total_attempts = 0;
    const int max_attempts = cfg::kHistoryLen * 10;  // 最多尝试 100 次

    while (valid_frames < cfg::kHistoryLen && total_attempts < max_attempts && g_running) {
        obs.UpdateObs();

        // 检查传感器是否就绪
        if (CheckSensorsReady(imu_component, controller, motor_indices, 100, 100)) {
            valid_frames++;
            std::cout << "\r  Valid frame " << valid_frames << "/" << cfg::kHistoryLen << std::flush;
        } else {
            // 无效帧：清空 history，重新开始计数
            obs.ClearHistory();
            valid_frames = 0;
            if (total_attempts % 10 == 0) {
                std::cout << "\r  Waiting for valid sensors... (attempt " << total_attempts << ")" << std::flush;
            }
        }

        // Hold 在 offset
        bringup.HoldOffsets(motor_indices);

        total_attempts++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / cfg::kControlHz));
    }

    if (valid_frames < cfg::kHistoryLen) {
        std::cerr << "\n[Prewarm] ERROR: Failed to get " << cfg::kHistoryLen << " valid frames after " << max_attempts << " attempts" << std::endl;
        logger.LogEvent("ERROR: Prewarm failed - sensors not ready");
        return 1;
    }

    // prewarm 完成，恢复 command 输入
    command_component->SetForceZero(false);

    std::cout << std::endl;
    logger.LogEvent("History prewarm complete with valid frames");

    // ========== Shadow 循环 ==========
    logger.LogEvent("Starting shadow loop");
    std::cout << "[Shadow] Running policy inference (press Ctrl+C to exit)..." << std::endl;
    std::cout << std::endl;

    uint64_t tick = 0;
    auto t0 = std::chrono::steady_clock::now();
    auto last = t0;

    while (g_running) {
        auto loop_start = std::chrono::steady_clock::now();

        // ========== 运行期安全检查：IMU 和电机是否都就绪且新鲜 ==========
        if (!CheckSensorsReady(imu_component, controller, motor_indices, 100, 100)) {
            if (!g_motor_offline) {
                std::cerr << "\n[Safety] Sensor not ready/fresh (IMU or motor)!" << std::endl;
                logger.LogEvent("SAFETY: Sensor not ready/fresh");
                g_motor_offline = true;
            }
            // 继续运行，但只 hold
            bringup.HoldOffsets(motor_indices);
            last = loop_start;
            auto next_time = loop_start + std::chrono::milliseconds(1000 / cfg::kControlHz);
            std::this_thread::sleep_until(next_time);
            continue;
        } else {
            g_motor_offline = false;
        }

        // 更新观测
        obs.UpdateObs();
        auto single_obs = obs.GetSingleObs();
        auto whole_obs = obs.GetWholeObs();

        // 推理 - 加异常保护
        PolicyOutput policy_out;
        double infer_ms = 0.0;
        bool infer_success = false;

        try {
            auto infer_t0 = std::chrono::steady_clock::now();
            policy_out = policy->Run(whole_obs);
            auto infer_t1 = std::chrono::steady_clock::now();
            infer_ms = std::chrono::duration<double, std::milli>(infer_t1 - infer_t0).count();
            infer_success = true;

            // 更新 action (用于下一帧的 prev_action)
            action_component->SetAction(policy_out.raw_action);
        } catch (const std::exception& e) {
            std::cerr << "\n[Policy] ERROR: Inference failed: " << e.what() << std::endl;
            logger.LogEvent(std::string("ERROR: Inference failed: ") + e.what());
            g_policy_error = true;
            infer_success = false;
            // Shadow 模式推理失败，继续 hold 但不更新 action
        }

        // Hold 在 offset (不发送 policy 输出)
        bringup.HoldOffsets(motor_indices);

        // 计算诊断信息
        float max_abs_action = 0.0f;
        float max_desired_diff = 0.0f;
        int clamp_count = 0;
        for (int i = 0; i < cfg::kNumMotors; ++i) {
            max_abs_action = std::max(max_abs_action, std::fabs(policy_out.raw_action[i]));
            if (std::fabs(policy_out.clipped_abs[i] - policy_out.desired_abs[i]) > 0.001f) {
                clamp_count++;
            }
        }

        // 记录帧数据
        FrameRecord rec;
        rec.tick = tick++;
        rec.t_sec = std::chrono::duration<double>(loop_start - t0).count();
        rec.loop_dt_ms = std::chrono::duration<double, std::milli>(loop_start - last).count();
        rec.infer_ms = infer_ms;
        rec.history_valid = static_cast<int>(obs.ValidFrames());
        rec.history_full = obs.IsHistoryFull() ? 1 : 0;
        rec.control_enabled = 0;  // Shadow 模式不发送 policy 输出
        rec.single_obs = single_obs;
        rec.raw_action = policy_out.raw_action;
        rec.desired_abs = policy_out.desired_abs;
        rec.clipped_abs = policy_out.clipped_abs;

        controller->GetAllMotorStates(motor_indices,
                                      rec.motor_pos_abs,
                                      rec.motor_vel_abs,
                                      rec.motor_torque);

        // 计算最大偏差（需要先获取电机位置）
        for (int i = 0; i < cfg::kNumMotors; ++i) {
            float diff = std::fabs(policy_out.desired_abs[i] - rec.motor_pos_abs[i]);
            max_desired_diff = std::max(max_desired_diff, diff);
        }
        rec.max_track_err = max_desired_diff;
        rec.clamp_count = clamp_count;
        rec.imu_fresh = imu_component->IsFresh(100) ? 1 : 0;
        rec.motors_fresh = controller->AllMotorsOnlineFresh(motor_indices, 100) ? 1 : 0;

        logger.LogFrame(rec);

        // 打印状态 (每 10 帧)
        if (tick % 10 == 0) {
            auto cmd = input_source->GetCommand();
            std::cout << "\r[t=" << std::fixed << std::setprecision(1) << rec.t_sec
                      << "s] infer: " << std::setprecision(1) << infer_ms << "ms"
                      << " | max_act: " << std::setprecision(2) << max_abs_action
                      << " | max_diff: " << max_desired_diff
                      << " | clamp: " << clamp_count
                      << " | hist: " << rec.history_valid << "/" << cfg::kHistoryLen
                      << " | imu: " << (rec.imu_fresh ? "OK" : "!!")
                      << " | motors: " << (rec.motors_fresh ? "OK" : "!!")
                      << std::flush;
        }

        last = loop_start;

        // 精确控制频率
        auto next_time = loop_start + std::chrono::milliseconds(1000 / cfg::kControlHz);
        std::this_thread::sleep_until(next_time);
    }

    logger.LogEvent("Shadow loop ended");
    logger.Close();

    std::cout << std::endl << "=== Shadow Complete ===" << std::endl;
    std::cout << "Log saved to: " << log_dir << std::endl;
    if (g_policy_error) {
        std::cout << "WARNING: Policy errors occurred during run" << std::endl;
    }
    if (g_motor_offline) {
        std::cout << "WARNING: Motor offline events occurred during run" << std::endl;
    }

    return 0;
}
/**
 * @file policy_test_main.cpp
 * @brief 主入口：支持 hold-only/shadow/closed_loop 三种模式
 *
 * 模式：
 * - hold-only: 只观测并 hold 在 offset，不推理
 * - shadow: 观测 + 推理 + 日志，不发控制
 * - run: 观测 + 推理 + 日志 + 发送控制 (闭环)
 *
 * 输入源：
 * - gamepad: 手柄控制
 * - keyboard: 键盘控制 (W/S: vx, A/D: vy, Q/E: yaw, Space: zero)
 * - auto: 自动选择（优先 gamepad，无则 keyboard）
 *
 * 流程：
 * 1. bringup (BindCAN -> BindMotor -> Enable -> AutoReport -> WaitFeedback -> RampToOffset)
 * 2. prewarm 10 帧有效观测（IMU 和电机都就绪，command 强制为 0）
 * 3. shadow / closed_loop
 *
 * 安全机制：
 * - 运行期检查电机 online/fresh，不通过就退回 HoldOnly
 * - Policy 加载/推理失败时安全降级
 * - prewarm 只计入有效帧，command 强制归零
 * - 键盘输入有 deadman timeout (200ms)
 */

#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>
#include <cstring>
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
std::atomic<bool> g_send_failed{false};

void signal_handler(int sig) {
    // 只置位，不做 I/O（async-signal-safe）
    g_running = false;
}

enum class RunMode {
    HoldOnly,       // 只 hold 在 offset，不推理
    ShadowPolicy,
    ClosedLoop
};

void PrintUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [mode] [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "  mode:" << std::endl;
    std::cout << "    hold     - Observe and hold at offset, no inference (safest)" << std::endl;
    std::cout << "    shadow   - Observe + inference + logging, NO control (default)" << std::endl;
    std::cout << "    run      - Observe + inference + logging + control (closed-loop)" << std::endl;
    std::cout << std::endl;
    std::cout << "  options:" << std::endl;
    std::cout << "    --input <source>  - Input source: gamepad, keyboard, auto (default: auto)" << std::endl;
    std::cout << "    --low-gain        - Use low gain mode for ground testing" << std::endl;
    std::cout << std::endl;
    std::cout << "  keyboard controls:" << std::endl;
    std::cout << "    W/S: forward/backward (vx)" << std::endl;
    std::cout << "    A/D: left/right (vy, disabled by default)" << std::endl;
    std::cout << "    Q/E: turn left/right (yaw)" << std::endl;
    std::cout << "    Space/R: command zero" << std::endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    // 解析参数
    RunMode mode = RunMode::ShadowPolicy;
    std::string input_mode = "auto";
    bool low_gain = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "hold" || arg == "hold-only") {
            mode = RunMode::HoldOnly;
        } else if (arg == "shadow") {
            mode = RunMode::ShadowPolicy;
        } else if (arg == "run") {
            mode = RunMode::ClosedLoop;
        } else if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        } else if (arg == "--input" && i + 1 < argc) {
            input_mode = argv[++i];
            if (input_mode != "gamepad" && input_mode != "keyboard" && input_mode != "auto") {
                std::cerr << "Unknown input mode: " << input_mode << std::endl;
                PrintUsage(argv[0]);
                return 1;
            }
        } else if (arg == "--low-gain") {
            low_gain = true;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    std::cout << "=== Policy Test Main ===" << std::endl;
    std::cout << "Mode: ";
    switch (mode) {
        case RunMode::HoldOnly: std::cout << "hold-only (observe and hold at offset, no inference)"; break;
        case RunMode::ShadowPolicy: std::cout << "shadow (inference + logging, NO control)"; break;
        case RunMode::ClosedLoop: std::cout << "run (closed-loop control)"; break;
    }
    if (low_gain) {
        std::cout << " [LOW-GAIN]";
    }
    std::cout << std::endl;
    std::cout << "Input: " << input_mode << std::endl;
    std::cout << std::endl;

    // 创建日志
    std::string log_dir = cfg::kLogDir + "/session_" + std::to_string(
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));

    SessionLogger logger;
    if (logger.Open(log_dir)) {
        std::string mode_str;
        switch (mode) {
            case RunMode::HoldOnly: mode_str = "hold_only"; break;
            case RunMode::ShadowPolicy: mode_str = "shadow"; break;
            case RunMode::ClosedLoop: mode_str = "closed_loop"; break;
        }
        logger.LogMeta("{\"mode\": \"" + mode_str + "\", \"input\": \"" + input_mode +
                       "\", \"policy_input_dim\": 450, \"history_len\": 10, \"control_hz\": 50}");
    }

    // ========== Step 1: Bringup ==========
    logger.LogEvent("Creating controller");
    auto controller = std::make_shared<RobstrideController>();
    RobotBringup bringup(controller);

    logger.LogEvent("Binding CAN interfaces");
    if (!bringup.BindAllCAN()) {
        logger.LogEvent("ERROR: BindAllCAN failed");
        return 1;
    }

    logger.LogEvent("Binding motors");
    std::vector<int> motor_indices;
    if (!bringup.BindAllMotors(motor_indices)) {
        logger.LogEvent("ERROR: BindAllMotors failed");
        return 1;
    }

    logger.LogEvent("Enabling motors");
    if (!bringup.EnableAll(motor_indices, low_gain)) {
        logger.LogEvent("ERROR: EnableAll failed");
        return 1;
    }

    logger.LogEvent("Enabling auto report");
    if (!bringup.EnableAutoReportAll(motor_indices)) {
        logger.LogEvent("ERROR: EnableAutoReportAll failed");
        return 1;
    }

    logger.LogEvent("Waiting for feedback");
    if (!bringup.WaitForFeedback(motor_indices, 3000)) {
        logger.LogEvent("ERROR: WaitForFeedback timeout");
        return 1;
    }

    logger.LogEvent("Moving to offset pose");
    if (!bringup.MoveCurrentToOffsets(motor_indices)) {
        logger.LogEvent("ERROR: MoveCurrentToOffsets failed");
        return 1;
    }

    // ========== Step 2: 创建观测组件 ==========
    logger.LogEvent("Creating observation components");
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

    // 创建 Policy Runner (只在 shadow 和 run 模式下使用)
    std::unique_ptr<PolicyRunner> policy;
    if (mode != RunMode::HoldOnly) {
        logger.LogEvent("Loading policy engine");
        try {
            policy = std::make_unique<PolicyRunner>(cfg::kPolicyEnginePath);
            std::cout << "[Policy] Engine loaded successfully" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Policy] ERROR: Failed to load policy engine: " << e.what() << std::endl;
            logger.LogEvent(std::string("ERROR: Policy load failed: ") + e.what());
            g_policy_error = true;

            // 如果是 run 模式，policy 加载失败是致命错误
            if (mode == RunMode::ClosedLoop) {
                std::cerr << "[Policy] Cannot run closed-loop without policy. Exiting." << std::endl;
                return 1;
            }
            // shadow 模式可以降级为 hold-only
            std::cout << "[Policy] Downgrading to hold-only mode" << std::endl;
            mode = RunMode::HoldOnly;
        }
    }

    // 初始化 action 为 0
    std::vector<float> zero_action(12, 0.0f);
    action_component->SetAction(zero_action);

    // ========== Step 3: 预热 history（只计入有效帧，command 强制为 0）==========
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
        if (!bringup.HoldOffsets(motor_indices)) {
            std::cerr << "\n[Prewarm] WARNING: SafeHold send failed" << std::endl;
        }

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
    std::cout << "[Ready] History is full with valid observations. Starting main loop..." << std::endl;
    std::cout << std::endl;

    // ========== Step 4: 主循环 ==========
    logger.LogEvent("Starting main loop");

    // 闭环软切入参数
    double run_enter_t = -1.0;              // 进入闭环的时间点
    const double kBlendInTime = 1.0;        // 渐入时间 1 秒
    const float kMaxStepPerTick = 0.03f;    // 每拍最大变化 0.03 rad
    const float kMaxTrackErr = 0.6f;        // 最大跟踪误差 0.6 rad

    std::vector<float> last_cmd = cfg::kJointOffsets;  // 上一拍命令，初始化为 offset

    uint64_t tick = 0;
    auto t0 = std::chrono::steady_clock::now();
    auto last = t0;

    while (g_running) {
        auto loop_start = std::chrono::steady_clock::now();

        // ========== 运行期安全检查：IMU 和电机是否都就绪且新鲜 ==========
        if (!CheckSensorsReady(imu_component, controller, motor_indices, 100, 100)) {
            // 传感器不就绪，立即退回 hold 模式
            if (mode == RunMode::ClosedLoop || mode == RunMode::ShadowPolicy) {
                std::cerr << "\n[Safety] Sensor not ready/fresh (IMU or motor), entering hold mode!" << std::endl;
                logger.LogEvent("SAFETY: Sensor not ready/fresh, entering hold mode");
                g_motor_offline = true;
                mode = RunMode::HoldOnly;
            }
            // 即使在 hold 模式也要继续检查，但只发 hold 命令
            if (!bringup.HoldOffsets(motor_indices)) {
                g_send_failed = true;
            }
            last = loop_start;
            auto next_time = loop_start + std::chrono::milliseconds(1000 / cfg::kControlHz);
            std::this_thread::sleep_until(next_time);
            continue;
        }

        // 更新观测
        obs.UpdateObs();
        auto single_obs = obs.GetSingleObs();
        auto whole_obs = obs.GetWholeObs();

        // 推理 (只在 shadow 和 run 模式)
        PolicyOutput policy_out;
        double infer_ms = 0.0;
        bool infer_success = false;

        if (mode != RunMode::HoldOnly && policy) {
            try {
                auto infer_t0 = std::chrono::steady_clock::now();
                policy_out = policy->Run(whole_obs);
                auto infer_t1 = std::chrono::steady_clock::now();
                infer_ms = std::chrono::duration<double, std::milli>(infer_t1 - infer_t0).count();
                infer_success = true;

                // 更新 action
                action_component->SetAction(policy_out.raw_action);
            } catch (const std::exception& e) {
                std::cerr << "\n[Policy] ERROR: Inference failed: " << e.what() << std::endl;
                logger.LogEvent(std::string("ERROR: Inference failed: ") + e.what());
                g_policy_error = true;
                infer_success = false;

                // 推理失败时降级为 hold-only
                if (mode == RunMode::ClosedLoop) {
                    std::cout << "[Policy] Entering safe hold mode" << std::endl;
                    mode = RunMode::HoldOnly;
                }
            }
        }

        // 发送控制 (只在 run 模式且推理成功)
        bool control_enabled = false;
        bool send_ok = true;
        std::vector<float> cmd_sent_abs(cfg::kNumMotors, 0.0f);
        std::vector<float> cmd_minus_pos(cfg::kNumMotors, 0.0f);
        std::vector<float> cmd_delta(cfg::kNumMotors, 0.0f);
        float max_track_err = 0.0f;
        int clamp_count = 0;

        if (mode == RunMode::ClosedLoop && infer_success) {
            control_enabled = true;

            // 计算渐入系数
            double t_sec = std::chrono::duration<double>(loop_start - t0).count();
            if (run_enter_t < 0.0) {
                run_enter_t = t_sec;
                std::cout << "\n[Control] Entering closed-loop at t=" << t_sec << "s" << std::endl;
                logger.LogEvent("Entering closed-loop control");
            }
            double alpha = std::clamp((t_sec - run_enter_t) / kBlendInTime, 0.0, 1.0);

            for (int i = 0; i < cfg::kNumMotors; ++i) {
                // 1. 渐入：从 offset 平滑过渡到 policy 输出
                float target = cfg::kJointOffsets[i] +
                               alpha * (policy_out.clipped_abs[i] - cfg::kJointOffsets[i]);

                // 2. 每拍命令限幅
                target = std::clamp(target, last_cmd[i] - kMaxStepPerTick, last_cmd[i] + kMaxStepPerTick);

                // 3. 跟踪误差保护
                auto st = controller->GetMotorState(motor_indices[i]);
                float err = std::fabs(target - st.position);
                if (err > max_track_err) max_track_err = err;

                if (err > kMaxTrackErr) {
                    std::cerr << "\n[Safety] Tracking error too large for motor " << i
                              << ": err=" << err << " > " << kMaxTrackErr << std::endl;
                    logger.LogEvent("SAFETY: tracking error too large, entering hold");
                    bringup.EmergencyHoldAll(motor_indices, 3);
                    mode = RunMode::HoldOnly;
                    control_enabled = false;
                    send_ok = false;
                    g_send_failed = true;
                    break;
                }

                // 发送命令
                int ret = controller->SendMITCommand(motor_indices[i], target);
                if (ret != 0) {
                    std::cerr << "\n[CAN] ERROR: SendMITCommand failed for motor " << i << std::endl;
                    g_send_failed = true;
                    logger.LogEvent("ERROR: CAN send failed, emergency hold");
                    bringup.EmergencyHoldAll(motor_indices, 3);
                    mode = RunMode::HoldOnly;
                    control_enabled = false;
                    send_ok = false;
                    break;
                }

                // 记录诊断数据
                cmd_sent_abs[i] = target;
                cmd_minus_pos[i] = target - st.position;
                cmd_delta[i] = target - last_cmd[i];
                last_cmd[i] = target;

                // 统计 clamp 次数
                if (std::fabs(policy_out.clipped_abs[i] - policy_out.desired_abs[i]) > 0.001f) {
                    clamp_count++;
                }
            }
        } else {
            // hold-only / shadow 模式 / 推理失败时都 hold 在 offset
            if (!bringup.HoldOffsets(motor_indices)) {
                g_send_failed = true;
                std::cerr << "\n[CAN] ERROR: SafeHold send failed" << std::endl;
            }
            // 非 run 模式重置渐入时间
            run_enter_t = -1.0;
        }

        // 记录帧数据
        FrameRecord rec;
        rec.tick = tick++;
        rec.t_sec = std::chrono::duration<double>(loop_start - t0).count();
        rec.loop_dt_ms = std::chrono::duration<double, std::milli>(loop_start - last).count();
        rec.infer_ms = infer_ms;
        rec.history_valid = static_cast<int>(obs.ValidFrames());
        rec.history_full = obs.IsHistoryFull() ? 1 : 0;
        rec.control_enabled = control_enabled ? 1 : 0;
        rec.single_obs = single_obs;
        rec.raw_action = policy_out.raw_action;
        rec.desired_abs = policy_out.desired_abs;
        rec.clipped_abs = policy_out.clipped_abs;

        // 新增：执行层诊断字段
        rec.cmd_sent_abs = cmd_sent_abs;
        rec.cmd_minus_pos = cmd_minus_pos;
        rec.cmd_delta = cmd_delta;
        rec.max_track_err = max_track_err;
        rec.clamp_count = clamp_count;
        rec.imu_fresh = imu_component->IsFresh(100) ? 1 : 0;
        rec.motors_fresh = controller->AllMotorsOnlineFresh(motor_indices, 100) ? 1 : 0;

        controller->GetAllMotorStates(motor_indices,
                                       rec.motor_pos_abs,
                                       rec.motor_vel_abs,
                                       rec.motor_torque);
        logger.LogFrame(rec);

        // 打印状态 (每 10 帧)
        if (tick % 10 == 0) {
            auto cmd = input_source->GetCommand();
            std::cout << "\r[t=" << std::fixed << std::setprecision(1) << rec.t_sec
                      << "s] dt: " << std::setprecision(1) << rec.loop_dt_ms << "ms"
                      << " | infer: " << infer_ms << "ms"
                      << " | ctrl: " << (control_enabled ? "ON" : "HOLD")
                      << " | cmd: [" << std::setprecision(2) << cmd[0] << "," << cmd[1] << "," << cmd[2] << "]"
                      << " | max_err: " << std::setprecision(3) << max_track_err
                      << " | clamp: " << clamp_count
                      << " | imu: " << (rec.imu_fresh ? "OK" : "!!")
                      << " | motors: " << (rec.motors_fresh ? "OK" : "!!")
                      << std::flush;
        }

        last = loop_start;

        // 精确控制频率
        auto next_time = loop_start + std::chrono::milliseconds(1000 / cfg::kControlHz);
        std::this_thread::sleep_until(next_time);
    }

    // ========== 清理 ==========
    logger.LogEvent("Main loop ended");

    // 安全停止：回到 offset 并保持
    std::cout << std::endl << "[Shutdown] Moving to safe position..." << std::endl;
    for (int i = 0; i < 100; ++i) {
        if (!bringup.HoldOffsets(motor_indices)) {
            std::cerr << "[Shutdown] WARNING: SafeHold send failed at iteration " << i << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    logger.Close();

    std::cout << "=== Complete ===" << std::endl;
    std::cout << "Log saved to: " << log_dir << std::endl;
    std::cout << "Total frames: " << tick << std::endl;
    if (g_policy_error) {
        std::cout << "WARNING: Policy errors occurred during run" << std::endl;
    }
    if (g_motor_offline) {
        std::cout << "WARNING: Motor offline events occurred during run" << std::endl;
    }
    if (g_send_failed) {
        std::cout << "WARNING: CAN send failures occurred during run" << std::endl;
    }

    return 0;
}
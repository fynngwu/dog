#pragma once
#include "input_source.hpp"
#include <atomic>
#include <array>
#include <chrono>
#include <mutex>
#include <thread>
#include <termios.h>

/**
 * @brief 键盘命令输入源
 *
 * 非阻塞、raw mode、单独线程读取键盘输入。
 *
 * 按键映射：
 * - W/S: 前进/后退 (vx)
 * - A/D: 左移/右移 (vy，默认关闭)
 * - Q/E: 左转/右转 (yaw_rate)
 * - Space/R: 命令归零
 *
 * 安全机制：
 * - deadman timeout: 200ms 无按键自动归零
 */
class KeyboardCommandSource : public ICommandSource {
public:
    /**
     * @brief 构造函数
     * @param vx_scale vx 缩放因子，默认 0.5
     * @param vy_scale vy 缩放因子，默认 0（关闭横移）
     * @param yaw_scale yaw 缩放因子，默认 0.5
     * @param deadman_ms deadman 超时时间，默认 200ms
     */
    KeyboardCommandSource(float vx_scale = 0.5f,
                          float vy_scale = 0.0f,
                          float yaw_scale = 0.5f,
                          int deadman_ms = 200);

    ~KeyboardCommandSource() override;

    void Update() override {}  // 线程驱动，无需主循环调用
    std::vector<float> GetCommand() const override;
    bool IsReady() const override { return ready_; }
    const char* Name() const override { return "keyboard"; }

private:
    void ReadLoop();
    void EnterRawMode();
    void RestoreTerminal();

private:
    std::atomic<bool> running_{true};
    std::atomic<bool> ready_{false};
    std::thread thread_;

    mutable std::mutex mutex_;
    std::array<float, 3> cmd_{0.0f, 0.0f, 0.0f};

    float vx_scale_;
    float vy_scale_;
    float yaw_scale_;
    int deadman_ms_;

    std::chrono::steady_clock::time_point last_key_tp_;
    int stdin_flags_ = 0;
    struct termios old_termios_{};
    bool raw_mode_enabled_ = false;
};
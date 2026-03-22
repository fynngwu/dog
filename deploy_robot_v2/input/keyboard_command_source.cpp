#include "keyboard_command_source.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

KeyboardCommandSource::KeyboardCommandSource(float vx_scale,
                                             float vy_scale,
                                             float yaw_scale,
                                             int deadman_ms)
    : vx_scale_(vx_scale)
    , vy_scale_(vy_scale)
    , yaw_scale_(yaw_scale)
    , deadman_ms_(deadman_ms) {
    EnterRawMode();
    last_key_tp_ = std::chrono::steady_clock::now();
    ready_ = raw_mode_enabled_;

    if (ready_) {
        thread_ = std::thread(&KeyboardCommandSource::ReadLoop, this);
        std::cout << "[Keyboard] Ready. W/S: vx, A/D: vy, Q/E: yaw, Space: zero" << std::endl;
    } else {
        std::cerr << "[Keyboard] Failed to enter raw mode (not a tty?)" << std::endl;
    }
}

KeyboardCommandSource::~KeyboardCommandSource() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    RestoreTerminal();
}

void KeyboardCommandSource::EnterRawMode() {
    if (!isatty(STDIN_FILENO)) {
        return;
    }
    if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
        return;
    }

    struct termios raw = old_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);  // 禁用规范模式和回显
    raw.c_cc[VMIN] = 0;   // 非阻塞读取
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        return;
    }

    stdin_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, stdin_flags_ | O_NONBLOCK);
    raw_mode_enabled_ = true;
}

void KeyboardCommandSource::RestoreTerminal() {
    if (!raw_mode_enabled_) {
        return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    fcntl(STDIN_FILENO, F_SETFL, stdin_flags_);
}

void KeyboardCommandSource::ReadLoop() {
    while (running_) {
        char c;
        bool got_key = false;

        // 读取所有待处理的字符
        while (read(STDIN_FILENO, &c, 1) > 0) {
            got_key = true;
            std::lock_guard<std::mutex> lock(mutex_);

            switch (c) {
                case 'w': case 'W':
                    cmd_[0] = +vx_scale_;  // 前进
                    break;
                case 's': case 'S':
                    cmd_[0] = -vx_scale_;  // 后退
                    break;
                case 'a': case 'A':
                    cmd_[1] = +vy_scale_;  // 左移（若 vy_scale_=0 则无效）
                    break;
                case 'd': case 'D':
                    cmd_[1] = -vy_scale_;  // 右移
                    break;
                case 'q': case 'Q':
                    cmd_[2] = +yaw_scale_;  // 左转
                    break;
                case 'e': case 'E':
                    cmd_[2] = -yaw_scale_;  // 右转
                    break;
                case ' ':  // Space: 立即归零
                case 'r': case 'R':
                    cmd_ = {0.0f, 0.0f, 0.0f};
                    break;
                default:
                    break;
            }
            last_key_tp_ = std::chrono::steady_clock::now();
        }

        // Deadman timeout: 超时自动归零
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto now = std::chrono::steady_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_key_tp_).count();
            if (ms > deadman_ms_) {
                cmd_ = {0.0f, 0.0f, 0.0f};
            }
        }

        usleep(5000);  // 5ms 轮询间隔
    }
}

std::vector<float> KeyboardCommandSource::GetCommand() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {cmd_[0], cmd_[1], cmd_[2]};
}
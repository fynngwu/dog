#include "twin_agent.hpp"
#include "twin_protocol.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>

namespace twin {
namespace {

bool SendAll(int fd, const std::string& data) {
    size_t sent = 0;
    while (sent < data.size()) {
        const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, 0);
        if (n <= 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

void SetSocketTimeoutMs(int fd, int send_timeout_ms) {
    timeval tv{};
    tv.tv_sec = send_timeout_ms / 1000;
    tv.tv_usec = (send_timeout_ms % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

}  // namespace

TwinAgent::TwinAgent(int cmd_port, int state_port)
    : cmd_port_(cmd_port), state_port_(state_port),
      last_joint_targets_(minimal::kNumMotors, 0.0f) {}

TwinAgent::~TwinAgent() {
    Stop();
}

bool TwinAgent::SetupServerSocket(int port, int& out_fd, std::string& err) {
    out_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (out_fd < 0) {
        err = std::string("创建 socket 失败: ") + std::strerror(errno);
        return false;
    }

    const int opt = 1;
    ::setsockopt(out_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(out_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        err = std::string("bind 失败: ") + std::strerror(errno);
        ::close(out_fd);
        out_fd = -1;
        return false;
    }
    if (::listen(out_fd, 8) < 0) {
        err = std::string("listen 失败: ") + std::strerror(errno);
        ::close(out_fd);
        out_fd = -1;
        return false;
    }
    return true;
}

bool TwinAgent::Start() {
    if (!motor_io_.Initialize()) {
        std::cerr << "[TwinAgent] MotorIO 初始化失败" << std::endl;
        return false;
    }

    std::string err;
    if (!SetupServerSocket(cmd_port_, cmd_server_fd_, err)) {
        std::cerr << "[TwinAgent] 命令端口启动失败: " << err << std::endl;
        return false;
    }
    if (!SetupServerSocket(state_port_, state_server_fd_, err)) {
        std::cerr << "[TwinAgent] 状态端口启动失败: " << err << std::endl;
        ::close(cmd_server_fd_);
        cmd_server_fd_ = -1;
        return false;
    }

    running_ = true;
    cmd_thread_ = std::thread(&TwinAgent::CommandLoop, this);
    state_thread_ = std::thread(&TwinAgent::StatePublishLoop, this);

    std::cout << "[TwinAgent] 启动成功" << std::endl;
    std::cout << "  命令端口: " << cmd_port_ << std::endl;
    std::cout << "  状态端口: " << state_port_ << std::endl;
    std::cout << "  支持命令: ping / get_state / enable / disable / init [sec] / hold / set_joint <12 floats>" << std::endl;
    return true;
}

void TwinAgent::CloseAllStateClients() {
    std::lock_guard<std::mutex> lk(state_clients_mutex_);
    for (int fd : state_clients_) {
        ::close(fd);
    }
    state_clients_.clear();
}

void TwinAgent::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;

    if (cmd_server_fd_ >= 0) {
        ::close(cmd_server_fd_);
        cmd_server_fd_ = -1;
    }
    if (state_server_fd_ >= 0) {
        ::close(state_server_fd_);
        state_server_fd_ = -1;
    }

    CloseAllStateClients();

    if (cmd_thread_.joinable()) cmd_thread_.join();
    if (state_thread_.joinable()) state_thread_.join();

    std::lock_guard<std::mutex> lk(control_mutex_);
    if (enabled_) {
        motor_io_.DisableAll();
        enabled_ = false;
    }

    std::cout << "[TwinAgent] 已停止" << std::endl;
}

void TwinAgent::CommandLoop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        const int client_fd = ::accept(cmd_server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0) {
            if (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            continue;
        }
        HandleCommand(client_fd);
        ::close(client_fd);
    }
}

void TwinAgent::HandleCommand(int client_fd) {
    char buf[4096];
    std::string buffer;

    while (running_) {
        const ssize_t n = ::recv(client_fd, buf, sizeof(buf), 0);
        if (n <= 0) {
            break;
        }
        buffer.append(buf, static_cast<size_t>(n));

        while (true) {
            const size_t pos = buffer.find('\n');
            if (pos == std::string::npos) {
                break;
            }
            const std::string line = Trim(buffer.substr(0, pos));
            buffer.erase(0, pos + 1);
            if (line.empty()) {
                continue;
            }

            const std::string reply = ProcessCommand(line);
            if (!SendAll(client_fd, reply)) {
                return;
            }
        }
    }
}

bool TwinAgent::InitToOffset(float duration_sec, std::string& err) {
    std::lock_guard<std::mutex> lk(control_mutex_);

    if (!enabled_) {
        if (!motor_io_.EnableAll()) {
            err = "使能失败";
            return false;
        }
    }

    if (!motor_io_.MoveToOffset(duration_sec)) {
        err = "缓慢回 offset 失败";
        return false;
    }

    if (!motor_io_.HoldOffsets()) {
        err = "到达 offset 后保持失败";
        return false;
    }

    // Re-enable continuous auto-report so motors stay online while idle
    motor_io_.EnableAllAutoReport();

    std::fill(last_joint_targets_.begin(), last_joint_targets_.end(), 0.0f);
    enabled_ = true;
    return true;
}

bool TwinAgent::SetJointTargets(const std::vector<float>& joint_targets, std::string& err) {
    if (joint_targets.size() != static_cast<size_t>(minimal::kNumMotors)) {
        err = "joint target size mismatch";
        return false;
    }

    std::lock_guard<std::mutex> lk(control_mutex_);
    if (!enabled_) {
        err = "电机未使能，请先执行 init 或 enable";
        return false;
    }

    if (!motor_io_.SendActions(joint_targets, 1.0f)) {
        err = "set_joint 下发失败";
        return false;
    }

    last_joint_targets_ = joint_targets;
    return true;
}

std::string TwinAgent::ProcessCommand(const std::string& cmd) {
    const auto tokens = SplitWS(cmd);
    if (tokens.empty()) {
        return ErrorReply("空命令");
    }

    const std::string& op = tokens[0];

    if (op == "ping") {
        return OkReply("pong");
    }
    if (op == "get_state") {
        return SnapshotToJson();
    }

    if (op == "enable") {
        std::string err;
        {
            std::lock_guard<std::mutex> lk(control_mutex_);
            if (!motor_io_.EnableAll()) {
                return ErrorReply("使能失败");
            }
            enabled_ = true;
        }
        return OkReply("已使能");
    }

    if (op == "disable") {
        {
            std::lock_guard<std::mutex> lk(control_mutex_);
            if (!motor_io_.DisableAll()) {
                return ErrorReply("去使能失败");
            }
            enabled_ = false;
            std::fill(last_joint_targets_.begin(), last_joint_targets_.end(), 0.0f);
        }
        return OkReply("已去使能");
    }

    if (op == "init" || op == "goto_offset" || op == "zero") {
        float duration_sec = 2.5f;
        if (tokens.size() >= 2) {
            try {
                duration_sec = std::stof(tokens[1]);
            } catch (const std::exception& e) {
                return ErrorReply(std::string("无效的 duration: ") + e.what());
            }
        }
        if (duration_sec <= 0.0f || duration_sec > 30.0f) {
            return ErrorReply("duration 需在 (0, 30] 秒范围内");
        }
        std::string err;
        if (!InitToOffset(duration_sec, err)) {
            return ErrorReply(err);
        }
        return OkReply("已缓慢到 offset 并保持");
    }

    if (op == "hold") {
        std::lock_guard<std::mutex> lk(control_mutex_);
        if (!motor_io_.HoldOffsets()) {
            return ErrorReply("保持失败");
        }
        std::fill(last_joint_targets_.begin(), last_joint_targets_.end(), 0.0f);
        return OkReply("保持 offset");
    }

    if (op == "set_joint" || op == "set_action") {
        std::vector<float> targets;
        std::string err;
        if (!ParseFloatList(tokens, 1, minimal::kNumMotors, targets, err)) {
            return ErrorReply(err);
        }
        if (!SetJointTargets(targets, err)) {
            return ErrorReply(err);
        }
        return OkReply("joint target 已更新");
    }

    return ErrorReply("未知命令: " + op);
}

void TwinAgent::StatePublishLoop() {
    while (running_) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(state_server_fd_, &read_fds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 20000;
        const int ret = ::select(state_server_fd_ + 1, &read_fds, nullptr, nullptr, &tv);

        if (ret > 0 && FD_ISSET(state_server_fd_, &read_fds)) {
            sockaddr_in client_addr{};
            socklen_t addr_len = sizeof(client_addr);
            const int client_fd = ::accept(state_server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
            if (client_fd >= 0) {
                SetSocketTimeoutMs(client_fd, 20);
                std::lock_guard<std::mutex> lk(state_clients_mutex_);
                state_clients_.push_back(client_fd);
            }
        }

        std::vector<int> clients_copy;
        {
            std::lock_guard<std::mutex> lk(state_clients_mutex_);
            clients_copy = state_clients_;
        }
        if (clients_copy.empty()) {
            continue;
        }

        const std::string json = SnapshotToJson();

        std::vector<int> dead_fds;
        for (int fd : clients_copy) {
            if (!SendAll(fd, json)) {
                dead_fds.push_back(fd);
            }
        }

        if (!dead_fds.empty()) {
            std::lock_guard<std::mutex> lk(state_clients_mutex_);
            for (int dead_fd : dead_fds) {
                auto it = std::find(state_clients_.begin(), state_clients_.end(), dead_fd);
                if (it != state_clients_.end()) {
                    ::close(*it);
                    state_clients_.erase(it);
                }
            }
        }
    }
}

std::string TwinAgent::SnapshotToJson() {
    const uint64_t seq = seq_.fetch_add(1) + 1;

    std::vector<float> motor_pos;
    std::vector<float> motor_vel;
    std::vector<float> motor_tau;
    std::vector<float> joint_obs;
    std::vector<float> target_joint_pos;
    int online = 0;
    int max_age_ms = -1;

    {
        std::lock_guard<std::mutex> lk(control_mutex_);
        motor_io_.GetMotorStates(motor_pos, motor_vel, motor_tau);
        joint_obs = motor_io_.GetJointObs();
        online = motor_io_.CountOnlineMotors(100);
        max_age_ms = motor_io_.MaxFeedbackAgeMs();
        target_joint_pos = last_joint_targets_;
    }

    std::vector<float> joint_pos(minimal::kNumMotors, 0.0f);
    std::vector<float> joint_vel(minimal::kNumMotors, 0.0f);
    for (int i = 0; i < minimal::kNumMotors; ++i) {
        if (i < static_cast<int>(joint_obs.size())) {
            joint_pos[i] = joint_obs[i];
        }
        if (i + minimal::kNumMotors < static_cast<int>(joint_obs.size())) {
            joint_vel[i] = joint_obs[i + minimal::kNumMotors];
        }
    }

    std::ostringstream oss;
    oss << "{"
        << "\"ok\":true,"
        << "\"mode\":\"" << (enabled_ ? "enabled" : "disabled") << "\"," 
        << "\"seq\":" << seq << ","
        << "\"motors_online\":" << online << ","
        << "\"feedback_age_ms\":" << max_age_ms << ","
        << "\"joint_positions\":" << JsonArray(joint_pos) << ","
        << "\"joint_velocities\":" << JsonArray(joint_vel) << ","
        << "\"motor_positions\":" << JsonArray(motor_pos) << ","
        << "\"motor_velocities\":" << JsonArray(motor_vel) << ","
        << "\"motor_torques\":" << JsonArray(motor_tau) << ","
        << "\"target_joint_positions\":" << JsonArray(target_joint_pos)
        << "}\n";
    return oss.str();
}

}  // namespace twin

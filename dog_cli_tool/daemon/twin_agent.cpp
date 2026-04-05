#include "twin_agent.hpp"

#include "twin_protocol.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
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

std::vector<std::string> SplitCsvLine(const std::string& line) {
    std::vector<std::string> out;
    std::string current;
    bool in_quotes = false;
    for (char c : line) {
        if (c == '"') {
            in_quotes = !in_quotes;
            continue;
        }
        if (c == ',' && !in_quotes) {
            out.push_back(current);
            current.clear();
        } else {
            current.push_back(c);
        }
    }
    out.push_back(current);
    return out;
}

}  // namespace

TwinAgent::TwinAgent(int cmd_port)
    : cmd_port_(cmd_port), last_joint_targets_(minimal::kNumMotors, 0.0f) {
    motion_last_tick_ = std::chrono::steady_clock::now();
}

TwinAgent::~TwinAgent() {
    Stop();
}

void TwinAgent::SetFault(const MotorFault& f) {
    last_fault_ = f;
    LOG("ERROR", "TwinAgent", f.code.c_str(),
        ("joint=" + f.joint_name + " " + f.message).c_str());
}

bool TwinAgent::SetupServerSocket(int port, int& out_fd, std::string& err) {
    out_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (out_fd < 0) {
        err = std::string("socket failed: ") + std::strerror(errno);
        return false;
    }

    const int opt = 1;
    ::setsockopt(out_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(out_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        err = std::string("bind failed: ") + std::strerror(errno);
        ::close(out_fd);
        out_fd = -1;
        return false;
    }
    if (::listen(out_fd, 8) < 0) {
        err = std::string("listen failed: ") + std::strerror(errno);
        ::close(out_fd);
        out_fd = -1;
        return false;
    }
    return true;
}

bool TwinAgent::Start() {
    if (!motor_io_.Initialize()) {
        LOG("ERROR", "TwinAgent", "start", "MotorIO initialization failed");
        return false;
    }

    std::string err;
    if (!SetupServerSocket(cmd_port_, cmd_server_fd_, err)) {
        LOG("ERROR", "TwinAgent", "start", ("command port setup failed: " + err).c_str());
        return false;
    }

    running_ = true;
    cmd_thread_ = std::thread(&TwinAgent::CommandLoop, this);
    return true;
}

void TwinAgent::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    AbortMotion("shutdown");

    if (cmd_server_fd_ >= 0) {
        ::close(cmd_server_fd_);
        cmd_server_fd_ = -1;
    }

    if (cmd_thread_.joinable()) cmd_thread_.join();
    if (motion_thread_.joinable()) motion_thread_.join();

    std::lock_guard<std::mutex> lk(control_mutex_);
    if (enabled_) {
        motor_io_.DisableAll();
        enabled_ = false;
    }
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

bool TwinAgent::InitToOffset(float duration_sec, std::string& err,
                            std::vector<std::string>& offline_motors) {
    offline_motors.clear();
    std::lock_guard<std::mutex> lk(control_mutex_);
    if (!enabled_) {
        if (!motor_io_.EnableAll()) {
            SetFault(motor_io_.GetLastFault());
            err = "enable failed: " + last_fault_.message;
            return false;
        }
        motor_io_.EnableAllAutoReport();

        // Wait and check which motors came online, retry up to 2 times for offline ones
        for (int attempt = 0; attempt < 2; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            auto offline = motor_io_.GetOfflineJoints(500);
            if (offline.empty()) break;
            for (int gi = 0; gi < minimal::kNumMotors; ++gi) {
                for (const auto& name : offline) {
                    if (name == minimal::kJointNames[gi]) {
                        LOG("WARN", "TwinAgent", "init",
                            (std::string("retrying enable for ") + name).c_str());
                        motor_io_.EnableJoint(gi);
                        break;
                    }
                }
            }
        }

        // Final check: collect still-offline motors
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        offline_motors = motor_io_.GetOfflineJoints(500);
        if (!offline_motors.empty()) {
            std::string offline_str;
            for (const auto& name : offline_motors) offline_str += name + " ";
            LOG("WARN", "TwinAgent", "init",
                ("motors still offline after retries: " + offline_str).c_str());
        }
    }
    if (!motor_io_.MoveToOffset(duration_sec)) {
        err = "move_to_offset failed";
        return false;
    }
    // Hold at offset by sending zero relative targets
    std::vector<float> zeros(minimal::kNumMotors, 0.0f);
    if (!motor_io_.SendJointRelativeTargets(zeros)) {
        err = "hold offset failed";
        return false;
    }
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
        err = "motors are not enabled";
        return false;
    }
    if (!motor_io_.SendJointRelativeTargets(joint_targets)) {
        err = "set_joint send failed";
        return false;
    }
    last_joint_targets_ = joint_targets;
    return true;
}

bool TwinAgent::MotionBusy() const {
    return motion_active_.load();
}

bool TwinAgent::AbortMotion(const std::string& reason) {
    if (!motion_active_) {
        return true;
    }
    motion_abort_ = true;
    if (motion_thread_.joinable()) {
        motion_thread_.join();
    }
    motion_active_ = false;
    std::lock_guard<std::mutex> lk(motion_mutex_);
    last_motion_error_ = reason;
    active_motion_name_.clear();
    return true;
}

bool TwinAgent::LaunchMotionThread(const std::string& name,
                                   float duration_sec,
                                   const std::function<bool(double, std::string&)>& step_fn,
                                   const std::function<void()>& on_success,
                                   std::string& err) {
    if (duration_sec <= 0.0f) {
        err = "duration must be positive";
        return false;
    }
    if (MotionBusy()) {
        err = "another motion is already active";
        return false;
    }
    if (motion_thread_.joinable()) {
        motion_thread_.join();
    }

    motion_abort_ = false;
    motion_active_ = true;
    {
        std::lock_guard<std::mutex> lk(motion_mutex_);
        active_motion_name_ = name;
        last_motion_error_.clear();
    }
    motion_last_tick_ = std::chrono::steady_clock::now();

    motion_thread_ = std::thread([this, name, duration_sec, step_fn, on_success]() {
        const int sleep_ms = 10;
        const auto start = std::chrono::steady_clock::now();
        std::string local_err;
        bool ok = true;
        while (!motion_abort_) {
            const auto now = std::chrono::steady_clock::now();
            const double t = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
            if (t > duration_sec) {
                break;
            }
            if (std::chrono::duration_cast<std::chrono::seconds>(now - motion_last_tick_).count() > 5) {
                local_err = "watchdog timeout during active motion";
                ok = false;
                break;
            }
            if (!step_fn(t, local_err)) {
                ok = false;
                break;
            }
            motion_last_tick_ = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        if (motion_abort_) {
            ok = false;
            if (local_err.empty()) {
                local_err = "motion aborted";
            }
        }
        if (ok && on_success) {
            on_success();
        }
        {
            std::lock_guard<std::mutex> lk(motion_mutex_);
            active_motion_name_.clear();
            last_motion_error_ = local_err;
        }
        motion_active_ = false;
    });
    return true;
}

bool TwinAgent::LoadReplayCsv(const std::string& csv_path, std::vector<ReplaySample>& samples, std::string& err) const {
    std::ifstream ifs(csv_path);
    if (!ifs.good()) {
        err = "unable to open csv: " + csv_path;
        return false;
    }

    std::string header_line;
    if (!std::getline(ifs, header_line)) {
        err = "csv is empty";
        return false;
    }
    const auto headers = SplitCsvLine(header_line);
    int time_idx = -1;
    int sim_time_idx = -1;
    std::vector<int> action_idx(minimal::kNumMotors, -1);
    for (size_t i = 0; i < headers.size(); ++i) {
        const std::string key = Trim(headers[i]);
        if (key == "timestamp_ms") time_idx = static_cast<int>(i);
        if (key == "sim_time") sim_time_idx = static_cast<int>(i);
        for (int j = 0; j < minimal::kNumMotors; ++j) {
            if (key == ("scaled_action_" + std::to_string(j))) {
                action_idx[j] = static_cast<int>(i);
            }
        }
    }
    if (time_idx < 0 && sim_time_idx < 0) {
        err = "csv must contain timestamp_ms or sim_time";
        return false;
    }
    for (int j = 0; j < minimal::kNumMotors; ++j) {
        if (action_idx[j] < 0) {
            err = "missing scaled_action_" + std::to_string(j);
            return false;
        }
    }

    std::string line;
    while (std::getline(ifs, line)) {
        if (Trim(line).empty()) continue;
        const auto cols = SplitCsvLine(line);
        ReplaySample sample;
        try {
            if (time_idx >= 0 && time_idx < static_cast<int>(cols.size())) {
                sample.time_sec = std::stod(cols[time_idx]) / 1000.0;
            } else {
                sample.time_sec = std::stod(cols[sim_time_idx]);
            }
            sample.targets_rel.resize(minimal::kNumMotors, 0.0f);
            for (int j = 0; j < minimal::kNumMotors; ++j) {
                sample.targets_rel[j] = std::stof(cols[action_idx[j]]);
            }
        } catch (const std::exception& e) {
            err = std::string("csv parse failed: ") + e.what();
            return false;
        }
        samples.push_back(sample);
    }
    if (samples.empty()) {
        err = "csv contains no samples";
        return false;
    }
    return true;
}

bool TwinAgent::StartReplayMotion(const std::string& csv_path, float speed_factor, std::string& err, bool skip_feedback_check) {
    if (speed_factor < 0.1f || speed_factor > 5.0f) {
        err = "speed_factor must be within [0.1, 5.0]";
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(control_mutex_);
        if (!enabled_) {
            err = "motors are not enabled";
            return false;
        }
    }
    std::vector<ReplaySample> samples;
    if (!LoadReplayCsv(csv_path, samples, err)) {
        return false;
    }
    const float duration_sec = static_cast<float>((samples.back().time_sec - samples.front().time_sec) / speed_factor + 0.05);
    return LaunchMotionThread(
        "replay", duration_sec,
        [this, samples, speed_factor, skip_feedback_check](double t, std::string& step_err) {
            // Check feedback freshness before each replay frame (optional)
            if (!skip_feedback_check) {
                MotorFault fault = motor_io_.CheckFeedbackFresh(500);
                if (fault.has_fault) {
                    SetFault(fault);
                    step_err = "replay aborted: " + fault.joint_name + " feedback lost age="
                               + std::to_string(fault.feedback_age_ms) + "ms";
                    return false;
                }
            }
            const double replay_time = samples.front().time_sec + t * speed_factor;
            size_t idx = 0;
            while (idx + 1 < samples.size() && samples[idx + 1].time_sec <= replay_time) {
                ++idx;
            }
            std::vector<float> targets = samples[idx].targets_rel;
            for (int j = 0; j < minimal::kNumMotors; ++j) {
                targets[j] = motor_io_.ClampRelativeTargetRad(j, targets[j]);
            }
            if (!motor_io_.SendJointRelativeTargets(targets)) {
                SetFault(motor_io_.GetLastFault());
                step_err = "failed to send replay frame: " + motor_io_.GetLastFault().message;
                return false;
            }
            last_joint_targets_ = targets;
            return true;
        },
        [this, samples]() { last_joint_targets_ = samples.back().targets_rel; }, err);
}

std::string TwinAgent::ProcessCommand(const std::string& cmd) {
    const auto tokens = SplitWS(cmd);
    if (tokens.empty()) {
        return ErrorReply("empty command", "bad_command");
    }
    const std::string& op = tokens[0];

    if (op == "ping") {
        return OkReply("pong");
    }
    if (op == "get_state") {
        return SnapshotToJson();
    }

    const bool motion_busy = MotionBusy();
    const bool allow_during_motion = (op == "disable" || op == "ping" || op == "get_state");
    if (motion_busy && !allow_during_motion) {
        return ErrorReply("motion is active; only ping/get_state/disable are allowed", "motion_aborted");
    }

    if (op == "enable") {
        std::lock_guard<std::mutex> lk(control_mutex_);
        if (!motor_io_.EnableAll()) {
            SetFault(motor_io_.GetLastFault());
            LOG("ERROR", "TwinAgent", "enable", ("failed: " + last_fault_.message).c_str());
            return ErrorReply("enable failed: " + last_fault_.message, "enable_failed");
        }
        enabled_ = true;
        LOG("INFO", "TwinAgent", "enable", "all motors enabled");
        return OkReply("enabled");
    }
    if (op == "disable") {
        AbortMotion("disabled by client");
        std::lock_guard<std::mutex> lk(control_mutex_);
        if (!motor_io_.DisableAll()) return ErrorReply("disable failed", "enable_failed");
        enabled_ = false;
        std::fill(last_joint_targets_.begin(), last_joint_targets_.end(), 0.0f);
        LOG("INFO", "TwinAgent", "disable", "all motors disabled");
        return OkReply("disabled");
    }
    if (op == "init" || op == "goto_offset") {
        float duration_sec = 2.5f;
        if (tokens.size() >= 2) {
            duration_sec = std::stof(tokens[1]);
        }
        std::string err;
        std::vector<std::string> offline_motors;
        if (!InitToOffset(duration_sec, err, offline_motors)) {
            LOG("ERROR", "TwinAgent", "init", ("failed: " + err).c_str());
            return ErrorReply(err, last_fault_.has_fault ? last_fault_.code : "enable_failed");
        }
        std::ostringstream oss;
        oss << "{\"ok\":true,\"msg\":\"moved to offset and holding\"";
        if (!offline_motors.empty()) {
            oss << ",\"offline_motors\":[";
            for (size_t i = 0; i < offline_motors.size(); ++i) {
                if (i > 0) oss << ",";
                oss << "\"" << offline_motors[i] << "\"";
            }
            oss << "]";
        }
        oss << "}\n";
        LOG("INFO", "TwinAgent", "init", "moved to offset and holding");
        return oss.str();
    }
    if (op == "set_joint") {
        std::vector<float> targets;
        std::string err;
        if (!ParseFloatList(tokens, 1, minimal::kNumMotors, targets, err)) return ErrorReply(err, "bad_command");
        for (int i = 0; i < minimal::kNumMotors; ++i) targets[i] = motor_io_.ClampRelativeTargetRad(i, targets[i]);
        if (!SetJointTargets(targets, err)) return ErrorReply(err, "send_failed");
        return OkReply("joint targets updated");
    }
    if (op == "replay") {
        if (!(tokens.size() == 2 || tokens.size() == 3 || tokens.size() == 4))
            return ErrorReply("replay expects: replay <csv_path> [speed_factor] [--no-feedback-check]", "bad_command");
        std::string csv_path = tokens[1];
        float speed_factor = 1.0f;
        bool skip_feedback_check = false;
        size_t arg_idx = 2;
        // Parse optional speed factor
        if (arg_idx < tokens.size() && tokens[arg_idx] != "--no-feedback-check") {
            try {
                speed_factor = std::stof(tokens[arg_idx]);
                ++arg_idx;
            } catch (const std::exception&) {
                return ErrorReply("invalid speed_factor: " + tokens[arg_idx], "bad_command");
            }
        }
        // Parse optional --no-feedback-check flag
        if (arg_idx < tokens.size() && tokens[arg_idx] == "--no-feedback-check") {
            skip_feedback_check = true;
        }
        std::string err;
        if (!StartReplayMotion(csv_path, speed_factor, err, skip_feedback_check)) {
            std::string code = "csv_error";
            if (err.find("feedback lost") != std::string::npos) code = "no_feedback";
            else if (err.find("send") != std::string::npos) code = "send_failed";
            LOG("ERROR", "TwinAgent", "replay", ("failed: " + err).c_str());
            return ErrorReply(err, code);
        }
        LOG("INFO", "TwinAgent", "replay", "started");
        return OkReply("replay started");
    }
    if (op == "set_mit_param") {
        if (tokens.size() != 5) return ErrorReply("set_mit_param expects: set_mit_param <kp> <kd> <vel_limit> <torque_limit>", "bad_command");
        try {
            const float kp = std::stof(tokens[1]);
            const float kd = std::stof(tokens[2]);
            const float vel_limit = std::stof(tokens[3]);
            const float torque_limit = std::stof(tokens[4]);
            if (kp <= 0 || kd <= 0 || vel_limit <= 0 || torque_limit <= 0) {
                return ErrorReply("all MIT params must be positive", "bad_command");
            }
            motor_io_.SetMITConfig(kp, kd, vel_limit, torque_limit);
            LOG("INFO", "TwinAgent", "set_mit_param", "updated");
            return OkReply("MIT params updated");
        } catch (const std::exception& e) {
            return ErrorReply(std::string("set_mit_param parse failed: ") + e.what(), "bad_command");
        }
    }

    return ErrorReply("unknown command: " + op, "bad_command");
}

std::string TwinAgent::SnapshotToJson() {
    const uint64_t seq = seq_.fetch_add(1) + 1;
    std::vector<float> motor_pos, motor_vel, motor_tau;
    std::vector<float> joint_obs;
    std::vector<float> target_joint_pos;

    {
        std::lock_guard<std::mutex> lk(control_mutex_);
        motor_io_.GetMotorStates(motor_pos, motor_vel, motor_tau);
        joint_obs = motor_io_.GetJointObs();
        target_joint_pos = last_joint_targets_;
    }

    std::vector<float> joint_pos(minimal::kNumMotors, 0.0f);
    std::vector<float> joint_vel(minimal::kNumMotors, 0.0f);
    for (int i = 0; i < minimal::kNumMotors; ++i) {
        if (i < static_cast<int>(joint_obs.size())) joint_pos[i] = joint_obs[i];
        if (i + minimal::kNumMotors < static_cast<int>(joint_obs.size())) joint_vel[i] = joint_obs[i + minimal::kNumMotors];
    }

    std::string active_motion;
    std::string motion_error;
    MotorFault fault;
    {
        std::lock_guard<std::mutex> lk(motion_mutex_);
        active_motion = active_motion_name_;
        motion_error = last_motion_error_;
        fault = last_fault_;
    }

    std::ostringstream oss;
    oss << "{"
        << "\"ok\":true,"
        << "\"mode\":\"" << (enabled_ ? "enabled" : "disabled") << "\","
        << "\"seq\":" << seq << ','
        << "\"motion_active\":" << (motion_active_ ? "true" : "false") << ','
        << "\"active_motion\":\"" << JsonEscape(active_motion) << "\","
        << "\"last_motion_error\":\"" << JsonEscape(motion_error) << "\","
        << "\"joint_positions\":" << JsonArray(joint_pos) << ','
        << "\"joint_velocities\":" << JsonArray(joint_vel) << ','
        << "\"target_joint_positions\":" << JsonArray(target_joint_pos);
    if (fault.has_fault) {
        oss << ",\"fault\":{"
            << "\"code\":\"" << JsonEscape(fault.code) << "\","
            << "\"joint_name\":\"" << JsonEscape(fault.joint_name) << "\","
            << "\"motor_index\":" << fault.motor_index << ','
            << "\"feedback_age_ms\":" << fault.feedback_age_ms << ','
            << "\"message\":\"" << JsonEscape(fault.message) << "\""
            << "}";
    }
    oss << "}\n";
    return oss.str();
}

}  // namespace twin

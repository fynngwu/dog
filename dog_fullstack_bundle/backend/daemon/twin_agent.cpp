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
#include <set>
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

TwinAgent::TwinAgent(int cmd_port, int state_port)
    : cmd_port_(cmd_port),
      state_port_(state_port),
      last_joint_targets_(minimal::kNumMotors, 0.0f) {
    motion_last_tick_ = std::chrono::steady_clock::now();
}

TwinAgent::~TwinAgent() {
    Stop();
}

void TwinAgent::SetFault(const MotorFault& f) {
    std::lock_guard<std::mutex> lk(motion_mutex_);
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
    if (!SetupServerSocket(state_port_, state_server_fd_, err)) {
        LOG("ERROR", "TwinAgent", "start", ("state port setup failed: " + err).c_str());
        ::close(cmd_server_fd_);
        cmd_server_fd_ = -1;
        return false;
    }

    running_ = true;
    cmd_thread_ = std::thread(&TwinAgent::CommandLoop, this);
    state_thread_ = std::thread(&TwinAgent::StateLoop, this);
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
    if (state_server_fd_ >= 0) {
        ::close(state_server_fd_);
        state_server_fd_ = -1;
    }

    if (cmd_thread_.joinable()) cmd_thread_.join();
    if (state_thread_.joinable()) state_thread_.join();
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

void TwinAgent::StateLoop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        const int client_fd = ::accept(state_server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0) {
            if (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            continue;
        }
        while (running_) {
            if (!SendAll(client_fd, SnapshotToJson())) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        ::close(client_fd);
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

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        offline_motors = motor_io_.GetOfflineJoints(500);
    }
    if (!motor_io_.MoveToOffset(duration_sec)) {
        err = "move_to_offset failed";
        return false;
    }
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
        SetFault(motor_io_.GetLastFault());
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

    motion_thread_ = std::thread([this, duration_sec, step_fn, on_success]() {
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
                if (action_idx[j] >= static_cast<int>(cols.size())) {
                    err = "csv row is shorter than header";
                    return false;
                }
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

bool TwinAgent::SendReplayFrameAtCursor(size_t cursor, std::string& err, bool update_cursor_after_send) {
    std::lock_guard<std::mutex> replay_lk(replay_mutex_);
    if (!replay_loaded_ || replay_samples_.empty()) {
        err = "replay csv is not loaded";
        return false;
    }
    if (cursor >= replay_samples_.size()) {
        err = "replay cursor out of range";
        return false;
    }
    MotorFault fault = motor_io_.CheckFeedbackFresh(500);
    if (fault.has_fault) {
        SetFault(fault);
        err = "replay aborted: " + fault.joint_name + " feedback lost age=" + std::to_string(fault.feedback_age_ms) + "ms";
        return false;
    }

    std::vector<float> targets = replay_samples_[cursor].targets_rel;
    for (int j = 0; j < minimal::kNumMotors; ++j) {
        targets[j] = motor_io_.ClampRelativeTargetRad(j, targets[j]);
    }
    {
        std::lock_guard<std::mutex> lk(control_mutex_);
        if (!enabled_) {
            err = "motors are not enabled";
            return false;
        }
        if (!motor_io_.SendJointRelativeTargets(targets)) {
            SetFault(motor_io_.GetLastFault());
            err = "failed to send replay frame: " + motor_io_.GetLastFault().message;
            return false;
        }
        last_joint_targets_ = targets;
    }
    if (update_cursor_after_send) {
        replay_cursor_ = std::min(cursor + 1, replay_samples_.size() - 1);
    }
    return true;
}

bool TwinAgent::SendCurrentReplayCursorFrame(std::string& err, bool advance_after_send) {
    size_t cursor = 0;
    {
        std::lock_guard<std::mutex> lk(replay_mutex_);
        if (!replay_loaded_ || replay_samples_.empty()) {
            err = "replay csv is not loaded";
            return false;
        }
        cursor = replay_cursor_;
    }
    return SendReplayFrameAtCursor(cursor, err, advance_after_send);
}

bool TwinAgent::StartReplayMotion(float speed_factor, std::string& err) {
    if (speed_factor <= 0.0f) {
        err = "speed_factor must be positive";
        return false;
    }
    std::vector<ReplaySample> samples;
    size_t start_cursor = 0;
    {
        std::lock_guard<std::mutex> lk(replay_mutex_);
        if (!replay_loaded_ || replay_samples_.empty()) {
            err = "replay csv is not loaded";
            return false;
        }
        samples = replay_samples_;
        start_cursor = replay_cursor_;
        replay_speed_factor_ = speed_factor;
    }
    if (start_cursor >= samples.size()) {
        err = "replay cursor out of range";
        return false;
    }
    const double start_time = samples[start_cursor].time_sec;
    const double end_time = samples.back().time_sec;
    const float duration_sec = static_cast<float>(std::max(0.05, (end_time - start_time) / speed_factor + 0.05));

    return LaunchMotionThread(
        "replay", duration_sec,
        [this, samples, start_cursor, speed_factor](double t, std::string& step_err) {
            const double replay_time = samples[start_cursor].time_sec + t * speed_factor;
            size_t idx = start_cursor;
            while (idx + 1 < samples.size() && samples[idx + 1].time_sec <= replay_time) {
                ++idx;
            }
            if (!SendReplayFrameAtCursor(idx, step_err, false)) {
                return false;
            }
            {
                std::lock_guard<std::mutex> lk(replay_mutex_);
                replay_cursor_ = idx;
            }
            return true;
        },
        [this, samples]() {
            std::lock_guard<std::mutex> lk(replay_mutex_);
            replay_cursor_ = samples.empty() ? 0 : (samples.size() - 1);
        },
        err);
}

bool TwinAgent::ParseJointIndexList(const std::string& token, std::vector<int>& indices, std::string& err) const {
    std::stringstream ss(token);
    std::string part;
    std::set<int> seen;
    while (std::getline(ss, part, ',')) {
        part = Trim(part);
        if (part.empty()) continue;
        try {
            const int idx = std::stoi(part);
            if (idx < 0 || idx >= minimal::kNumMotors) {
                err = "joint index out of range: " + part;
                return false;
            }
            if (seen.insert(idx).second) {
                indices.push_back(idx);
            }
        } catch (const std::exception& e) {
            err = std::string("joint index parse failed: ") + e.what();
            return false;
        }
    }
    if (indices.empty()) {
        err = "no joint index specified";
        return false;
    }
    return true;
}

std::string TwinAgent::ProcessCommand(const std::string& cmd) {
    const auto tokens = SplitWS(cmd);
    if (tokens.empty()) {
        return ErrorReply("empty command");
    }
    const std::string& op = tokens[0];

    if (op == "ping") return OkReply("pong");
    if (op == "get_state") return SnapshotToJson();

    const bool motion_busy = MotionBusy();
    const bool allow_during_motion = (
        op == "disable" || op == "ping" || op == "get_state" || op == "replay_stop");
    if (motion_busy && !allow_during_motion) {
        return ErrorReply("motion is active; only ping/get_state/disable/replay_stop are allowed", "motion_active");
    }

    try {
        if (op == "enable") {
            std::lock_guard<std::mutex> lk(control_mutex_);
            if (!motor_io_.EnableAll()) {
                SetFault(motor_io_.GetLastFault());
                return ErrorReply("enable failed: " + last_fault_.message, "enable_failed");
            }
            enabled_ = true;
            return OkReply("enabled");
        }
        if (op == "disable") {
            AbortMotion("disabled by client");
            std::lock_guard<std::mutex> lk(control_mutex_);
            if (!motor_io_.DisableAll()) return ErrorReply("disable failed", "disable_failed");
            enabled_ = false;
            std::fill(last_joint_targets_.begin(), last_joint_targets_.end(), 0.0f);
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
                return ErrorReply(err, last_fault_.has_fault ? last_fault_.code : "enable_failed");
            }
            std::ostringstream oss;
            oss << "{\"ok\":true,\"msg\":\"moved to offset and holding\"";
            if (!offline_motors.empty()) {
                oss << ",\"offline_motors\":" << JsonStringArray(offline_motors);
            }
            oss << "}\n";
            return oss.str();
        }
        if (op == "set_joint") {
            std::vector<float> targets;
            std::string err;
            if (!ParseFloatList(tokens, 1, minimal::kNumMotors, targets, err)) {
                return ErrorReply(err, "bad_command");
            }
            for (int i = 0; i < minimal::kNumMotors; ++i) {
                targets[i] = motor_io_.ClampRelativeTargetRad(i, targets[i]);
            }
            if (!SetJointTargets(targets, err)) {
                return ErrorReply(err, "send_failed");
            }
            return OkReply("joint targets updated");
        }
        if (op == "joint_test") {
            if (tokens.size() != 3) {
                return ErrorReply("joint_test expects: joint_test <idx[,idx...]> <target_rad>", "bad_command");
            }
            std::vector<int> indices;
            std::string err;
            if (!ParseJointIndexList(tokens[1], indices, err)) {
                return ErrorReply(err, "bad_command");
            }
            const float target = std::stof(tokens[2]);
            std::vector<float> targets;
            {
                std::lock_guard<std::mutex> lk(control_mutex_);
                targets = last_joint_targets_;
            }
            for (int idx : indices) {
                targets[idx] = motor_io_.ClampRelativeTargetRad(idx, target);
            }
            if (!SetJointTargets(targets, err)) {
                return ErrorReply(err, "send_failed");
            }
            return OkReply("joint test target applied");
        }
        if (op == "joint_sine") {
            if (tokens.size() != 5) {
                return ErrorReply("joint_sine expects: joint_sine <idx[,idx...]> <amp_rad> <freq_hz> <duration_sec>", "bad_command");
            }
            std::vector<int> indices;
            std::string err;
            if (!ParseJointIndexList(tokens[1], indices, err)) {
                return ErrorReply(err, "bad_command");
            }
            const float amp = std::stof(tokens[2]);
            const float freq = std::stof(tokens[3]);
            const float duration_sec = std::stof(tokens[4]);
            if (freq <= 0.0f || duration_sec <= 0.0f) {
                return ErrorReply("joint_sine requires positive freq and duration", "bad_command");
            }
            std::vector<float> base_targets;
            {
                std::lock_guard<std::mutex> lk(control_mutex_);
                if (!enabled_) {
                    return ErrorReply("motors are not enabled", "not_enabled");
                }
                base_targets = last_joint_targets_;
            }
            if (!LaunchMotionThread(
                    "joint_sine", duration_sec,
                    [this, indices, amp, freq, base_targets](double t, std::string& step_err) {
                        std::vector<float> targets = base_targets;
                        const float value = static_cast<float>(amp * std::sin(2.0 * M_PI * freq * t));
                        for (int idx : indices) {
                            targets[idx] = motor_io_.ClampRelativeTargetRad(idx, value);
                        }
                        if (!SetJointTargets(targets, step_err)) {
                            return false;
                        }
                        return true;
                    },
                    nullptr,
                    err)) {
                return ErrorReply(err, "motion_start_failed");
            }
            return OkReply("joint sine started");
        }
        if (op == "load_replay_csv") {
            if (tokens.size() != 2) {
                return ErrorReply("load_replay_csv expects: load_replay_csv <csv_path>", "bad_command");
            }
            std::vector<ReplaySample> samples;
            std::string err;
            if (!LoadReplayCsv(tokens[1], samples, err)) {
                return ErrorReply(err, "csv_error");
            }
            {
                std::lock_guard<std::mutex> lk(replay_mutex_);
                replay_samples_ = std::move(samples);
                replay_csv_path_ = tokens[1];
                replay_cursor_ = 0;
                replay_speed_factor_ = 1.0f;
                replay_loaded_ = true;
            }
            return OkReply("replay csv loaded");
        }
        if (op == "replay_start") {
            float speed_factor = 1.0f;
            if (tokens.size() == 2) {
                speed_factor = std::stof(tokens[1]);
            } else if (tokens.size() > 2) {
                return ErrorReply("replay_start expects: replay_start [speed_factor]", "bad_command");
            }
            std::string err;
            if (!StartReplayMotion(speed_factor, err)) {
                return ErrorReply(err, err.find("feedback lost") != std::string::npos ? "no_feedback" : "replay_start_failed");
            }
            return OkReply("replay started");
        }
        if (op == "replay_stop") {
            AbortMotion("replay stopped by client");
            return OkReply("replay stopped");
        }
        if (op == "replay_step") {
            std::string err;
            if (!SendCurrentReplayCursorFrame(err, true)) {
                return ErrorReply(err, err.find("feedback lost") != std::string::npos ? "no_feedback" : "replay_step_failed");
            }
            return OkReply("replay frame sent");
        }
        if (op == "replay_prev") {
            size_t cursor = 0;
            {
                std::lock_guard<std::mutex> lk(replay_mutex_);
                if (!replay_loaded_ || replay_samples_.empty()) {
                    return ErrorReply("replay csv is not loaded", "replay_not_loaded");
                }
                cursor = replay_cursor_ > 0 ? replay_cursor_ - 1 : 0;
                replay_cursor_ = cursor;
            }
            std::string err;
            if (!SendReplayFrameAtCursor(cursor, err, false)) {
                return ErrorReply(err, err.find("feedback lost") != std::string::npos ? "no_feedback" : "replay_prev_failed");
            }
            return OkReply("replay previous frame sent");
        }
        if (op == "replay_seek") {
            if (tokens.size() != 2) {
                return ErrorReply("replay_seek expects: replay_seek <frame_idx>", "bad_command");
            }
            const size_t frame_idx = static_cast<size_t>(std::stoul(tokens[1]));
            std::lock_guard<std::mutex> lk(replay_mutex_);
            if (!replay_loaded_ || replay_samples_.empty()) {
                return ErrorReply("replay csv is not loaded", "replay_not_loaded");
            }
            if (frame_idx >= replay_samples_.size()) {
                return ErrorReply("frame_idx out of range", "bad_command");
            }
            replay_cursor_ = frame_idx;
            return OkReply("replay cursor moved");
        }
        if (op == "replay_status") {
            return SnapshotToJson();
        }
        if (op == "replay") {
            if (tokens.size() < 2 || tokens.size() > 3) {
                return ErrorReply("replay expects: replay <csv_path> [speed_factor]", "bad_command");
            }
            std::vector<ReplaySample> samples;
            std::string err;
            if (!LoadReplayCsv(tokens[1], samples, err)) {
                return ErrorReply(err, "csv_error");
            }
            {
                std::lock_guard<std::mutex> lk(replay_mutex_);
                replay_samples_ = std::move(samples);
                replay_csv_path_ = tokens[1];
                replay_cursor_ = 0;
                replay_speed_factor_ = 1.0f;
                replay_loaded_ = true;
            }
            float speed_factor = 1.0f;
            if (tokens.size() == 3) {
                speed_factor = std::stof(tokens[2]);
            }
            if (!StartReplayMotion(speed_factor, err)) {
                return ErrorReply(err, err.find("feedback lost") != std::string::npos ? "no_feedback" : "replay_start_failed");
            }
            return OkReply("replay started");
        }
        if (op == "set_mit_param") {
            if (tokens.size() != 5) {
                return ErrorReply("set_mit_param expects: set_mit_param <kp> <kd> <vel_limit> <torque_limit>", "bad_command");
            }
            const float kp = std::stof(tokens[1]);
            const float kd = std::stof(tokens[2]);
            const float vel_limit = std::stof(tokens[3]);
            const float torque_limit = std::stof(tokens[4]);
            if (kp <= 0 || kd <= 0 || vel_limit <= 0 || torque_limit <= 0) {
                return ErrorReply("all MIT params must be positive", "bad_command");
            }
            motor_io_.SetMITConfig(kp, kd, vel_limit, torque_limit);
            return OkReply("MIT params updated");
        }
    } catch (const std::exception& e) {
        return ErrorReply(std::string("command parse failed: ") + e.what(), "bad_command");
    }

    return ErrorReply("unknown command: " + op, "bad_command");
}

std::string TwinAgent::SnapshotToJson() {
    const uint64_t seq = seq_.fetch_add(1) + 1;
    std::vector<float> motor_pos, motor_vel, motor_tau;
    std::vector<float> joint_obs;
    std::vector<float> target_joint_pos;
    std::vector<std::string> offline_motors;

    {
        std::lock_guard<std::mutex> lk(control_mutex_);
        motor_io_.GetMotorStates(motor_pos, motor_vel, motor_tau);
        joint_obs = motor_io_.GetJointObs();
        target_joint_pos = last_joint_targets_;
        offline_motors = motor_io_.GetOfflineJoints(500);
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

    std::string replay_csv_path;
    size_t replay_cursor = 0;
    size_t replay_total = 0;
    float replay_speed = 1.0f;
    bool replay_loaded = false;
    {
        std::lock_guard<std::mutex> lk(replay_mutex_);
        replay_csv_path = replay_csv_path_;
        replay_cursor = replay_cursor_;
        replay_total = replay_samples_.size();
        replay_speed = replay_speed_factor_;
        replay_loaded = replay_loaded_;
    }

    std::ostringstream oss;
    oss << "{"
        << "\"ok\":true,"
        << "\"mode\":\"" << (enabled_ ? "enabled" : "disabled") << "\","
        << "\"seq\":" << seq << ','
        << "\"state\":{"
        << "\"joint_positions\":" << JsonArray(joint_pos) << ','
        << "\"joint_velocities\":" << JsonArray(joint_vel) << ','
        << "\"joint_torques\":" << JsonArray(motor_tau) << ','
        << "\"target_joint_positions\":" << JsonArray(target_joint_pos) << ','
        << "\"offline_motors\":" << JsonStringArray(offline_motors)
        << "},"
        << "\"motion\":{"
        << "\"active\":" << (motion_active_ ? "true" : "false") << ','
        << "\"name\":\"" << JsonEscape(active_motion) << "\","
        << "\"last_error\":\"" << JsonEscape(motion_error) << "\""
        << "},"
        << "\"replay\":{"
        << "\"loaded\":" << (replay_loaded ? "true" : "false") << ','
        << "\"csv_path\":\"" << JsonEscape(replay_csv_path) << "\","
        << "\"cursor\":" << replay_cursor << ','
        << "\"total_frames\":" << replay_total << ','
        << "\"speed_factor\":" << replay_speed << ','
        << "\"status\":\"" << (motion_active_ && active_motion == "replay" ? "playing" : "idle") << "\""
        << "}";
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

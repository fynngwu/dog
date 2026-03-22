#include "session_logger.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <cstdio>

namespace {
// 高效的浮点向量拼接：预分配字符串，直接追加，避免 ostringstream 分配开销
std::string Join(const std::vector<float>& v) {
    if (v.empty()) return {};
    std::string result;
    result.reserve(v.size() * 12);  // 预分配空间避免多次分配
    char buf[32];
    for (size_t i = 0; i < v.size(); ++i) {
        if (i > 0) result += ',';
        int len = std::snprintf(buf, sizeof(buf), "%.6g", v[i]);
        result.append(buf, len);
    }
    return result;
}

std::string GetTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}
}

bool SessionLogger::Open(const std::string& dir) {
    std::filesystem::create_directories(dir);
    
    meta_.open(dir + "/meta.json");
    events_.open(dir + "/events.log");
    csv_.open(dir + "/frames.csv");
    
    if (!meta_.is_open() || !events_.is_open() || !csv_.is_open()) {
        std::cerr << "[SessionLogger] Failed to open log files in " << dir << std::endl;
        return false;
    }
    
    std::cout << "[SessionLogger] Opened session: " << dir << std::endl;
    return true;
}

void SessionLogger::LogMeta(const std::string& text) {
    if (meta_.is_open()) {
        meta_ << text << std::endl;
    }
}

void SessionLogger::LogEvent(const std::string& text) {
    if (events_.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        events_ << "[" << std::put_time(std::localtime(&in_time_t), "%H:%M:%S") << "] " 
                << text << std::endl;
    }
}

void SessionLogger::LogFrame(const FrameRecord& r) {
    if (!csv_.is_open()) return;

    if (!header_written_) {
        csv_ << "tick,t_sec,loop_dt_ms,infer_ms,history_valid,history_full,control_enabled,"
             << "single_obs,raw_action,desired_abs,clipped_abs,"
             << "cmd_sent_abs,cmd_minus_pos,cmd_delta,max_track_err,clamp_count,imu_fresh,motors_fresh,"
             << "motor_pos_abs,motor_vel_abs,motor_torque\n";
        header_written_ = true;
    }

    csv_ << r.tick << ","
         << r.t_sec << ","
         << r.loop_dt_ms << ","
         << r.infer_ms << ","
         << r.history_valid << ","
         << r.history_full << ","
         << r.control_enabled << ",\""
         << Join(r.single_obs) << "\",\""
         << Join(r.raw_action) << "\",\""
         << Join(r.desired_abs) << "\",\""
         << Join(r.clipped_abs) << "\",\""
         << Join(r.cmd_sent_abs) << "\",\""
         << Join(r.cmd_minus_pos) << "\",\""
         << Join(r.cmd_delta) << "\","
         << r.max_track_err << ","
         << r.clamp_count << ","
         << r.imu_fresh << ","
         << r.motors_fresh << ",\""
         << Join(r.motor_pos_abs) << "\",\""
         << Join(r.motor_vel_abs) << "\",\""
         << Join(r.motor_torque) << "\"\n";

    frame_count_++;

    // 定期 flush 确保数据不会因崩溃丢失
    if (frame_count_ % flush_interval_ == 0) {
        csv_.flush();
        events_.flush();
    }
}

void SessionLogger::Close() {
    if (meta_.is_open()) meta_.close();
    if (events_.is_open()) events_.close();
    if (csv_.is_open()) csv_.close();
    std::cout << "[SessionLogger] Closed. Total frames: " << frame_count_ << std::endl;
}
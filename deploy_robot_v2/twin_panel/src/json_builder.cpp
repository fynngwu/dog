#include "hw_cli/json_builder.hpp"

#include <iomanip>
#include <sstream>

namespace hw_cli {
namespace {

template <typename T>
std::string JsonArray(const T& values) {
    std::ostringstream oss;
    oss << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) oss << ',';
        oss << std::fixed << std::setprecision(6) << values[i];
    }
    oss << ']';
    return oss.str();
}

std::string Escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else out += c;
    }
    return out;
}
}  // namespace

std::string ToJson(const ResultPayload& p) {
    std::ostringstream oss;
    oss << '{'
        << "\"ok\":" << (p.ok ? "true" : "false") << ','
        << "\"mode\":\"" << Escape(p.mode) << "\","
        << "\"seq\":" << p.seq << ','
        << "\"rtt_ms\":" << p.rtt_ms << ','
        << "\"error\":\"" << Escape(p.error) << "\","
        // 状态可信度字段
        << "\"sample_fresh\":" << (p.sample_fresh ? "true" : "false") << ','
        << "\"max_feedback_age_ms\":" << p.max_feedback_age_ms << ','
        << "\"motors_online_count\":" << p.motors_online_count << ','
        // 动作数据
        << "\"raw_action\":" << JsonArray(p.raw_action) << ','
        << "\"target_joint_rel\":" << JsonArray(p.target_joint_rel) << ','
        << "\"target_motor_abs\":" << JsonArray(p.target_motor_abs) << ','
        << "\"motor_abs\":" << JsonArray(p.motor_abs) << ','
        << "\"motor_vel\":" << JsonArray(p.motor_vel) << ','
        << "\"motor_tau\":" << JsonArray(p.motor_tau) << ','
        << "\"hw_joint_rel\":" << JsonArray(p.hw_joint_rel)
        << '}';
    return oss.str();
}

}  // namespace hw_cli

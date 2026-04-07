#pragma once

#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace twin {

inline std::string Trim(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])) != 0) {
        ++b;
    }
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])) != 0) {
        --e;
    }
    return s.substr(b, e - b);
}

inline std::vector<std::string> SplitWS(const std::string& s) {
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string tok;
    while (iss >> tok) {
        out.push_back(tok);
    }
    return out;
}

inline std::string JsonEscape(const std::string& s) {
    std::ostringstream oss;
    for (const char c : s) {
        switch (c) {
            case '\\': oss << "\\\\"; break;
            case '"': oss << "\\\""; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default: oss << c; break;
        }
    }
    return oss.str();
}

inline std::string JsonArray(const std::vector<float>& values) {
    std::ostringstream oss;
    oss << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i != 0) oss << ',';
        oss << values[i];
    }
    oss << ']';
    return oss.str();
}

inline std::string JsonStringArray(const std::vector<std::string>& values) {
    std::ostringstream oss;
    oss << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i != 0) oss << ',';
        oss << '"' << JsonEscape(values[i]) << '"';
    }
    oss << ']';
    return oss.str();
}

inline std::string OkReply(const std::string& msg = "ok") {
    std::ostringstream oss;
    oss << "{\"ok\":true,\"msg\":\"" << JsonEscape(msg) << "\"}\n";
    return oss.str();
}

inline std::string ErrorReply(const std::string& message,
                              const std::string& code = "bad_command",
                              const std::string& detail = "") {
    std::ostringstream oss;
    oss << "{\"ok\":false,\"msg\":\"" << JsonEscape(message) << "\""
        << ",\"error\":{"
        << "\"code\":\"" << JsonEscape(code) << "\","
        << "\"message\":\"" << JsonEscape(message) << "\"";
    if (!detail.empty()) {
        oss << ",\"detail\":\"" << JsonEscape(detail) << "\"";
    }
    oss << "}}\n";
    return oss.str();
}

inline bool ParseFloatList(const std::vector<std::string>& tokens,
                           size_t start_idx,
                           size_t expected_count,
                           std::vector<float>& out,
                           std::string& err) {
    if (tokens.size() != start_idx + expected_count) {
        std::ostringstream oss;
        oss << "expected " << expected_count << " floats, got "
            << (tokens.size() >= start_idx ? tokens.size() - start_idx : 0);
        err = oss.str();
        return false;
    }
    out.resize(expected_count);
    try {
        for (size_t i = 0; i < expected_count; ++i) {
            out[i] = std::stof(tokens[start_idx + i]);
        }
    } catch (const std::exception& e) {
        err = std::string("float parse failed: ") + e.what();
        return false;
    }
    return true;
}

}  // namespace twin

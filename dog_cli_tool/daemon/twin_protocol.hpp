#pragma once

#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace twin {

/** Trim leading and trailing ASCII whitespace. */
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

/** Split a string on ASCII whitespace. */
inline std::vector<std::string> SplitWS(const std::string& s) {
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string tok;
    while (iss >> tok) {
        out.push_back(tok);
    }
    return out;
}

/** Escape one string for safe embedding inside JSON text. */
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

/** Serialize a float vector as a compact JSON array. */
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

/** Serialize a string vector as a compact JSON array. */
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

/** Build a standard success reply. */
inline std::string OkReply(const std::string& msg = "ok") {
    std::ostringstream oss;
    oss << "{\"ok\":true,\"msg\":\"" << JsonEscape(msg) << "\"}\n";
    return oss.str();
}

/** Build a standard error reply. */
inline std::string ErrorReply(const std::string& msg) {
    std::ostringstream oss;
    oss << "{\"ok\":false,\"msg\":\"" << JsonEscape(msg) << "\"}\n";
    return oss.str();
}

/** Build an error reply with an error code for programmatic handling. */
inline std::string ErrorReply(const std::string& msg, const std::string& code) {
    std::ostringstream oss;
    oss << "{\"ok\":false,\"msg\":\"" << JsonEscape(msg) << "\""
        << ",\"code\":\"" << JsonEscape(code) << "\"}\n";
    return oss.str();
}

/** Parse exactly @p expected_count float tokens starting from @p start_idx. */
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

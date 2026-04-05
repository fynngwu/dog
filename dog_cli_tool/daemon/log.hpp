#pragma once

#include <cstdio>

/** Structured logging to stderr. All layers share this single header. */
inline void LOG(const char* level, const char* module, const char* msg) {
    std::fprintf(stderr, "[%s][%s] %s\n", level, module, msg);
}

inline void LOG(const char* level, const char* module, const char* action, const char* msg) {
    std::fprintf(stderr, "[%s][%s][%s] %s\n", level, module, action, msg);
}

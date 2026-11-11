// relay_util.h - 时间/杂项工具
#pragma once
#include <string>
#include <thread>
#include <chrono>

namespace relay {

inline long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

inline void sleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ms -> "%Y-%m-%d %H:%M:%S.xxx"
std::string timeStr(long long ms);

} // namespace relay

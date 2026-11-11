#include "relay/relay_util.h"
#include <ctime>
#include <thread>

namespace relay {

std::string timeStr(long long ms) {
    time_t sec = (time_t)(ms / 1000);
    int milli = (int)(ms % 1000);
    struct tm tmv;
#if defined(_WIN32)
    localtime_s(&tmv, &sec);
#else
    localtime_r(&sec, &tmv);
#endif
    char buf[64];
    snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec, milli);
    return std::string(buf);
}

} // namespace relay

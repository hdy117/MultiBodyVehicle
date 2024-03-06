#pragma once

#include "SimBase.h"
#include "glog/logging.h"

#define LOG_0 VLOG(0) // info log
#define LOG_1 VLOG(1) // debug log
#define LOG_2 VLOG(2) // debug log
#define LOG_3 VLOG(3) // debug log
#define LOG_INFO LOG(INFO)
#define LOG_ERROR LOG(ERROR)
#define LOG_WARNING LOG(WARNING)

#ifdef _WIN32
#pragma comment(lib, "glog.lib")
#pragma comment(lib, "gflags.lib")
#endif

namespace tx_car {
enum class LogLevel {
  INFO = 0,
  DEBUG = 1,
};

void setLogLevel(int &logLevel,
                 const LogLevel &defaultLogLevel = LogLevel::DEBUG);
} // namespace tx_car
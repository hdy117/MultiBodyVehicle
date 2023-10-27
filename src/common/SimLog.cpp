#include "SimLog.h"

namespace tx_car {
void setLogLevel(int &logLevel, const LogLevel &defaultLogLevel) {
  logLevel = static_cast<int>(defaultLogLevel);
  FLAGS_v = logLevel;
  LOG(INFO) << "FLAGS_v:" << logLevel << "\n";
}
} // namespace tx_car
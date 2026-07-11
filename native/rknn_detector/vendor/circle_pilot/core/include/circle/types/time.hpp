#pragma once

#include <chrono>
#include <cstdint>

namespace circle::types {

using TimestampNs = int64_t;

inline TimestampNs monotonicNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

inline double secondsBetween(TimestampNs t0, TimestampNs t1) {
  return static_cast<double>(t1 - t0) * 1.0e-9;
}

}  // namespace circle::types

#pragma once

#include <prometheus/gauge.h>
#include <prometheus/registry.h>

#include <cstdint>

#include "common/clock_functions.h"

namespace weld_control {

class PerformanceMetrics {
 public:
  PerformanceMetrics(prometheus::Registry* registry, clock_functions::SystemClockNowFunc system_clock_now_func);

  void UpdateABWLatencyLpcs(uint64_t time_stamp);

 private:
  clock_functions::SystemClockNowFunc system_clock_now_func_;
  prometheus::Gauge& abw_latency_lpcs_;
};

}  // namespace weld_control

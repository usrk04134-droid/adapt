#include "performance_metrics.h"

#include <prometheus/gauge.h>
#include <prometheus/registry.h>

#include <chrono>
#include <cstdint>
#include <utility>

#include "common/clock_functions.h"
#include "common/logging/application_log.h"

namespace weld_control {

PerformanceMetrics::PerformanceMetrics(prometheus::Registry* registry,
                                       clock_functions::SystemClockNowFunc system_clock_now_func)
    : system_clock_now_func_(std::move(system_clock_now_func)),
      abw_latency_lpcs_{prometheus::BuildGauge()
                            .Name("adaptio_abw_latency_lpcs_seconds")
                            .Help("ABW data latency from image taken")
                            .Register(*registry)
                            .Add({})} {}

void PerformanceMetrics::UpdateABWLatencyLpcs(uint64_t time_stamp) {
  auto now                  = system_clock_now_func_();
  auto scanner_data_time    = std::chrono::system_clock::time_point{std::chrono::nanoseconds{time_stamp}};
  auto latency              = now - scanner_data_time;
  auto latency_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(latency).count();

  auto now_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  auto latency_seconds = std::chrono::duration<double>(latency).count();
  LOG_DEBUG("ABW latency timestamps: now_ns={} scanner_ns={} latency_ms={} latency_s={}", now_nanoseconds,
            time_stamp, latency_milliseconds, latency_seconds);

  abw_latency_lpcs_.Set(static_cast<double>(latency_milliseconds / 1000.0));
}

}  // namespace weld_control

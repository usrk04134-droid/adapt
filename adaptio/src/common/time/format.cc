
#include "common/time/format.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace common::time {

auto TimePointTimeZoneToString(std::chrono::system_clock::time_point const& tp) -> std::string {
  auto const sysclock = std::chrono::system_clock::to_time_t(tp);

  struct tm tm{};
  localtime_r(&sysclock, &tm);

  auto offset_seconds = tm.tm_gmtoff;
  auto const sign     = (offset_seconds >= 0) ? '+' : '-';
  offset_seconds      = std::abs(offset_seconds);

  auto const offset_hours   = offset_seconds / SECONDS_HOUR;
  auto const offset_minutes = (offset_seconds % SECONDS_HOUR) / MINUTES_HOUR;

  std::stringstream ss;

  ss << sign << std::setfill('0') << std::setw(2) << offset_hours << ":" << std::setfill('0') << std::setw(2)
     << offset_minutes;

  return ss.str();
}

auto TimePointToString(std::chrono::system_clock::time_point const& tp, const Format& format) -> std::string {
  auto const sysclock = std::chrono::system_clock::to_time_t(tp);

  struct tm tm{};
  localtime_r(&sysclock, &tm);

  const auto* fmt       = "";
  const auto* delimiter = "";

  switch (format.type) {
    case Format::Type::TIMESTAMP:
      fmt       = "%Y-%m-%dT%H:%M:%S";
      delimiter = ".";
      break;
    case Format::Type::PATH:
      fmt       = "%Y%m%d_%H%M%S";
      delimiter = "_";
      break;
  }

  std::stringstream ss;
  ss << std::put_time(&tm, fmt);

  /* Fractional seconds */
  const std::chrono::system_clock::time_point::duration tt = tp.time_since_epoch();
  auto const seconds = std::chrono::duration_cast<std::chrono::seconds>(tt).count();

  switch (format.unit) {
    case Format::TimeUnit::MILLISECONDS: {
      auto const ms = std::chrono::duration_cast<std::chrono::milliseconds>(tt).count();
      ss << delimiter << std::setw(DIGITS_MS) << std::setfill('0') << static_cast<int>(ms - (seconds * MS_PER_SEC));
      break;
    }
    case Format::TimeUnit::MICROSECONDS: {
      auto const us = std::chrono::duration_cast<std::chrono::microseconds>(tt).count();
      ss << delimiter << std::setw(DIGITS_US) << std::setfill('0') << static_cast<int>(us - (seconds * US_PER_SEC));
      break;
    }
    case Format::TimeUnit::SECONDS:
      break;
  }

  if (format.type == Format::Type::TIMESTAMP) {
    /* Time zone */
    auto offset_seconds = tm.tm_gmtoff;
    auto const sign     = (offset_seconds >= 0) ? '+' : '-';
    offset_seconds      = std::abs(offset_seconds);

    auto const offset_hours   = offset_seconds / SECONDS_HOUR;
    auto const offset_minutes = (offset_seconds % SECONDS_HOUR) / MINUTES_HOUR;

    ss << sign << std::setfill('0') << std::setw(2) << offset_hours << ":" << std::setfill('0') << std::setw(2)
       << offset_minutes;
  }

  return ss.str();
}

}  // namespace common::time

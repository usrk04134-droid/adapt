#pragma once

#include <chrono>
#include <string>

namespace core::time {

auto const DIGITS_MS    = 3;
auto const DIGITS_US    = 6;
auto const MS_PER_SEC   = 1000;
auto const US_PER_SEC   = 1000000;
auto const SECONDS_HOUR = 3600;
auto const MINUTES_HOUR = 60;

struct Format {
  enum class TimeUnit {
    SECONDS,
    MILLISECONDS,
    MICROSECONDS,
  };
  enum class Type {
    TIMESTAMP,
    PATH,
  };
  Type type;
  TimeUnit unit;
};

// TS_SEC   - RFC3339  format with seconds precision e.g.      2025-06-05T10:36:28+02:00
// TS_MS    - RFC3339  format with milliseconds precision e.g. 2025-06-05T10:36:28.312+02:00
// TS_US    - RFC3339  format with microseconds precision e.g. 2025-06-05T10:36:28.312882+02:00
// PATH_SEC - FILENAME format with seconds precision e.g.      20250605_103629
// PATH_MS  - FILENAME format with milliseconds precision e.g. 20250605_103629_312
auto const FMT_TS_SEC   = Format{.type = Format::Type::TIMESTAMP, .unit = Format::TimeUnit::SECONDS};
auto const FMT_TS_MS    = Format{.type = Format::Type::TIMESTAMP, .unit = Format::TimeUnit::MILLISECONDS};
auto const FMT_TS_US    = Format{.type = Format::Type::TIMESTAMP, .unit = Format::TimeUnit::MICROSECONDS};
auto const FMT_PATH_SEC = Format{.type = Format::Type::PATH, .unit = Format::TimeUnit::SECONDS};
auto const FMT_PATH_MS  = Format{.type = Format::Type::PATH, .unit = Format::TimeUnit::MILLISECONDS};

auto TimePointTimeZoneToString(std::chrono::system_clock::time_point const& tp) -> std::string;
auto TimePointToString(std::chrono::system_clock::time_point const& tp, const Format& format) -> std::string;

}  // namespace core::time

#pragma once
#include <fmt/core.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>

namespace image_logging {

auto const STR_MODE_OFF              = "off";
auto const STR_MODE_DIRECT           = "direct";
auto const STR_MODE_BUFFERED         = "buffered";
auto const STR_MODE_ON_ERROR         = "on_error";
auto const STR_MODE_ON_ERROR_WELDING = "on_error_welding";

enum class Mode {
  OFF,
  DIRECT,
  BUFFERED,
  ON_ERROR,
  ON_ERROR_WELDING,
};

auto inline ModeToString(Mode mode) -> std::string {
  switch (mode) {
    case Mode::OFF:
      return STR_MODE_OFF;
    case Mode::DIRECT:
      return STR_MODE_DIRECT;
    case Mode::BUFFERED:
      return STR_MODE_BUFFERED;
    case Mode::ON_ERROR:
      return STR_MODE_ON_ERROR;
    case Mode::ON_ERROR_WELDING:
      return STR_MODE_ON_ERROR_WELDING;
      break;
  }

  return "invalid";
};

auto inline ModeFromString(const std::string& str) -> std::optional<Mode> {
  static const std::unordered_map<std::string, Mode> MAP_STRING_TO_MODE = {
      {STR_MODE_OFF,              Mode::OFF             },
      {STR_MODE_DIRECT,           Mode::DIRECT          },
      {STR_MODE_BUFFERED,         Mode::BUFFERED        },
      {STR_MODE_ON_ERROR,         Mode::ON_ERROR        },
      {STR_MODE_ON_ERROR_WELDING, Mode::ON_ERROR_WELDING},
  };

  auto it = MAP_STRING_TO_MODE.find(str);
  if (it != MAP_STRING_TO_MODE.end()) {
    return it->second;
  }
  return {};
};

struct Configuration {
  Mode mode{Mode::OFF};
  uint32_t sample_rate{1};
  uint32_t buffer_size{1};
  std::filesystem::path path;
  std::chrono::milliseconds on_error_interval;

  auto ToString() -> std::string {
    return fmt::format("mode={}, sample_rate: {}, buffer_size: {}, on_error_interval(ms): {}, path: {}",
                       ModeToString(mode), sample_rate, buffer_size, on_error_interval.count(), path.string());
  };
};

}  // namespace image_logging

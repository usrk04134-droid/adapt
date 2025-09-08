#pragma once

#include <cstdint>
#include <optional>

#include "controller/pn_driver/pn_driver.h"

namespace controller {

enum class ControllerType : uint32_t {
  PN_DRIVER = 0,
  SIMULATION,
};

struct ControllerConfigurationData {
  ControllerType type;
  int64_t cycle_time_ms;

  std::optional<controller::pn_driver::Configuration> pn_driver;
};
}  // namespace controller

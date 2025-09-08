#pragma once

#include <memory>

#include "common/clock_functions.h"
#include "controller/controller.h"
#include "controller/controller_configuration.h"

namespace controller {

class ControllerFactory {
 public:
  static auto CreateController(const ControllerConfigurationData& configuration,
                               clock_functions::SteadyClockNowFunc steady_clock_now_func)
      -> std::unique_ptr<Controller>;
};

}  // namespace controller

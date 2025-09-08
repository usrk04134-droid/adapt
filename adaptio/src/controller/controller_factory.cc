#include "controller_factory.h"

#include <memory>

#include "common/clock_functions.h"
#include "common/logging/application_log.h"
#include "controller/controller.h"
#include "controller/controller_configuration.h"
#include "controller/pn_driver/pn_driver.h"
#include "controller/simulation/simulation.h"

using controller::ControllerFactory;
using controller::ControllerType;
using controller::pn_driver::PnDriver;
using controller::simulation::Simulation;

// TODO: This should return a boost::result iso we can return a proper error instead of just nullptr
auto ControllerFactory::CreateController(const ControllerConfigurationData& configuration,
                                         clock_functions::SteadyClockNowFunc _func) -> std::unique_ptr<Controller> {
  ControllerPtr controller(nullptr);

  switch (configuration.type) {
    case ControllerType::PN_DRIVER: {
      using namespace controller::pn_driver;

      if (configuration.pn_driver.has_value()) {
        controller = std::make_unique<PnDriver>(configuration.pn_driver.value(), _func);
      } else {
        LOG_ERROR("PN driver selected but no configuration provided");
      }
    } break;
    case ControllerType::SIMULATION: {
      controller = std::make_unique<Simulation>();
    } break;
  }

  return controller;
}

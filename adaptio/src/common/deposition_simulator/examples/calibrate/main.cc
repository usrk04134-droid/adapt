#include <memory>

#include "sim-config.h"
#include "simulator_helpers.h"
#include "simulator_interface.h"
using deposition_simulator::ISimulator;
using deposition_simulator::SimConfig;

const bool CALIBRATE = true;
const bool DRIFT     = false;

auto main() -> int {  // NOLINT(readability*)

  if (CALIBRATE) {
    // Create simulator and set up config.
    double wire_diam                      = 4e-3;
    double wire_feed_speed                = 5.0 / 60;
    std::unique_ptr<ISimulator> simulator = deposition_simulator::CreateSimulator();

    SimConfig sim_config = cal_example::Configure(simulator.get());
    simulator->Initialize(sim_config);
    simulator->AddSingleWireTorch(wire_diam, wire_feed_speed);
    cal_example::MoveInGrid(simulator.get());
  }

  if (DRIFT) {
    double wire_diam                      = 4e-3;
    double wire_feed_speed                = 5.0 / 60;
    std::unique_ptr<ISimulator> simulator = deposition_simulator::CreateSimulator();

    SimConfig sim_config   = cal_example::Configure(simulator.get());
    sim_config.drift_speed = 10e-3;  // 10mm/rad
    simulator->Initialize(sim_config);
    simulator->AddSingleWireTorch(wire_diam, wire_feed_speed);
  }
}

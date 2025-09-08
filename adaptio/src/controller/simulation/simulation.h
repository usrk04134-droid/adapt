#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "controller/controller.h"
#include "controller/simulation/kinematics.h"
#include "controller/simulation/power_source.h"

namespace controller::simulation {

class Simulation final : public controller::Controller {
 public:
  Simulation();

  Simulation(Simulation&)                     = delete;
  auto operator=(Simulation&) -> Simulation&  = delete;
  Simulation(Simulation&&)                    = delete;
  auto operator=(Simulation&&) -> Simulation& = delete;

  ~Simulation() = default;
  auto Connect() -> boost::outcome_v2::result<bool> override;
  void Disconnect() override;
  auto IsConnected() -> bool override;

  void WriteSlideCrossXOutput(controller::AxisOutput data) override;
  void WriteSlideCrossYOutput(controller::AxisOutput data) override;
  void WriteWeldAxisOutput(controller::AxisOutput data) override;
  void WritePowerSource1Output(controller::PowerSourceOutput data) override;
  void WritePowerSource2Output(controller::PowerSourceOutput data) override;

 private:
  bool is_connected_;
  std::atomic<bool> shutdown_;
  PowerSource ps1_{};
  PowerSource ps2_{};
  Kinematics kinematics_{};
  bool commands_set_external_command_mode_{false};

  std::unique_ptr<std::thread> worker_thread_{};

  void Run();
  auto RetrieveInputs() -> boost::outcome_v2::result<bool> override;

  void SetInputs();
};

}  // namespace controller::simulation

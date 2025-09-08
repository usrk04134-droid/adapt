#pragma once

namespace controller::simulation {

class PowerSource {
 public:
  PowerSource() = default;

  PowerSource(PowerSource&)                     = delete;
  auto operator=(PowerSource&) -> PowerSource&  = delete;
  PowerSource(PowerSource&&)                    = delete;
  auto operator=(PowerSource&&) -> PowerSource& = delete;

  ~PowerSource() = default;

  void Update();

  struct {
    bool weld_start   = false;
    double voltage    = 0.0;
    double current    = 0.0;
    double wire_speed = 0.0;
    double weld_speed = 0.0;
    int method        = 0;
  } commands;

  struct {
    bool ready_to_start            = false;
    bool in_welding_sequence       = false;
    bool arcing                    = false;
    bool start_failure             = false;
    bool error                     = false;
    bool deviation_setpoint_actual = false;
    double voltage                 = 0.0;
    double current                 = 0.0;
    double deposition_rate         = 0.0;
    double heat_input              = 0.0;
  } status;

 private:
};

}  // namespace controller::simulation

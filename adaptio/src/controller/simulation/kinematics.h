#pragma once

#include <chrono>

#include "controller/simulation/kinematics_axis.h"

namespace controller::simulation {

class Kinematics {
 public:
  Kinematics();

  Kinematics(Kinematics&)                     = delete;
  auto operator=(Kinematics&) -> Kinematics&  = delete;
  Kinematics(Kinematics&&)                    = delete;
  auto operator=(Kinematics&&) -> Kinematics& = delete;

  ~Kinematics() = default;
  void Update();

  struct {
    struct KinematicsAxis::Commands x;
    struct KinematicsAxis::Commands y;
    struct KinematicsAxis::Commands z;
    struct KinematicsAxis::Commands a;
  } commands;  // NOLINT(*-non-private-member-variables-in-classes)

  struct {
    struct KinematicsAxis::Status x;
    struct KinematicsAxis::Status y;
    struct KinematicsAxis::Status z;
    struct KinematicsAxis::Status a;
  } status;  // NOLINT(*-non-private-member-variables-in-classes)

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> last_update_;
  KinematicsAxis x_axis_{"Horizontal Axis"};
  KinematicsAxis y_axis_{"Vertical Axis"};
  KinematicsAxis z_axis_{"Weld-Z Axis"};
  KinematicsAxis a_axis_{"Weld-A Axis"};
};

}  // namespace controller::simulation

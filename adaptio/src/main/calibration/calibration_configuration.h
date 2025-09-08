#pragma once
#include <cstdint>

namespace calibration {

struct GridConfiguration {
  double margin_top{};              // margin between top surface and measurement points
  double margin_x{};                // grid area expansion outside joint horizontally (on each side)
  double margin_z{};                // grid area expansion outside joint vertically (in addition to margin_top)
  double margin_c{};                // no dots closer than this to the touch left and right points
  uint32_t target_nr_gridpoints{};  // target number of dots (minimum)
};

struct RunnerConfiguration {
  double slide_velocity{};            // mm/sec
  double stabilization_time{};        // sec
  double near_target_delta{};         // mm
  double max_time_per_observation{};  // sec
};

struct Configuration {
  GridConfiguration grid_config;
  RunnerConfiguration runner_config;
};

}  // namespace calibration

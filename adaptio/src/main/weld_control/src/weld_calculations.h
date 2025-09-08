#pragma once

#include <algorithm>
#include <cassert>

#include "common/logging/application_log.h"
#include "common/math/math.h"

namespace {
auto const HEAT_INPUT_EXCEEDED_THRESHOLD = 0.01;
}  // namespace

namespace weld_control {

class WeldCalc {
 public:
  explicit WeldCalc() = default;

  struct CalculateAdaptivityInput {
    double weld_current_ratio{};
    double weld_speed_ratio{};
    double heat_input_min{};
    double heat_input_max{};
    struct {
      double current{};
      double voltage{};
    } ws1;
    struct {
      double current_min{};
      double current_max{};
      double voltage{};
    } ws2;
    struct {
      double weld_speed_min{};  // mm/sec
      double weld_speed_max{};  // mm/sec
    } weld_object;
  };

  struct CalculateAdaptivityOutput {
    double weld_speed{};  // mm/sec
    double ws2_current{};
  };

  auto static CalculateAdaptivity(const CalculateAdaptivityInput& params) -> CalculateAdaptivityOutput {
    /* new weld-speed in mm/sec */
    auto new_weld_speed = std::clamp(
        (params.weld_object.weld_speed_max + params.weld_object.weld_speed_min) / 2. * params.weld_speed_ratio,
        params.weld_object.weld_speed_min, params.weld_object.weld_speed_max);

    auto ws2_current_nom = (params.ws2.current_max + params.ws2.current_min) / 2.;
    auto new_ws2_current =
        std::clamp(((params.ws1.current + ws2_current_nom) - params.ws1.current) * params.weld_current_ratio,
                   params.ws2.current_min, params.ws2.current_max);

    auto calc_heat_input = [params](double ws2_current, double weld_speed) {
      return ((params.ws1.voltage * params.ws1.current) + (params.ws2.voltage * ws2_current)) / (weld_speed * 1000.);
    };

    auto heat_input = calc_heat_input(new_ws2_current, new_weld_speed);

    if (heat_input < params.heat_input_min) {
      /* calculate new heat-input for weld-speed calculation compensating for half of the exceeded heat-input limit */
      auto heat_input_limited = heat_input + (params.heat_input_min - heat_input) / 2.;

      new_weld_speed = std::clamp((params.ws1.voltage * params.ws1.current + params.ws2.voltage * new_ws2_current) /
                                      (heat_input_limited * 1000.),
                                  params.weld_object.weld_speed_min, params.weld_object.weld_speed_max);

      /* compensate for the remaning exceeded heat-input by adjusting the ws2 current */
      new_ws2_current =
          std::clamp(((1000. * params.heat_input_min * new_weld_speed) - (params.ws1.voltage * params.ws1.current)) /
                         params.ws2.voltage,
                     params.ws2.current_min, params.ws2.current_max);
    } else if (heat_input > params.heat_input_max) {
      /* calculate new heat-input for weld-speed calculation compensating for half of the exceeded heat-input limit */
      auto heat_input_limited = heat_input - (heat_input - params.heat_input_max) / 2.;

      new_weld_speed = std::clamp((params.ws1.voltage * params.ws1.current + params.ws2.voltage * new_ws2_current) /
                                      (heat_input_limited * 1000.),
                                  params.weld_object.weld_speed_min, params.weld_object.weld_speed_max);

      /* compensate for the remaning exceeded heat-input by adjusting the ws2 current */
      new_ws2_current =
          std::clamp(((1000. * params.heat_input_max * new_weld_speed) - (params.ws1.voltage * params.ws1.current)) /
                         params.ws2.voltage,
                     params.ws2.current_min, params.ws2.current_max);
    }

    heat_input = calc_heat_input(new_ws2_current, new_weld_speed);
    if (heat_input < params.heat_input_min - HEAT_INPUT_EXCEEDED_THRESHOLD ||
        heat_input > params.heat_input_max + HEAT_INPUT_EXCEEDED_THRESHOLD) {
      LOG_ERROR("Failed heat-input restriction calculation failed: {} <= {} <= {}!", params.heat_input_min, heat_input,
                params.heat_input_max);
      return {.weld_speed  = (params.weld_object.weld_speed_min + params.weld_object.weld_speed_max) / 2.,
              .ws2_current = ws2_current_nom};
    }

    return {.weld_speed = new_weld_speed, .ws2_current = new_ws2_current};
  }
};
}  // namespace weld_control

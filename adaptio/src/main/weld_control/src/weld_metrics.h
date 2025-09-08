#pragma once

#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include "weld_system_client/weld_system_types.h"

namespace weld_control {

class WeldMetrics {
 public:
  explicit WeldMetrics(prometheus::Registry* registry);

  void WSStateUpdate(weld_system::WeldSystemId id, const weld_system::WeldSystemState& state);

 private:
  weld_system::WeldSystemState previous_ws1_state_{weld_system::WeldSystemState::INIT};
  weld_system::WeldSystemState previous_ws2_state_{weld_system::WeldSystemState::INIT};

  prometheus::Counter* arcing_count_ws1_{};
  prometheus::Counter* arcing_count_ws2_{};
};

}  // namespace weld_control

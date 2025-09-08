#include "weld_metrics.h"

#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include "weld_system_client/weld_system_types.h"

namespace weld_control {

WeldMetrics::WeldMetrics(prometheus::Registry* registry) {
  auto& arcing_family = prometheus::BuildCounter()
                            .Name("adaptio_weld_system_transitions_to_arcing_total")
                            .Help("Number of transitions to arcing state")
                            .Register(*registry);

  arcing_count_ws1_ = &arcing_family.Add({
      {"weld_system", "1"}
  });
  arcing_count_ws2_ = &arcing_family.Add({
      {"weld_system", "2"}
  });
}

void WeldMetrics::WSStateUpdate(weld_system::WeldSystemId id, const weld_system::WeldSystemState& state) {
  switch (id) {
    case weld_system::WeldSystemId::ID1:
      if (state == weld_system::WeldSystemState::ARCING &&
          previous_ws1_state_ != weld_system::WeldSystemState::ARCING) {
        arcing_count_ws1_->Increment();
      }
      previous_ws1_state_ = state;
      break;
    case weld_system::WeldSystemId::ID2:

      if (state == weld_system::WeldSystemState::ARCING &&
          previous_ws2_state_ != weld_system::WeldSystemState::ARCING) {
        arcing_count_ws2_->Increment();
      }
      previous_ws2_state_ = state;
      break;
    case weld_system::WeldSystemId::INVALID:
      break;
  }
}
}  // namespace weld_control

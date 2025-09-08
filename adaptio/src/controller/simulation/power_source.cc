#include "power_source.h"

using controller::simulation::PowerSource;

void PowerSource::Update() {
  status.ready_to_start = true;

  if (commands.weld_start) {
    status.in_welding_sequence = true;
    status.arcing              = true;
  } else {
    status.in_welding_sequence = false;
    status.arcing              = false;
  }
}

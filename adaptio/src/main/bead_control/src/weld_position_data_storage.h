#pragma once

#include <boost/circular_buffer.hpp>
#include <cstdint>

#include "common/containers/relative_position_buffer.h"
#include "common/groove/groove.h"

namespace bead_control {

// The buffer size is set to worst case
// radius on weld object: 7000mm
// weld speed:            500mm/min
// rate of storage:       1200 per min
// data stored for two turns
// 2*pi*700/50*1200*2 = 211115
const uint32_t MAX_BUFFER_SIZE = 220000;

struct WeldPositionData {
  double weld_object_lin_velocity{};
  common::groove::Groove groove;
  struct {
    double current{0.};
    double wire_lin_velocity{0.};
  } weld_system1, weld_system2;
};

using WeldPositionDataStorage = common::containers::RelativePositionBuffer<WeldPositionData>;

}  // namespace bead_control

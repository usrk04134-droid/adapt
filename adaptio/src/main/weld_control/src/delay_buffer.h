#pragma once

#include <cstdint>

#include "common/containers/relative_position_buffer.h"
#include "common/groove/groove.h"

// The buffer size is set to worst case
// radius on weld object: 7000mm
// weld speed:            500mm/min
// rate of storage:       1200 per min
// data stored for 0.25 turns
// 2*pi*700/50*1200*0.25 = 26389
namespace weld_control {
const uint32_t DELAY_BUFFER_SIZE = 25000;

using DelayBuffer = common::containers::RelativePositionBuffer<common::groove::Groove>;

}  // namespace weld_control

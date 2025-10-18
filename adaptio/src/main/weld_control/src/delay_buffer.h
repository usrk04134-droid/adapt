#pragma once

#include <cstdint>

#include "common/containers/relative_position_buffer.h"
#include "common/groove/groove.h"

// Delay buffer holds data for the distance between laser line and weld head
// Weld speed 80cm/min = 13.3 mm/s
// sample rate 50ms
// Normal distance between laser line and weld head: 15cm
// Size: 150/13.3 * 20 = 225
namespace weld_control {
const uint32_t DELAY_BUFFER_SIZE = 225 * 10;

using DelayBuffer = common::containers::RelativePositionBuffer<common::Groove>;

}  // namespace weld_control

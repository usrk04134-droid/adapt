#pragma once

#include <string>
#include <vector>

namespace event {

using Code = std::string;

auto const ABP_INVALID_INPUT          = "101";
auto const ABP_CALCULATION_ERROR      = "102";
auto const GROOVE_DETECTION_ERROR     = "103";
auto const SCANNER_START_FAILED       = "104";
auto const WELD_AXIS_INVALID_POSITION = "105";
auto const ARCING_LOST                = "106";
auto const EDGE_SENSOR_LOST           = "107";
auto const HANDOVER_FAILED            = "108";

inline const std::vector<Code> EVENT_CODES = {
    ABP_INVALID_INPUT,          ABP_CALCULATION_ERROR, GROOVE_DETECTION_ERROR, SCANNER_START_FAILED,
    WELD_AXIS_INVALID_POSITION, ARCING_LOST,           EDGE_SENSOR_LOST,       HANDOVER_FAILED};

}  // namespace event

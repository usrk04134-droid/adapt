#pragma once

#include <optional>
#include <string>

#include "weld_system_types.h"

namespace weld_system {

auto MethodFromString(const std::string& method_str) -> std::optional<WeldSystemSettings::Method>;

auto RegulationTypeFromString(const std::string& regulation_type_str)
    -> std::optional<WeldSystemSettings::RegulationType>;

auto StartTypeFromString(const std::string& start_type_str) -> std::optional<WeldSystemSettings::StartType>;

}  // namespace weld_system

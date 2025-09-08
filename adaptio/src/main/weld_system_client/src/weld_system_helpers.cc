#include "weld_system_client/weld_system_helpers.h"

#include <optional>
#include <string>
#include <unordered_map>

#include "weld_system_client/weld_system_types.h"

namespace weld_system {

auto MethodFromString(const std::string& method_str) -> std::optional<WeldSystemSettings::Method> {
  static const std::unordered_map<std::string, WeldSystemSettings::Method> STRING_TO_METHOD_MAP = {
      {"dc",     WeldSystemSettings::Method::DC    },
      {"dc_neg", WeldSystemSettings::Method::DC_NEG},
      {"ac",     WeldSystemSettings::Method::AC    }
  };

  auto it = STRING_TO_METHOD_MAP.find(method_str);
  if (it != STRING_TO_METHOD_MAP.end()) {
    return it->second;
  }
  return std::nullopt;
}

auto RegulationTypeFromString(const std::string& regulation_type_str)
    -> std::optional<WeldSystemSettings::RegulationType> {
  static const std::unordered_map<std::string, WeldSystemSettings::RegulationType> STRING_TO_REGULATION_TYPE_MAP = {
      {"cw", WeldSystemSettings::RegulationType::CW},
      {"ca", WeldSystemSettings::RegulationType::CA},
      {"cc", WeldSystemSettings::RegulationType::CC}
  };

  auto it = STRING_TO_REGULATION_TYPE_MAP.find(regulation_type_str);
  if (it != STRING_TO_REGULATION_TYPE_MAP.end()) {
    return it->second;
  }
  return std::nullopt;
}

auto StartTypeFromString(const std::string& start_type_str) -> std::optional<WeldSystemSettings::StartType> {
  static const std::unordered_map<std::string, WeldSystemSettings::StartType> STRING_TO_START_TYPE_MAP = {
      {"direct",  WeldSystemSettings::StartType::DIRECT },
      {"scratch", WeldSystemSettings::StartType::SCRATCH}
  };

  auto it = STRING_TO_START_TYPE_MAP.find(start_type_str);
  if (it != STRING_TO_START_TYPE_MAP.end()) {
    return it->second;
  }
  return std::nullopt;
}

}  // namespace weld_system

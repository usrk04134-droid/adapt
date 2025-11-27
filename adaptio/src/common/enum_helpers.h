#pragma once

#include <string>
#include <unordered_map>

namespace common {

template <typename EnumType>
auto EnumToString(EnumType value, const std::unordered_map<EnumType, std::string>& map) -> std::string {
  auto it = map.find(value);
  return (it != map.end()) ? it->second : "unknown";
}

}  // namespace common

#pragma once

#include <nlohmann/json.hpp>
#include <optional>

namespace common {

// General case: to be specialized
template <typename T>
auto FromJson(const nlohmann::json& json_obj) -> std::optional<T> = delete;

}  // namespace common

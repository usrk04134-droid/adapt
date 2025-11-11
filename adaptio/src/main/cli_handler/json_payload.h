#pragma once

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

inline auto LogLevelToPayload(const std::string& loglevel) -> nlohmann::json {
  auto json_obj = nlohmann::json{
      {"logLevel", loglevel},
  };

  return json_obj;
}

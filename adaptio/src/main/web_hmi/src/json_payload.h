#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include "common/groove/groove.h"
#include "coordination/activity_status.h"

inline auto VersionToPayload(const std::string& version) -> nlohmann::json {
  auto json_obj = nlohmann::json{
      {"version", version},
  };

  return json_obj;
}

inline auto PositionToPayload(double horizontal, double vertical) -> nlohmann::json {
  nlohmann::json payload = {
      {"horizontal", horizontal},
      {"vertical",   vertical  }
  };

  return payload;
}

inline auto SlidesStatusToPayload(bool horizontal_in_position, bool vertical_in_position) -> nlohmann::json {
  nlohmann::json payload = {
      {"horizontalInPosition", horizontal_in_position},
      {"verticalInPosition",   vertical_in_position  }
  };

  return payload;
}

inline auto ActivityStatusToPayload(coordination::ActivityStatusE activity_status) -> nlohmann::json {
  nlohmann::json payload = {
      {"value", static_cast<uint32_t>(activity_status)},
  };

  return payload;
}

inline auto GrooveToPayload(const common::Groove& groove) -> nlohmann::json {
  auto json_groove = nlohmann::json::array();
  for (auto const& coordinate : groove) {
    nlohmann::json const json_coordinate = {
        {"horizontal", coordinate.horizontal},
        {"vertical",   coordinate.vertical  }
    };
    json_groove.push_back(json_coordinate);
  }

  nlohmann::json payload = {
      {"groove", json_groove},
  };

  return payload;
}

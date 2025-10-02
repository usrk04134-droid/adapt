#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include "calibration/calibration_types.h"
#include "coordination/activity_status.h"
#include "common/groove/groove.h"

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
      {"horizontal_in_position", horizontal_in_position},
      {"vertical_in_position",   vertical_in_position  }
  };

  return payload;
}

inline auto ActivityStatusToPayload(coordination::ActivityStatusE activity_status) -> nlohmann::json {
  nlohmann::json payload = {
      {"value", static_cast<uint32_t>(activity_status)},
  };

  return payload;
}

inline auto GrooveToPayload(const common::groove::Groove& groove) -> nlohmann::json {
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

inline auto LaserTorchCalibrationToPayload(bool valid, const calibration::LaserTorchCalibration& calibration)
    -> nlohmann::json {
  nlohmann::json payload = {
      {"valid", valid            },
      {"x",     calibration.x    },
      {"y",     calibration.y    },
      {"z",     calibration.z    },
      {"angle", calibration.angle}
  };

  return payload;
}

inline auto LaserTorchCalibrationFromPayload(const nlohmann::json& payload) -> calibration::LaserTorchCalibration {
  calibration::LaserTorchCalibration calibration{};

  payload.at("x").get_to(calibration.x);
  payload.at("y").get_to(calibration.y);
  payload.at("z").get_to(calibration.z);
  payload.at("angle").get_to(calibration.angle);
  if (payload.contains(std::string{"store_persistent"})) {
    payload.at("store_persistent").get_to(calibration.store_persistent);
  }

  return calibration;
}

inline auto ResultPayload(bool result) -> nlohmann::json {
  nlohmann::json payload = {
      {"result", result}
  };

  return payload;
}

inline auto WeldObjectCalibrationToPayload(bool valid, const calibration::WeldObjectCalibration& calibration)
    -> nlohmann::json {
  nlohmann::json payload = {
      {"valid",       valid                   },
      {"y",           calibration.y           },
      {"z",           calibration.z           },
      {"xAdjustment", calibration.x_adjustment},
  };

  return payload;
}

inline auto WeldObjectCalibrationFromPayload(const nlohmann::json& payload) -> calibration::WeldObjectCalibration {
  calibration::WeldObjectCalibration calibration{};

  payload.at("y").get_to(calibration.y);
  payload.at("z").get_to(calibration.z);
  payload.at("xAdjustment").get_to(calibration.x_adjustment);

  if (payload.contains(std::string{"store_persistent"})) {
    payload.at("store_persistent").get_to(calibration.store_persistent);
  }

  return calibration;
}

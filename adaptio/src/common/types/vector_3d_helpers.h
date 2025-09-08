#pragma once

#include <fmt/core.h>

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>

#include "common/json_helpers.h"
#include "common/logging/application_log.h"
#include "vector_3d.h"

namespace common {

inline auto ToString(const Vector3D& val) -> std::string {
  return fmt::format("c1: {:.4f} c2: {:.4f} c3 {:.4f}", val.c1, val.c2, val.c3);
}

inline auto ToJson(const Vector3D& val) -> nlohmann::json {
  return {
      {"c1", val.c1},
      {"c2", val.c2},
      {"c3", val.c3},
  };
}

template <>
inline auto FromJson<Vector3D>(const nlohmann::json& json_obj) -> std::optional<Vector3D> {
  Vector3D result;
  try {
    result.c1 = json_obj.at("c1").get<double>();
    result.c2 = json_obj.at("c2").get<double>();
    result.c3 = json_obj.at("c3").get<double>();
  } catch (const nlohmann::json::exception& e) {
    LOG_ERROR("Failed to parse Vector3D - exception: {}", e.what());
    return std::nullopt;
  }
  return result;
}

inline auto CommonVector2EigenVector(const common::Vector3D& common_vector) -> Eigen::Vector3d {
  return {common_vector.c1, common_vector.c2, common_vector.c3};
}

inline auto EigenVector2CommonVector(const Eigen::Vector3d& eigen_vector) -> common::Vector3D {
  return {.c1 = eigen_vector(0), .c2 = eigen_vector(1), .c3 = eigen_vector(2)};
}

}  // namespace common

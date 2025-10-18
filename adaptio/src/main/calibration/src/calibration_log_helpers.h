#pragma once

#include <common/types/vector_3d_helpers.h>

#include <cmath>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

#include "calibration/calibration_configuration.h"
#include "calibration_solver.h"
#include "joint_geometry/joint_geometry.h"

const int LOG_4_DECIMALS = 4;

inline auto Floor(double value, int decimals) -> double {
  double const md = pow(10, decimals);
  return std::floor(value * md) / md;
};

// These ToString functions are temporarily in this file. Clean up when function has been tested
[[maybe_unused]] inline auto ToString(const calibration::Observation& obs) -> std::string {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "slide=(" << obs.slide_position.horizontal << ", " << obs.slide_position.vertical << "), abw_points=[";
  for (size_t i = 0; i < obs.abw_points_lpcs.size(); ++i) {
    const auto& pt = obs.abw_points_lpcs[i];
    oss << "(" << pt.x << ", " << pt.y << ")";
    if (i + 1 < obs.abw_points_lpcs.size()) {
      oss << ", ";
    }
  }
  oss << "]";
  return oss.str();
}

inline auto ToJsonLog(const calibration::Observation& data) -> nlohmann::json {
  nlohmann::json obj = {
      {
       "slidePosition", {
              "horizontal",
              Floor(data.slide_position.horizontal, LOG_4_DECIMALS),
              "vertical",
              Floor(data.slide_position.vertical, LOG_4_DECIMALS),
          }, }
  };

  auto abw_points_lpcs_json = nlohmann::json::array();
  for (const auto& abw_point : data.abw_points_lpcs) {
    nlohmann::json abw_point_json = {
        {
         "x", Floor(abw_point.x, LOG_4_DECIMALS),
         "y", Floor(abw_point.y, LOG_4_DECIMALS),
         }
    };

    abw_points_lpcs_json.push_back(abw_point_json);
  }

  obj["abwPointsLpcs"] = abw_points_lpcs_json;

  return obj;
}

[[maybe_unused]] inline auto ToString(const calibration::TorchPlaneInfo& info) -> std::string {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "top_center=(" << info.top_center_at_torch_plane.horizontal << ", " << info.top_center_at_torch_plane.vertical
      << "), ";
  oss << "bottom_center=(" << info.bottom_center_at_torch_plane.horizontal << ", "
      << info.bottom_center_at_torch_plane.vertical << ")";
  return oss.str();
}

inline auto ToJsonLog(const calibration::TorchPlaneInfo& data) -> nlohmann::json {
  return {
      {
       "topCenterAtTorchPlane", {
              "horizontal",
              Floor(data.top_center_at_torch_plane.horizontal, LOG_4_DECIMALS),
              "vertical",
              Floor(data.top_center_at_torch_plane.vertical, LOG_4_DECIMALS),
          }, },
      {
       "bottomCenterAtTorchPlane",                      {
              "horizontal",
              Floor(data.bottom_center_at_torch_plane.horizontal, LOG_4_DECIMALS),
              "vertical",
              Floor(data.bottom_center_at_torch_plane.vertical, LOG_4_DECIMALS),
          }, },
  };
}

[[maybe_unused]] inline auto ToString(const calibration::GeometricConstants& gc) -> std::string {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "object_radius: " << gc.object_radius << ", scanner_mount_angle: " << gc.scanner_mount_angle
      << ", ltc_stickout: " << gc.ltc_stickout << ", ltc_laser_plane_distance: " << gc.ltc_laser_plane_distance;
  return oss.str();
}

inline auto ToJsonLog(const calibration::GeometricConstants& data) -> nlohmann::json {
  return {
      {"objectRadius",          data.object_radius           },
      {"scannerMountAngle",     data.scanner_mount_angle     },
      {"ltcStickout",           data.ltc_stickout            },
      {"ltcLaserPlaneDistance", data.ltc_laser_plane_distance},
  };
}

inline auto ToJsonLog(const joint_geometry::JointGeometry& data) -> nlohmann::json {
  return {
      {"upper_joint_width_mm",        data.upper_joint_width_mm       },
      {"groove_depth_mm",             data.groove_depth_mm            },
      {"left_joint_angle_rad",        data.left_joint_angle_rad       },
      {"right_joint_angle_rad",       data.right_joint_angle_rad      },
      {"left_max_surface_angle_rad",  data.left_max_surface_angle_rad },
      {"right_max_surface_angle_rad", data.right_max_surface_angle_rad}
  };
}

inline auto ToJsonLog(const calibration::GridConfiguration& data) -> nlohmann::json {
  return {
      {"margin_top",           data.margin_top          },
      {"margin_x",             data.margin_x            },
      {"margin_z",             data.margin_z            },
      {"margin_c",             data.margin_c            },
      {"target_nr_gridpoints", data.target_nr_gridpoints},
  };
}

inline auto ToJsonLog(const calibration::RunnerConfiguration& data) -> nlohmann::json {
  return {
      {"slide_velocity",           data.slide_velocity          },
      {"stabilization_time",       data.stabilization_time      },
      {"near_target_delta",        data.near_target_delta       },
      {"max_time_per_observation", data.max_time_per_observation},
  };
}

inline auto ToJsonLog(const calibration::CalibrationResult& data) -> nlohmann::json {
  nlohmann::json obj = {
      {"weldObjectRotationAxis", common::ToJson(data.weld_object_rotation_axis)},
      {"rotationCenter", common::ToJson(data.rotation_center)},
      {"torchToLpcsTranslation", common::ToJson(data.torch_to_lpcs_translation)},
      {"torchPlaneReference",
       {
           {"horizontal", Floor(data.torch_plane_reference.horizontal, LOG_4_DECIMALS)},
           {"vertical", Floor(data.torch_plane_reference.vertical, LOG_4_DECIMALS)},
       }},
      {"maxResidual", Floor(data.max_residual, LOG_4_DECIMALS)},
      {"standardDeviation", Floor(data.standard_deviation, LOG_4_DECIMALS)},
      {"residualSumOfSquares", Floor(data.residual_sum_of_squares, LOG_4_DECIMALS)},
      {"residualStandardError", Floor(data.residual_standard_error, LOG_4_DECIMALS)},
      {"rSquared", Floor(data.r_squared, LOG_4_DECIMALS)},
      {"adjustedRSquared", Floor(data.adjusted_r_squared, LOG_4_DECIMALS)},
  };

  auto projected_points_json = nlohmann::json::array();
  for (const auto& pt : data.projected_points) {
    projected_points_json.push_back({
        {"horizontal", Floor(pt.horizontal, LOG_4_DECIMALS)},
        {"vertical",   Floor(pt.vertical,   LOG_4_DECIMALS)}
    });
  }

  obj["projectedPoints"] = projected_points_json;
  return obj;
}

inline auto ToJsonLog(const common::Point& data) -> nlohmann::json {
  nlohmann::json obj = {
      {"horizontal", Floor(data.horizontal, LOG_4_DECIMALS)},
      {"vertical",   Floor(data.vertical,   LOG_4_DECIMALS)},
  };

  return obj;
}

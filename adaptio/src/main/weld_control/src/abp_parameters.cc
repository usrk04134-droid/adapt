#include "abp_parameters.h"

#include <fmt/core.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <cassert>
#include <exception>
#include <functional>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/logging/application_log.h"
#include "common/math/math.h"
#include "sql_table_names.h"

namespace {

auto StepUpLimitsToJson(const std::vector<double>& step_up_limits) -> nlohmann::json {
  auto json = nlohmann::json::array();
  for (auto step_up_limit : step_up_limits) {
    json.push_back(step_up_limit);
  }

  return json;
}

auto StepUpLimitsFromJson(const nlohmann::json& json) -> std::vector<double> {
  std::vector<double> vec;

  for (const auto& value : json) {
    if (value >= 0.0) {
      vec.push_back(value.get<double>());
    } else {
      break;
    }
  }
  return vec;
}

}  // namespace

namespace weld_control {

auto ABPParameters::WallOffset() const -> double { return wall_offset_; }
auto ABPParameters::BeadOverlap() const -> double { return bead_overlap_; }
auto ABPParameters::StepUpValue() const -> double { return step_up_value_; }
auto ABPParameters::BeadSwitchAngle() const -> double { return bead_switch_angle_; }
auto ABPParameters::KGain() const -> double { return k_gain_; }
auto ABPParameters::HeatInputMin() const -> double { return heat_input_.min; }
auto ABPParameters::HeatInputMax() const -> double { return heat_input_.max; }
auto ABPParameters::WS2CurrentMin() const -> double { return weld_system2_current_.min; }
auto ABPParameters::WS2CurrentMax() const -> double { return weld_system2_current_.max; }
auto ABPParameters::WS2CurrentAvg() const -> double {
  return std::midpoint(weld_system2_current_.min, weld_system2_current_.max);
}
auto ABPParameters::WeldSpeedMin() const -> double { return weld_speed_.min; }
auto ABPParameters::WeldSpeedMax() const -> double { return weld_speed_.max; }
auto ABPParameters::WeldSpeedAvg() const -> double { return std::midpoint(weld_speed_.min, weld_speed_.max); }
auto ABPParameters::FirstBeadPositionValue() const -> FirstBeadPosition { return first_bead_position_; }
auto ABPParameters::StepUpLimit(int bead_no) const -> std::optional<double> {
  assert(bead_no >= 3);
  auto const index = bead_no - 3;

  if (index + 1 > step_up_limits_.size()) {
    return {};
  }

  return step_up_limits_.at(index);
}
auto ABPParameters::CapCornerOffset() const -> double { return cap_corner_offset_; }
auto ABPParameters::CapBeads() const -> int { return cap_beads_; }
auto ABPParameters::CapInitDepth() const -> double { return cap_init_depth_; }

void ABPParameters::SetWallOffset(double value) { wall_offset_ = value; }
void ABPParameters::SetBeadOverlap(double value) { bead_overlap_ = value; }
void ABPParameters::SetStepUpValue(double value) { step_up_value_ = value; }
void ABPParameters::SetBeadSwitchAngle(double value) { bead_switch_angle_ = value; }
void ABPParameters::SetKGain(double value) { k_gain_ = value; }
void ABPParameters::SetHeatInputRange(double min, double max) {
  heat_input_.min = min;
  heat_input_.max = max;
}
void ABPParameters::SetWS2CurrentRange(double min, double max) {
  weld_system2_current_.min = min;
  weld_system2_current_.max = max;
}
void ABPParameters::SetFirstBeadPosition(FirstBeadPosition value) { first_bead_position_ = value; }
void ABPParameters::SetCapCornerOffset(double value) { cap_corner_offset_ = value; }
void ABPParameters::SetCapBeads(int value) { cap_beads_ = value; }
void ABPParameters::SetCapInitDepth(double value) { cap_init_depth_ = value; }

auto ABPParameters::IsValid() const -> bool {
  auto ok = true;

  ok &= wall_offset_ > 0.0;
  ok &= bead_overlap_ >= 0.0;
  ok &= bead_switch_angle_ >= ABP_PARAMETERS_MIN_BEAD_SWITCH_ANGLE;
  ok &= bead_switch_angle_ <= ABP_PARAMETERS_MAX_BEAD_SWITCH_ANGLE;

  ok &= heat_input_.min >= 0.0;
  ok &= heat_input_.max >= heat_input_.min;

  ok &= weld_system2_current_.min >= 0.0;
  ok &= weld_system2_current_.max >= weld_system2_current_.min;

  ok &= weld_speed_.min >= 0.0;
  ok &= weld_speed_.max >= weld_speed_.min;

  ok &= first_bead_position_ != FirstBeadPosition::INVALID;

  for (auto i = 0; i < step_up_limits_.size(); ++i) {
    auto const step_up_limit = step_up_limits_.at(i);

    ok &= step_up_limit >= 0.0;

    if (i > 0) {
      /* check that the step_up_limits are in increasing */
      ok &= step_up_limit >= step_up_limits_.at(i - 1);
    }
  }

  ok &= cap_beads_ >= 2;
  ok &= cap_init_depth_ >= 0.0;

  return ok;
}

auto ABPParameters::ToString() const -> std::string {
  return fmt::format(
      "wall_offset: {} bead_overlap: {}, step_up_value: {}, bead_switch_angle(deg): {:.1f}, k_gain: {}, "
      "first_bead_position: {}, heat_input: {}-{}, weld_system2_current: {}-{}, weld_speed(cm/sec): {:.2f}-{:.2f}, "
      "step_up_limits: {}, cap_corner_offset: {}, cap_beads: {}, cap_init_depth: {}",
      wall_offset_, bead_overlap_, step_up_value_, common::math::RadToDeg(bead_switch_angle_), k_gain_,
      FirstBeadPositionToString(first_bead_position_), heat_input_.min, heat_input_.max, weld_system2_current_.min,
      weld_system2_current_.max, common::math::MmSecToCmMin(weld_speed_.min),
      common::math::MmSecToCmMin(weld_speed_.max), StepUpLimitsToJson(step_up_limits_).dump(), cap_corner_offset_,
      cap_beads_, cap_init_depth_);
}

auto ABPParameters::ToJson() const -> nlohmann::json {
  return {
      {"wallOffset",         wall_offset_                                                                          },
      {"beadOverlap",        bead_overlap_                                                                         },
      {"stepUpValue",        step_up_value_                                                                        },
      {"kGain",              k_gain_                                                                               },
      {"firstBeadPosition",  FirstBeadPositionToString(first_bead_position_)                                       },
      {"heatInput",          {{"min", heat_input_.min}, {"max", heat_input_.max}}                                  },
      {"weldSystem2Current", {{"min", weld_system2_current_.min}, {"max", weld_system2_current_.max}}              },
      {"weldSpeed",
       {{"min", common::math::MmSecToCmMin(weld_speed_.min)}, {"max", common::math::MmSecToCmMin(weld_speed_.max)}}},
      {"beadSwitchAngle",    common::math::RadToDeg(bead_switch_angle_)                                            },
      {"stepUpLimits",       StepUpLimitsToJson(step_up_limits_)                                                   },
      {"capCornerOffset",    cap_corner_offset_                                                                    },
      {"capBeads",           cap_beads_                                                                            },
      {"capInitDepth",       cap_init_depth_                                                                       }
  };
}

auto ABPParameters::FromJson(const nlohmann::json& json_obj) -> std::optional<ABPParameters> {
  ABPParameters abp;

  try {
    json_obj.at("wallOffset").get_to(abp.wall_offset_);
    json_obj.at("beadOverlap").get_to(abp.bead_overlap_);
    json_obj.at("stepUpValue").get_to(abp.step_up_value_);

    if (json_obj.contains("kGain")) {
      json_obj.at("kGain").get_to(abp.k_gain_);
    }

    if (json_obj.contains("heatInput")) {
      json_obj.at("heatInput").at("min").get_to(abp.heat_input_.min);
      json_obj.at("heatInput").at("max").get_to(abp.heat_input_.max);
    }

    if (json_obj.contains("weldSystem2Current")) {
      json_obj.at("weldSystem2Current").at("min").get_to(abp.weld_system2_current_.min);
      json_obj.at("weldSystem2Current").at("max").get_to(abp.weld_system2_current_.max);
    }

    if (json_obj.contains("weldSpeed")) {
      double weld_speed_min = 0.;
      double weld_speed_max = 0.;
      json_obj.at("weldSpeed").at("min").get_to(weld_speed_min);
      json_obj.at("weldSpeed").at("max").get_to(weld_speed_max);

      abp.weld_speed_.min = common::math::CmMinToMmSec(weld_speed_min);
      abp.weld_speed_.max = common::math::CmMinToMmSec(weld_speed_max);
    }

    if (json_obj.contains("beadSwitchAngle")) {
      double angle{};
      json_obj.at("beadSwitchAngle").get_to(angle);
      abp.bead_switch_angle_ = common::math::DegToRad(angle);
    }

    if (json_obj.contains("firstBeadPosition")) {
      std::string pos_str;
      json_obj.at("firstBeadPosition").get_to(pos_str);
      abp.first_bead_position_ = FirstBeadPositionFromString(pos_str);
    }

    if (json_obj.contains("stepUpLimits")) {
      auto const& json_step_up_limits = json_obj.at("stepUpLimits");
      abp.step_up_limits_             = StepUpLimitsFromJson(json_step_up_limits);
    }

    if (json_obj.contains("capCornerOffset")) {
      json_obj.at("capCornerOffset").get_to(abp.cap_corner_offset_);
    }

    if (json_obj.contains("capBeads")) {
      json_obj.at("capBeads").get_to(abp.cap_beads_);
    }

    if (json_obj.contains("capInitDepth")) {
      json_obj.at("capInitDepth").get_to(abp.cap_init_depth_);
    }
  } catch (const nlohmann::json::exception& e) {
    LOG_ERROR("Failed to parse ABP Parameters from JSON - exception: {}", e.what());
    return std::nullopt;
  }

  return abp;
}

auto ABPParameters::FirstBeadPositionToString(FirstBeadPosition value) -> std::string {
  switch (value) {
    case FirstBeadPosition::LEFT:
      return "left";
    case FirstBeadPosition::RIGHT:
      return "right";
    default:
      break;
  }
  return "invalid";
}

auto ABPParameters::FirstBeadPositionFromString(const std::string& str) -> FirstBeadPosition {
  static const std::unordered_map<std::string, FirstBeadPosition> MAP = {
      {"left",  FirstBeadPosition::LEFT },
      {"right", FirstBeadPosition::RIGHT}
  };
  auto it = MAP.find(str);
  return it != MAP.end() ? it->second : FirstBeadPosition::INVALID;
}

void ABPParameters::CreateTable(SQLite::Database* db) {
  if (db->tableExists(ABP_PARAMETERS_TABLE_NAME)) {
    return;
  }

  std::string cmd = fmt::format(
      "CREATE TABLE {} ("
      "id INTEGER PRIMARY KEY, "
      "wall_offset REAL, "
      "bead_overlap REAL, "
      "step_up_value REAL, "
      "k_gain REAL, "
      "heat_input_min REAL, "
      "heat_input_max REAL, "
      "ws2_current_min REAL, "
      "ws2_current_max REAL, "
      "weld_speed_min REAL, "
      "weld_speed_max REAL, "
      "bead_switch_angle REAL, "
      "step_up_limits TEXT, "
      "cap_corner_offset REAL, "
      "cap_beads INTEGER, "
      "cap_init_depth REAL)",
      ABP_PARAMETERS_TABLE_NAME);

  db->exec(cmd);
}

auto ABPParameters::StoreFn() -> std::function<bool(SQLite::Database*, const ABPParameters&)> {
  return [](SQLite::Database* db, const ABPParameters& abp) -> bool {
    LOG_TRACE("Store ABPParameters {}", abp.ToString());

    try {
      std::string cmd = fmt::format("INSERT OR REPLACE INTO {} VALUES (1, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                                    ABP_PARAMETERS_TABLE_NAME);

      SQLite::Statement query(*db, cmd);
      SQLite::bind(query, abp.wall_offset_, abp.bead_overlap_, abp.step_up_value_, abp.k_gain_, abp.heat_input_.min,
                   abp.heat_input_.max, abp.weld_system2_current_.min, abp.weld_system2_current_.max,
                   abp.weld_speed_.min, abp.weld_speed_.max, abp.bead_switch_angle_,
                   StepUpLimitsToJson(abp.step_up_limits_).dump(), abp.cap_corner_offset_, abp.cap_beads_,
                   abp.cap_init_depth_);

      return query.exec() == 1;
    } catch (const std::exception& e) {
      LOG_ERROR("Failed to store ABPParameters - exception: {}", e.what());
      return false;
    }
  };
}

// NOLINTBEGIN(*-magic-numbers)
auto ABPParameters::GetFn() -> std::function<std::optional<ABPParameters>(SQLite::Database*)> {
  return [](SQLite::Database* db) -> std::optional<ABPParameters> {
    std::string cmd = fmt::format("SELECT * FROM {}", ABP_PARAMETERS_TABLE_NAME);
    SQLite::Statement query(*db, cmd);

    if (!query.executeStep()) {
      return std::nullopt;
    }

    ABPParameters abp;
    abp.wall_offset_              = query.getColumn(1).getDouble();
    abp.bead_overlap_             = query.getColumn(2).getDouble();
    abp.step_up_value_            = query.getColumn(3).getDouble();
    abp.k_gain_                   = query.getColumn(4).getDouble();
    abp.heat_input_.min           = query.getColumn(5).getDouble();
    abp.heat_input_.max           = query.getColumn(6).getDouble();
    abp.weld_system2_current_.min = query.getColumn(7).getDouble();
    abp.weld_system2_current_.max = query.getColumn(8).getDouble();
    abp.weld_speed_.min           = query.getColumn(9).getDouble();
    abp.weld_speed_.max           = query.getColumn(10).getDouble();
    abp.bead_switch_angle_        = query.getColumn(11).getDouble();
    abp.step_up_limits_           = StepUpLimitsFromJson(nlohmann::json::parse(query.getColumn(12).getString()));
    abp.cap_corner_offset_        = query.getColumn(13).getDouble();
    abp.cap_beads_                = query.getColumn(14).getInt();
    abp.cap_init_depth_           = query.getColumn(15).getDouble();

    return abp;
  };
}
// NOLINTEND(*-magic-numbers)

}  // namespace weld_control

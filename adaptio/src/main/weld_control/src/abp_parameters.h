#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "common/math/math.h"

namespace weld_control {

inline const double ABP_PARAMETERS_DEFAULT_K_GAIN      = 2.0;
inline const auto ABP_PARAMETERS_MAX_BEAD_SWITCH_ANGLE = common::math::DegToRad(30);
inline const auto ABP_PARAMETERS_MIN_BEAD_SWITCH_ANGLE = common::math::DegToRad(5);
inline const auto ABP_PARAMETERS_DEFAULT_SWITCH_ANGLE  = common::math::DegToRad(15);

// Note firstBeadPosition is currently not stored in the database.
// Maybe it should be a hardcoded constant in WeldControl instead?
class ABPParameters {
 public:
  enum class FirstBeadPosition { INVALID, LEFT, RIGHT };

  auto WallOffset() const -> double;
  auto BeadOverlap() const -> double;
  auto StepUpValue() const -> double;
  auto BeadSwitchAngle() const -> double;
  auto KGain() const -> double;
  auto HeatInputMin() const -> double;
  auto HeatInputMax() const -> double;
  auto WS2CurrentMin() const -> double;
  auto WS2CurrentMax() const -> double;
  auto WS2CurrentAvg() const -> double;
  auto WeldSpeedMin() const -> double;  // mm/sec
  auto WeldSpeedMax() const -> double;  // mm/sec
  auto WeldSpeedAvg() const -> double;  // mm/sec
  auto FirstBeadPositionValue() const -> FirstBeadPosition;
  auto StepUpLimit(int bead_no) const -> std::optional<double>;
  auto CapCornerOffset() const -> double;
  auto CapBeads() const -> int;
  auto CapInitDepth() const -> double;

  void SetWallOffset(double);
  void SetBeadOverlap(double);
  void SetStepUpValue(double);
  void SetBeadSwitchAngle(double);
  void SetKGain(double);
  void SetHeatInputRange(double min, double max);
  void SetWS2CurrentRange(double min, double max);
  void SetFirstBeadPosition(FirstBeadPosition);
  void SetCapCornerOffset(double);
  void SetCapBeads(int);
  void SetCapInitDepth(double);

  auto IsValid() const -> bool;
  auto ToString() const -> std::string;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json&) -> std::optional<ABPParameters>;

  static void CreateTable(SQLite::Database*);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const ABPParameters&)>;
  static auto GetFn() -> std::function<std::optional<ABPParameters>(SQLite::Database*)>;

 private:
  static auto FirstBeadPositionToString(FirstBeadPosition) -> std::string;
  static auto FirstBeadPositionFromString(const std::string&) -> FirstBeadPosition;

  double wall_offset_{};
  double bead_overlap_{};
  double step_up_value_{};
  double bead_switch_angle_{ABP_PARAMETERS_DEFAULT_SWITCH_ANGLE};
  double k_gain_{ABP_PARAMETERS_DEFAULT_K_GAIN};
  FirstBeadPosition first_bead_position_{FirstBeadPosition::LEFT};

  struct {
    double min{};
    double max{};
  } heat_input_, weld_system2_current_, weld_speed_;

  std::vector<double> step_up_limits_;
  double cap_corner_offset_{};
  int cap_beads_{2};
  double cap_init_depth_{};
};

}  // namespace weld_control

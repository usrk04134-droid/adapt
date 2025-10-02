#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <tuple>

#include "bead_control/bead_control.h"
#include "bead_calculations.h"
#include "bead_control/src/weld_position_data_storage.h"
#include "common/containers/position_buffer.h"
#include "common/groove/groove.h"
#include "common/logging/application_log.h"

namespace bead_control {

class BeadControlImpl : public BeadControl {
 public:
  explicit BeadControlImpl(WeldPositionDataStorage* storage, clock_functions::SteadyClockNowFunc now_func)
      : storage_(storage), now_func_(now_func) {}

  auto Update(const BeadControl::Input& data) -> std::pair<Result, Output> override;
  auto GetStatus() const -> Status override;
  void Reset() override;

  void SetWallOffset(double wall_offset) override { wall_offset_ = wall_offset; }
  void SetBeadSwitchAngle(double angle) override { bead_switch_angle_ = angle; }
  void SetBeadOverlap(double bead_overlap) override { bead_overlap_ = bead_overlap; }
  void SetStepUpValue(double step_up_value) override { step_up_value_ = step_up_value; }
  void SetFirstBeadPosition(WeldSide side) override { first_bead_position_ = side; }
  void SetKGain(double k_gain) override { k_gain_ = k_gain; }
  void SetCapBeads(int beads) override { cap_beads_ = beads; }
  void SetCapCornerOffset(double offset) override { cap_corner_offset_ = offset; }
  void SetTopWidthToNumBeads(const std::vector<BeadTopWidthData>& data) override { top_width_to_num_beads_ = data; }

  void ResetGrooveData() override;
  auto GetEmptyGroove(double pos) -> std::optional<common::groove::Groove> override;
  void RegisterCapNotification(std::chrono::seconds notification_grace, double last_layer_depth,
                               OnCapNotification on_notification) override;
  void UnregisterCapNotification() override;
  void NextLayerCap() override;

 private:
  enum class State { NONE, FILL, CAP } abp_state_{State::NONE};

  /* status */
  Status status_;

  /* bead control parameters */
  double wall_offset_{};  // in mm
  double bead_overlap_{}; // percentage between 0-1
  double step_up_value_{};
  WeldSide first_bead_position_{WeldSide::LEFT};
  double k_gain_{1.0};
  int cap_beads_{};
  double cap_corner_offset_{};
  std::vector<BeadTopWidthData> top_width_to_num_beads_{};

  double progress_{0.};
  std::optional<common::groove::Groove> average_empty_groove_;
  std::optional<GrooveFit> empty_layer_groove_fit_;
  std::optional<common::containers::PositionBuffer<common::groove::Groove>> empty_groove_buffer_;
  common::containers::PositionBuffer<common::groove::Groove> empty_layer_groove_buffer_;
  double empty_layer_average_groove_area_{};
  std::optional<common::groove::Groove> locked_groove_;
  bool last_fill_layer_{false};

  WeldPositionDataStorage* storage_;
  clock_functions::SteadyClockNowFunc now_func_;

  auto OnNewBead() -> Result;
  auto CalculateBeadPosition(const common::groove::Groove& groove,
                             const std::optional<common::groove::Groove>& maybe_empty_groove)
      -> std::tuple<double, tracking::TrackingMode, tracking::TrackingReference>;
  auto CalculateBeadSliceAreaRatio(const common::groove::Groove& maybe_empty_groove) -> double;
  void extracted();
};

}  // namespace bead_control

#pragma once

#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include <Eigen/Eigen>
#include <optional>
#include <utility>

#include "bead_control/bead_control.h"
#include "bead_control/bead_control_types.h"
#include "bead_control/src/groove_fit.h"
#include "bead_control/src/weld_position_data_storage.h"
#include "common/clock_functions.h"
#include "common/containers/position_buffer.h"
#include "macs/macs_groove.h"
#include "macs/macs_point.h"

namespace bead_control {

enum class LayerType {
  FILL,
  CAP,
};

class BeadControlImpl : public BeadControl {
 public:
  explicit BeadControlImpl(WeldPositionDataStorage* storage, clock_functions::SteadyClockNowFunc steady_clock_now_func);
  auto Update(const Input& input) -> std::pair<Result, Output> override;
  auto GetStatus() const -> Status override;
  void Reset() override;

  void SetWallOffset(double wall_offset) override { wall_offset_ = wall_offset; };
  void SetBeadSwitchAngle(double angle) override { bead_switch_angle_ = angle; };
  void SetBeadOverlap(double bead_overlap) override { bead_overlap_ = bead_overlap; };
  void SetStepUpValue(double step_up_value) override { step_up_value_ = step_up_value; };
  void SetFirstBeadPosition(WeldSide side) override { first_bead_position_ = side; };
  void SetKGain(double k_gain) override { k_gain_ = k_gain; };
  void SetCapBeads(int beads) override { cap_beads_ = beads; };
  void SetCapCornerOffset(double offset) override { cap_corner_offset_ = offset; };
  void SetBottomWidthToNumBeads(const std::vector<BeadBottomWidthData>& data) override {
    bottom_width_to_num_beads_ = data;
  };
  void ResetGrooveData() override;
  auto GetEmptyGroove(double pos) -> std::optional<macs::Groove> override;
  void RegisterCapNotification(std::chrono::seconds notification_grace, double last_layer_depth,
                               OnCapNotification on_notification) override;
  void UnregisterCapNotification() override;
  void NextLayerCap() override;

 private:
  WeldPositionDataStorage* storage_;
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;

  double wall_offset_{0.};
  double bead_switch_angle_{0.};
  double step_up_value_{0.};
  double weld_object_radius_{0.};
  double weld_system1_wire_diameter_{0.};
  double weld_system2_wire_diameter_{0.};
  bool weld_system1_twin_wire_{false};
  bool weld_system2_twin_wire_{false};
  WeldSide first_bead_position_{WeldSide::LEFT};
  double k_gain_{0.};
  int cap_beads_{2};
  double cap_corner_offset_{0.};
  std::vector<BeadBottomWidthData> bottom_width_to_num_beads_;

  int layer_number_{0};
  int bead_number_{0};
  double left_bead_area_{0.};
  std::optional<int> total_beads_in_prev_full_layer_;
  std::optional<int> total_beads_in_full_layer_;
  double bead_overlap_{0.};
  WeldPositionDataStorage::Slice empty_layer_;
  double last_angular_position_{0.};
  State state_{State::IDLE};
  double start_angular_position_{0.};
  double progress_{0.};
  std::optional<macs::Groove> average_empty_groove_;
  std::optional<GrooveFit> empty_layer_groove_fit_;
  std::optional<common::containers::PositionBuffer<macs::Groove>> empty_groove_buffer_;
  common::containers::PositionBuffer<macs::Groove> empty_layer_groove_buffer_;
  double empty_layer_average_groove_area_{};
  std::optional<macs::Groove> locked_groove_;
  bool last_fill_layer_{false};
  LayerType next_layer_type_{LayerType::FILL};
  LayerType layer_type_{LayerType::FILL};

  struct {
    std::chrono::seconds grace;
    double last_layer_depth;
    OnCapNotification on_notification;
  } cap_notification_;

  auto CalculateBeadsInLayer(double right_bead_area) -> std::tuple<std::optional<int>, double>;
  auto OnFillLayerFirstBead() -> bool;
  auto OnFillLayerSecondBead() -> bool;
  auto OnNewBead() -> Result;
  auto CalculateBeadPosition(const macs::Groove& groove, const std::optional<macs::Groove>& maybe_empty_groove)
      -> std::tuple<double, tracking::TrackingMode, tracking::TrackingReference>;
  auto CalculateBeadSliceAreaRatio(const macs::Groove& maybe_empty_groove) -> double;
  auto BeadOperationUpdate(double angular_position, double angular_velocity, bool steady_satisfied) -> Result;
  void UpdateGrooveLocking(const Input& input);
};

auto GetRepositionHorizontalVelocity(double angular_velocity, double radius, double angle) -> double;

}  // namespace bead_control

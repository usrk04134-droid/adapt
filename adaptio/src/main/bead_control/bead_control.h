#pragma once

#include <chrono>
#include <optional>
#include <ratio>

#include "bead_control/src/groove_fit.h"
#include "bead_control_types.h"
#include "common/groove/groove.h"
#include "tracking/tracking_manager.h"

namespace bead_control {

/* all groove coordinates are in MCS format */

class BeadControl {
 public:
  virtual ~BeadControl() = default;

  struct Input {
    double weld_object_angle{0.};
    double weld_object_ang_velocity{0.};
    double weld_object_radius{0.};
    struct {
      double wire_lin_velocity{0.};
      double current{0.};
      double wire_diameter{0.};
      bool twin_wire{false};
    } weld_system1, weld_system2;
    common::groove::Groove groove;

    bool in_horizontal_position{false};
    bool paused{false};

    /* distance added to 'weld_object_angle' used to look up how the future groove looks like for
     * 'bead-slice-area-ratio' and 'groove-area-ratio' calculations */
    double look_ahead_distance{};
  };

  enum class Result {
    OK,
    ERROR,
    FINISHED,
  };

  struct Output {
    double horizontal_offset{0.};
    tracking::TrackingMode tracking_mode{tracking::TrackingMode::TRACKING_LEFT_HEIGHT};
    tracking::TrackingReference tracking_reference{tracking::TrackingReference::BOTTOM};
    std::optional<double> horizontal_lin_velocity{std::nullopt};

    /* The bead-slice-area-ratio is calculated by dividing the groove into slices, one slice for each bead,
     * and then comparing the size of the current bead's area to the average bead area. If the current bead's
     * area is > the average a value > 1 will be returned and the opposite if the area is less.
     *
     * For example, a groove with two beads will slice the groove into two areas A1 and A2.
     *
     *  *-----------------------*
     *   \          |          /
     *    \    A1   |   A2    /
     *     \        |        /
     *      \       |       /
     *       *--*---*---*--*
     *
     * */
    double bead_slice_area_ratio{1.};

    /* The groove-area-ratio is calculated using an empty layer's average area divided by the empty layer's
     * area for the current position.
     *
     * values:
     *   1 - current position's area and average area are the same
     * < 1 - current position's area is > average area
     * > 1 - current position's area is < average area
     */
    double groove_area_ratio{1.};
  };

  struct Status {
    int bead_number{0};
    int layer_number{0};
    std::optional<int> total_beads{std::nullopt};
    double progress{0.};
    State state{State::IDLE};
  };

  virtual auto Update(const Input& data) -> std::pair<Result, Output> = 0;
  virtual auto GetStatus() const -> Status                            = 0;
  virtual void Reset()                                                = 0;

  virtual void SetWallOffset(double wall_offset)    = 0;
  virtual void SetBeadSwitchAngle(double angle)     = 0;
  virtual void SetBeadOverlap(double bead_overlap)  = 0;
  virtual void SetStepUpValue(double step_up_value) = 0;
  virtual void SetFirstBeadPosition(WeldSide side)  = 0;
  virtual void SetKGain(double k_gain)              = 0;
  virtual void SetCapBeads(int beads)               = 0;
  virtual void SetCapCornerOffset(double offset)    = 0;
  struct BeadTopWidthData {
    int beads_allowed;
    double required_width;
  };
  virtual void SetTopWidthToNumBeads(const std::vector<BeadTopWidthData>& data) = 0;
  virtual void ResetGrooveData()                                                = 0;
  virtual auto GetEmptyGroove(double pos) -> std::optional<common::groove::Groove>        = 0;

  using OnCapNotification                                                 = std::function<void()>;
  virtual void RegisterCapNotification(std::chrono::seconds notification_grace, double last_layer_depth,
                                       OnCapNotification on_notification) = 0;
  virtual void UnregisterCapNotification()                                = 0;
  virtual void NextLayerCap()                                             = 0;
};
}  // namespace bead_control

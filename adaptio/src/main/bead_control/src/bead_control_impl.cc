
#include "bead_control_impl.h"

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <boost/range/numeric.hpp>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <Eigen/Eigen>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include "bead_control/bead_control.h"
#include "bead_control/bead_control_types.h"
#include "bead_control/src/bead_calculations.h"
#include "bead_control/src/weld_position_data_buffer.h"
#include "common/clock_functions.h"
#include "common/groove/groove.h"
#include "common/groove/point.h"
#include "common/logging/application_log.h"
#include "common/math/math.h"
#include "groove_fit.h"
#include "tracking/tracking_manager.h"

namespace {
auto const FITTED_GROOVE_SAMPLES        = 250;
auto const FITTED_ABW_POINT_CURVE_ORDER = 8;
auto const OVERLAP_LOCK_MARGIN          = 70.0;  // mm
auto const MIN_FILL_RATIO               = 0.95;

auto LayerTypeToString(bead_control::LayerType layer_type) -> std::string {
  switch (layer_type) {
    case bead_control::LayerType::FILL:
      return "fill";
    case bead_control::LayerType::CAP:
      return "cap";
  }

  return "invalid";
}

}  // namespace

namespace bead_control {

BeadControlImpl::BeadControlImpl(double storage_resolution, clock_functions::SteadyClockNowFunc steady_clock_now_func)
    : steady_clock_now_func_(std::move(steady_clock_now_func)),
      empty_layer_groove_buffer_(2 * std::numbers::pi),
      storage_(nullptr),
      storage_resolution_(storage_resolution) {}

auto BeadControlImpl::CalculateBeadsInLayer(double right_bead_area) -> std::tuple<std::optional<int>, double> {
  assert(average_empty_groove_.has_value());

  auto const layer_area =
      BeadCalc::MeanLayerArea(average_empty_groove_.value(), left_bead_area_, right_bead_area, step_up_value_);

  if (layer_area <= 0.) {
    LOG_ERROR("Unexpected bead calculation result!");
    return {{}, 0.};
  }

  auto const average_bead_area = std::midpoint(left_bead_area_, right_bead_area);
  auto total_beads             = static_cast<int>(std::round(layer_area / average_bead_area));

  LOG_DEBUG("Actual depth of previous layer: left {:.5f}, right {:.5f}", average_empty_groove_.value().LeftDepth(),
            average_empty_groove_.value().RightDepth());

  if (!top_width_to_num_beads_.empty()) {
    auto const top_width =
        BeadCalc::MeanLayerTopWidth(average_empty_groove_.value(), left_bead_area_, right_bead_area, step_up_value_);
    for (auto const& data : top_width_to_num_beads_) {
      if (data.required_width <= top_width) {
        continue;
      }

      auto const max_beads_for_width = data.beads_allowed - 1;
      if (max_beads_for_width < total_beads) {
        total_beads = max_beads_for_width;
        LOG_INFO("Limit number of beads in layer to {} for top width {:.2f}", total_beads, top_width);
      }
      break;
    }
  }

  return {std::max(2, total_beads), layer_area};
}

auto BeadControlImpl::OnFillLayerFirstBead() -> bool {
  if (storage_.FillRatio() < MIN_FILL_RATIO) {
    LOG_ERROR("Not enough samples for empty layer!");
    return false;
  }

  auto const start = steady_clock_now_func_();
  empty_layer_groove_fit_ =
      GrooveFit(storage_, GrooveFit::Type::FOURIER, FITTED_ABW_POINT_CURVE_ORDER, FITTED_GROOVE_SAMPLES);

  LOG_INFO("Calculated empty layer groove fit using {}/{} samples, fill ratio {:.2f}, took {} ms",
           FITTED_GROOVE_SAMPLES, storage_.FilledSlots(), storage_.FillRatio() * 100,
           std::chrono::duration_cast<std::chrono::milliseconds>(steady_clock_now_func_() - start).count());

  average_empty_groove_ = common::Groove();

  empty_layer_groove_buffer_.Clear();
  empty_layer_average_groove_area_ = 0.0;
  left_bead_area_                  = 0.0;

  auto const samples = static_cast<double>(storage_.FilledSlots());
  for (auto& slot_data : storage_) {
    if (!slot_data.has_value()) {
      continue;
    }

    auto [pos, data] = slot_data.value();

    empty_layer_groove_buffer_.Store(pos, data.groove);
    empty_layer_average_groove_area_ += data.groove.Area() / samples;
    left_bead_area_                  += data.left_bead_area / samples;

    for (auto abw_point = 0; abw_point < common::ABW_POINTS; ++abw_point) {
      average_empty_groove_.value()[abw_point] += {
          .horizontal = data.groove[abw_point].horizontal / samples,
          .vertical   = data.groove[abw_point].vertical / samples,
      };
    }
  }

  last_fill_layer_ = cap_notification_.on_notification != nullptr &&
                     average_empty_groove_->AvgDepth() < cap_notification_.last_layer_depth;

  LOG_INFO("last-layer={} depth/threshold: {:.2f}/{:.2f}", last_fill_layer_ ? "yes" : "no",
           average_empty_groove_->AvgDepth(), cap_notification_.last_layer_depth);

  if (last_fill_layer_ && total_beads_in_prev_full_layer_.value_or(2) == 2) {
    /* special handling for last fill layer with only two beads since the notification is required before the last bead
     * is finished */
    auto [opt_beads, _] = CalculateBeadsInLayer(left_bead_area_);
    if (!opt_beads.has_value()) {
      return false;
    }

    if (opt_beads.value() == 2) {
      total_beads_in_full_layer_ = 2;
    }
  }

  return true;
}

auto BeadControlImpl::OnFillLayerSecondBead() -> bool {
  auto right_bead_area = 0.0;

  auto const samples = static_cast<double>(storage_.FilledSlots());
  for (auto& slot_data : storage_) {
    if (!slot_data.has_value()) {
      continue;
    }

    auto [pos, data]  = slot_data.value();
    right_bead_area  += data.right_bead_area / samples;
  }

  auto [opt_beads, layer_area] = CalculateBeadsInLayer(right_bead_area);
  if (!opt_beads.has_value()) {
    return false;
  }

  if (!total_beads_in_full_layer_) {
    total_beads_in_full_layer_ = opt_beads.value();
  }

  storage_.Clear();

  LOG_INFO(
      "Empty layer area: {:.5f} Left bead area: {:.5f} Right bead area: {:.5f} Beads in layer: {} groove width: "
      "{:.5f} groove area: {:.5f}",
      layer_area, left_bead_area_, right_bead_area, total_beads_in_full_layer_.value(),
      average_empty_groove_->BottomWidth(), average_empty_groove_->Area());

  return true;
}

auto BeadControlImpl::OnNewBead() -> Result {
  auto result = Result::OK;

  auto new_layer = bead_number_ == 0 || bead_number_ == total_beads_in_full_layer_.value_or(0);

  switch (layer_type_) {
    case LayerType::FILL:
      if (bead_number_ == 1) {
        result = OnFillLayerFirstBead() ? Result::OK : Result::ERROR;
      } else if (bead_number_ == 2) {
        result    = OnFillLayerSecondBead() ? Result::OK : Result::ERROR;
        new_layer = bead_number_ == total_beads_in_full_layer_.value_or(0);
      }

      if (result == Result::OK && new_layer) {
        bead_number_                    = 0;
        layer_type_                     = next_layer_type_;
        total_beads_in_prev_full_layer_ = layer_type_ == LayerType::FILL ? total_beads_in_full_layer_ : std::nullopt;
        total_beads_in_full_layer_      = layer_type_ == LayerType::CAP ? std::optional(cap_beads_) : std::nullopt;
        ++layer_number_;
      }
      break;

    case LayerType::CAP: {
      auto const new_layer = bead_number_ == 0 || (total_beads_in_full_layer_.has_value() &&
                                                   bead_number_ == total_beads_in_full_layer_.value());
      if (new_layer) {
        LOG_INFO("CAP finished!");
        result = Result::FINISHED;
      }
      break;
    }
  }

  if (result == Result::OK) {
    ++bead_number_;
    LOG_INFO("New bead: layer: {} bead: {} type: {}", layer_number_, bead_number_, LayerTypeToString(layer_type_));
  }

  return result;
}

auto GetRepositionHorizontalVelocity(double angular_velocity, double radius, double angle) -> double {
  // velocity in rad/sec, radius in mm, angle in radians, and output in mm/sec
  return angular_velocity * radius * tan(angle);
}

auto BeadControlImpl::CalculateBeadPosition(const common::Groove& groove,
                                            const std::optional<common::Groove>& maybe_empty_groove)
    -> std::tuple<double, tracking::TrackingMode, tracking::TrackingReference> {
  std::tuple<double, tracking::TrackingMode, tracking::TrackingReference> result;

  auto left_tracking_offset_adjustment  = 0.0;
  auto right_tracking_offset_adjustment = 0.0;
  if (locked_groove_.has_value()) {
    left_tracking_offset_adjustment =
        (groove[common::ABW_LOWER_LEFT] - locked_groove_.value()[common::ABW_LOWER_LEFT]).horizontal;
    right_tracking_offset_adjustment =
        (locked_groove_.value()[common::ABW_LOWER_RIGHT] - groove[common::ABW_LOWER_RIGHT]).horizontal;
    LOG_TRACE("{} diff compared to locked abw1/5: {:.3f}/{:.3f}", StateToString(state_),
              left_tracking_offset_adjustment, right_tracking_offset_adjustment);
  }

  BeadPlacementStrategy bead_placement_strategy{};
  double offset{};
  tracking::TrackingReference tracking_reference{};
  switch (layer_type_) {
    case LayerType::FILL:
      bead_placement_strategy = BeadPlacementStrategy::CORNERS_FIRST;
      offset                  = wall_offset_;
      tracking_reference      = tracking::TrackingReference::BOTTOM;
      break;
    case LayerType::CAP:
      bead_placement_strategy = BeadPlacementStrategy::SIDE_TO_SIDE;
      offset                  = cap_corner_offset_;
      tracking_reference      = tracking::TrackingReference::TOP;
      break;
  }

  switch (bead_placement_strategy) {
    case BeadPlacementStrategy::CORNERS_FIRST: {
      if (bead_number_ == 1) {
        auto const horizontal_offset =
            offset + (first_bead_position_ == WeldSide::LEFT ? left_tracking_offset_adjustment
                                                             : right_tracking_offset_adjustment);
        auto const tracking_mode = first_bead_position_ == WeldSide::LEFT
                                       ? tracking::TrackingMode::TRACKING_LEFT_HEIGHT
                                       : tracking::TrackingMode::TRACKING_RIGHT_HEIGHT;
        result                   = {horizontal_offset, tracking_mode, tracking_reference};
        break;
      }

      if (bead_number_ == 2) {
        auto const horizontal_offset =
            offset + (first_bead_position_ == WeldSide::LEFT ? right_tracking_offset_adjustment
                                                             : left_tracking_offset_adjustment);
        auto const tracking_mode = first_bead_position_ == WeldSide::LEFT
                                       ? tracking::TrackingMode::TRACKING_RIGHT_HEIGHT
                                       : tracking::TrackingMode::TRACKING_LEFT_HEIGHT;
        result                   = {horizontal_offset, tracking_mode, tracking_reference};
        break;
      }

      auto const total_beads_in_full_layer = total_beads_in_full_layer_.value();
      assert(total_beads_in_full_layer > 2);
      assert(average_empty_groove_.has_value());

      auto const center_tracking_offset_adjustment =
          (left_tracking_offset_adjustment - right_tracking_offset_adjustment) / 2;

      /* Beads are placed in the opposite direction to 'first_bead_position_'
       * Calculate relative bead position from 0-1 where 0 and 1 are the left and right corners, respectively */
      auto bead_position_rel = first_bead_position_ == WeldSide::LEFT
                                   ? (total_beads_in_full_layer - bead_number_ + 1) / (total_beads_in_full_layer - 1.)
                                   : (bead_number_ - 2.) / (total_beads_in_full_layer - 1.);

      if (maybe_empty_groove.has_value() && state_ == State::STEADY && !locked_groove_.has_value()) {
        auto const new_bead_position_rel =
            BeadCalc::BeadPositionAdjustment(maybe_empty_groove.value(), bead_position_rel, k_gain_);
        bead_position_rel = new_bead_position_rel;
      }

      auto const available_groove = std::max(tracking_reference == tracking::TrackingReference::BOTTOM
                                                 ? average_empty_groove_->BottomWidth()
                                                 : average_empty_groove_->TopWidth() - (2 * offset),
                                             0.);
      auto const horizontal_offset = ((bead_position_rel - 0.5) * available_groove) + center_tracking_offset_adjustment;

      result = {horizontal_offset, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, tracking_reference};
      break;
    }
    case BeadPlacementStrategy::SIDE_TO_SIDE:
      /* bead position adjustment currently not supported to SIDE_TO_SIDE placement */
      auto const total_beads_in_full_layer = total_beads_in_full_layer_.value();

      assert(total_beads_in_full_layer >= 2);

      /* Beads are placed in the 'first_bead_position_' direction
       * calculate relative bead position from 0-1 where 0 and 1 are the left and right corners, respectively */
      auto bead_position_rel = first_bead_position_ == WeldSide::LEFT
                                   ? (total_beads_in_full_layer - bead_number_) / (total_beads_in_full_layer - 1.)
                                   : (bead_number_ - 1) / (total_beads_in_full_layer - 1.);

      auto const available_groove =
          std::max(tracking_reference == tracking::TrackingReference::BOTTOM ? groove.BottomWidth()
                                                                             : groove.TopWidth() - (2 * offset),
                   0.);
      auto const horizontal_offset = ((0.5 - bead_position_rel) * available_groove);

      result = {horizontal_offset, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, tracking_reference};
      break;
  }

  return result;
}

auto BeadControlImpl::CalculateBeadSliceAreaRatio(const common::Groove& empty_groove) -> double {
  auto bead_number_l_to_r = 0;

  /* if total_beads_in_full_layer_ has not yet been calculated for the current layer -> use previous layers number of
   * beads since the number of beads for the ongoing layer should be >= that of the previous layer */
  auto total_beads = total_beads_in_prev_full_layer_.value_or(2);
  if (bead_number_ == 1) {
    bead_number_l_to_r = first_bead_position_ == WeldSide::LEFT ? 1 : total_beads;
  } else if (bead_number_ == 2) {
    bead_number_l_to_r = first_bead_position_ == WeldSide::LEFT ? total_beads : 1;
  } else {
    total_beads        = total_beads_in_full_layer_.value();
    bead_number_l_to_r = first_bead_position_ == WeldSide::LEFT ? total_beads - bead_number_ + 2 : bead_number_ - 1;
  }

  return BeadCalc::BeadSliceAreaRatio(empty_groove, bead_number_l_to_r, total_beads);
}

void BeadControlImpl::UpdateGrooveLocking(const Input& input) {
  if (state_ == State::OVERLAPPING) {
    return;
  }

  if (state_ == State::STEADY) {
    auto const lock_at_progress = 1.0 - (OVERLAP_LOCK_MARGIN / (std::numbers::pi * input.weld_object_radius));
    if (progress_ >= lock_at_progress) {
      if (!locked_groove_.has_value()) {
        locked_groove_ = input.groove;
        LOG_INFO("Locking groove");
      }
      return;
    }
  }

  if (locked_groove_) {
    locked_groove_ = {};
  }
}

auto BeadControlImpl::Update(const Input& input) -> std::pair<Result, Output> {
  weld_system1_wire_diameter_ = input.weld_system1.wire_diameter;
  weld_system2_wire_diameter_ = input.weld_system2.wire_diameter;
  weld_system1_twin_wire_     = input.weld_system1.twin_wire;
  weld_system2_twin_wire_     = input.weld_system2.twin_wire;
  weld_object_radius_         = input.weld_object_radius;

  auto bead_area = BeadCalc::BeadArea(input.weld_system1.wire_lin_velocity, input.weld_system1.wire_diameter,
                                      input.weld_object_ang_velocity * weld_object_radius_) +
                   BeadCalc::BeadArea(input.weld_system2.wire_lin_velocity, input.weld_system2.wire_diameter,
                                      input.weld_object_ang_velocity * weld_object_radius_);
  auto const data = WeldPositionData{
      .groove         = input.groove,
      .left_bead_area = bead_area,
  };

  if (!storage_.Available()) {
    storage_.Init(weld_object_radius_, storage_resolution_);
  }
  if (state_ == State::STEADY && !input.paused) {
    if (bead_number_ == 1) {
      storage_.Store(input.weld_object_angle, data);
    } else if (bead_number_ == 2) {
      auto maybe_weld_data = storage_.Get(input.weld_object_angle);
      if (maybe_weld_data.has_value()) {
        auto [pos, weld_data]     = maybe_weld_data.value();
        weld_data.right_bead_area = bead_area;
        storage_.Store(pos, weld_data);
      }
    }
  }

  auto const result = BeadOperationUpdate(input.weld_object_angle, input.weld_object_ang_velocity, input.paused,
                                          input.in_horizontal_position);
  switch (result) {
    case Result::OK:
      break;
    case Result::ERROR:
    case Result::FINISHED:
      return {result, {}};
  }

  if (layer_type_ == LayerType::FILL) {
    UpdateGrooveLocking(input);
  }

  std::optional<common::Groove> layer_empty_groove_fit;
  if (empty_layer_groove_fit_.has_value()) {
    auto const groove = empty_layer_groove_fit_->Fit(input.weld_object_angle);

    if (groove.IsValid()) {
      layer_empty_groove_fit = groove;
    }
  }

  auto bead_slice_area_ratio = 1.0;
  auto groove_area_ratio     = 1.0;
  if (!empty_layer_groove_buffer_.Empty()) {
    auto const groove = empty_layer_groove_buffer_.Get(
        std::fmod(input.weld_object_angle + input.look_ahead_distance, 2 * std::numbers::pi));

    if (groove.has_value() && groove->IsValid() && state_ == State::STEADY && layer_type_ == LayerType::FILL) {
      bead_slice_area_ratio = CalculateBeadSliceAreaRatio(groove.value());
      groove_area_ratio     = empty_layer_average_groove_area_ / groove->Area();
    }
  }

  auto const pos = CalculateBeadPosition(input.groove, layer_empty_groove_fit);
  auto const [horizontal_offset, tracking_mode, tracking_reference] = pos;

  auto output = Output{
      .horizontal_offset     = horizontal_offset,
      .tracking_mode         = tracking_mode,
      .tracking_reference    = tracking_reference,
      .bead_slice_area_ratio = bead_slice_area_ratio,
      .groove_area_ratio     = groove_area_ratio,
  };

  if (state_ == State::REPOSITIONING && input.weld_object_ang_velocity > 0.0) {
    output.horizontal_lin_velocity =
        GetRepositionHorizontalVelocity(input.weld_object_ang_velocity, input.weld_object_radius, bead_switch_angle_);
  }

  return {Result::OK, output};
}

auto BeadControlImpl::GetStatus() const -> Status {
  return Status{
      .bead_number  = bead_number_,
      .layer_number = layer_number_,
      .total_beads  = total_beads_in_full_layer_,
      .progress     = progress_,
      .state        = state_,
  };
}

void BeadControlImpl::Reset() {
  state_                          = State::IDLE;
  progress_                       = 0.;
  layer_number_                   = 0;
  bead_number_                    = 0;
  next_layer_type_                = LayerType::FILL;
  layer_type_                     = LayerType::FILL;
  total_beads_in_prev_full_layer_ = {};
  paused_angular_position_        = {};
}

void BeadControlImpl::ResumeBeadOperation(double angular_position) {
  assert(paused_angular_position_);

  LOG_INFO("Resume state {} at position {:.4f} layer: {} bead: {} progress: {:.1f}%", StateToString(state_),
           *paused_angular_position_, layer_number_, bead_number_, progress_ * 100.0);

    auto bead_operation_distance = 0.0;
    switch (state_) {
      case State::STEADY:
        bead_operation_distance = 2 * std::numbers::pi;
        break;
      case State::OVERLAPPING:
        bead_operation_distance = BeadCalc::Distance2Angle(weld_object_radius_, bead_overlap_);
        break;
      case State::REPOSITIONING:
        /* repositioning does not need to be resumed */
      case State::IDLE:
        return;
    }

  auto const distance_since_pause =
      common::math::WrappedDist(*paused_angular_position_, angular_position, 2 * std::numbers::pi);

  LOG_INFO("Distance since pause {:.1f} mm", common::math::AngularToLinear(distance_since_pause, weld_object_radius_));

  if (distance_since_pause >= 0.0) {
    // bead operation resumed in from of the pause position - this is an unexpected scenario - no special handling
    // needed
    return;
  }

  auto const distance_progress = distance_since_pause / bead_operation_distance;

  auto const new_progress = progress_ + distance_progress;
  if (new_progress >= 0.0) {
    LOG_INFO("Updating progress {:.1f}% -> {:.1f}%", progress_ * 100.0, new_progress * 100.0);
    progress_ = new_progress;
  } else {
    LOG_INFO("Resume position before previous start position -> set new start position");
    progress_               = 0.0;
    start_angular_position_ = angular_position;
  }
}

auto BeadControlImpl::BeadOperationUpdate(double angular_position, double angular_velocity, bool paused,
                                          bool in_horizontal_position) -> Result {
  auto start_repositioning = [this]() -> Result {
    auto const result = OnNewBead();
    if (result == Result::OK) {
      LOG_INFO("Start Repositioning");
      progress_ = 0.; /* reposition state will not update progress */
      state_    = State::REPOSITIONING;
    }

    return result;
  };

  auto update_angular_position = [this](double position, double distance) -> bool {
    if (distance <= 0.0) {
      return true;
    }

    if (std::fabs(position - last_angular_position_) <= std::numeric_limits<double>::epsilon()) {
      return false;
    }

    double const progress = (position > start_angular_position_
                                 ? position - start_angular_position_
                                 : position - start_angular_position_ + (2 * static_cast<double>(std::numbers::pi))) /
                            distance;

    LOG_TRACE("position: {:.4f} start_position: {:.4f} distance: {:.4f} progress: {:.4f}% -> {:.4f}%", position,
              start_angular_position_, distance, progress_ * 100.0, progress * 100.0);

    auto const done = progress >= 1 || progress < progress_;

    progress_ = done ? 1. : progress;

    return done;
  };

  auto result = Result::OK;

  if (paused && !paused_angular_position_) {
    paused_angular_position_ = last_angular_position_;
  } else if (!paused && paused_angular_position_) {
    ResumeBeadOperation(angular_position);
    paused_angular_position_ = {};
    last_angular_position_   = angular_position;
    return result;
  }

  switch (state_) {
    case State::IDLE:
      result = start_repositioning();
      break;
    case State::STEADY:
      if (paused) {
        break;
      }

      if (update_angular_position(angular_position, 2 * std::numbers::pi)) {
        /* Bead finished -> start overlapping */
        LOG_INFO("Start Overlapping at position: {:.4f}", angular_position);
        start_angular_position_ = angular_position;
        progress_               = 0.;
        state_                  = State::OVERLAPPING;
      }

      if (cap_notification_.on_notification != nullptr && last_fill_layer_ &&
          total_beads_in_full_layer_.value_or(0) == bead_number_) {
        auto const notification_at_progress =
            1.0 - (static_cast<double>(cap_notification_.grace.count()) / (2 * std::numbers::pi / angular_velocity));

        if (progress_ >= notification_at_progress) {
          cap_notification_.on_notification();
          cap_notification_.on_notification = nullptr;
        }
      }
      break;
    case State::REPOSITIONING:
      if (!paused && in_horizontal_position) {
        /* Repositioning finished -> start steady */
        LOG_INFO("Start Steady at position: {:.4f}", angular_position);
        start_angular_position_ = angular_position;
        progress_               = 0.;
        state_                  = State::STEADY;
      }
      break;
    case State::OVERLAPPING:
      if (!paused &&
          update_angular_position(angular_position, BeadCalc::Distance2Angle(weld_object_radius_, bead_overlap_))) {
        /* Overlap finished -> start repositioning */
        result = start_repositioning();
      }
      break;
    default:
      break;
  }

  last_angular_position_ = angular_position;

  return result;
}

void BeadControlImpl::ResetGrooveData() {
  empty_layer_groove_fit_ = {};
  empty_groove_buffer_    = {};
  empty_layer_groove_buffer_.Clear();
  storage_.Clear();
}

void BeadControlImpl::RegisterCapNotification(std::chrono::seconds notification_grace, double last_layer_depth,
                                              OnCapNotification on_notification) {
  cap_notification_.grace            = notification_grace;
  cap_notification_.last_layer_depth = last_layer_depth;
  cap_notification_.on_notification  = std::move(on_notification);
}

void BeadControlImpl::UnregisterCapNotification() {
  cap_notification_.grace            = {};
  cap_notification_.last_layer_depth = 0;
  cap_notification_.on_notification  = nullptr;
}

void BeadControlImpl::NextLayerCap() { next_layer_type_ = LayerType::CAP; }

}  // namespace bead_control

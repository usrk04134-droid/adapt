#include "calibration_sequence_runner.h"

#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <utility>
#include <vector>

#include "calibration/calibration_configuration.h"
#include "calibration/src/calibration_grid_generator.h"
#include "calibration/src/calibration_solver.h"
#include "common/groove/point.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_socket.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"

using calibration::CalibrationSequenceRunner;

namespace {
const uint32_t MILLISEC_PER_SEC = 1000;
}  // namespace

CalibrationSequenceRunner::CalibrationSequenceRunner(zevs::Timer* timer,
                                                     kinematics::KinematicsClient* kinematics_client,
                                                     std::vector<GridPoint> grid_points, OnFailure on_failure,
                                                     OnComplete on_complete, OnProgress on_progress,
                                                     RunnerConfiguration runner_config)
    : timer_(timer),
      kinematics_client_(kinematics_client),
      grid_points_(std::move(grid_points)),
      on_failure_(std::move(on_failure)),
      on_complete_(std::move(on_complete)),
      on_progress_(std::move(on_progress)),
      runner_config_(runner_config) {}

void CalibrationSequenceRunner::Start() {
  if (grid_points_.empty()) {
    LOG_ERROR("No grid points provided");
    on_failure_();
    return;
  }

  busy_          = true;
  current_index_ = 0;
  observations_.clear();
  state_ = State::MOVING;

  MoveToCurrentPoint();

  CancelTimeout();
  timeout_task_id_ = timer_->Request(&CalibrationSequenceRunner::HandleTimeout, this,
                                     static_cast<uint64_t>(runner_config_.max_time_per_observation * MILLISEC_PER_SEC),
                                     "observation_timer");
}

void CalibrationSequenceRunner::MoveToCurrentPoint() {
  const auto& target = grid_points_[current_index_];

  // Check for diagonal move and insert intermediate point if needed.
  // The intermediate point means we first move vertically and then horizontally.
  // This reduces the risk to bump into the top corner of the groove.
  if (current_index_ > 0) {
    const auto& prev = grid_points_[current_index_ - 1];
    if (prev.index_x != target.index_x && prev.index_z != target.index_z) {
      GridPoint intermediate{.index_x = prev.index_x, .index_z = target.index_z, .layout = target.layout};

      auto insert_pos = grid_points_.begin();
      std::advance(insert_pos, current_index_);
      grid_points_.insert(insert_pos, intermediate);

      skip_next_observation_ = true;
    }
  }

  const auto& point = grid_points_[current_index_];
  kinematics_client_->SetSlidesPosition(point.GetX(), point.GetZ(), runner_config_.slide_velocity,
                                        runner_config_.slide_velocity);
}

void CalibrationSequenceRunner::OnScannerDataUpdate(const lpcs::Slice& data, const common::Point& axis_position) {
  if (data.confidence == lpcs::SliceConfidence::NO || !data.groove) {
    return;
  }

  if (!busy_ || current_index_ >= grid_points_.size()) {
    return;
  }

  const auto& target = grid_points_[current_index_];
  common::Point diff{.horizontal = (axis_position.horizontal - target.GetX()),
                     .vertical   = (axis_position.vertical - target.GetZ())};

  if (state_ == State::MOVING && NearTarget(diff)) {
    LOG_DEBUG("Near target at: h: {:.2f}, v: {:.2f}, start stabilization timer", target.GetX(), target.GetZ());
    state_ = State::STABILIZING;
    StartStabilizationTimer();

  } else if (state_ == State::STABLE) {
    const auto observation = Observation{.slide_position = axis_position, .abw_points_lpcs = data.groove.value()};
    RecordObservation(observation);
  }
}

void CalibrationSequenceRunner::StartStabilizationTimer() {
  stabilization_task_id_ = timer_->Request(&CalibrationSequenceRunner::OnStabilizationComplete, this,
                                           static_cast<uint64_t>(runner_config_.stabilization_time * MILLISEC_PER_SEC),
                                           "stabilization_delay");
}

void CalibrationSequenceRunner::OnStabilizationComplete() {
  LOG_DEBUG("Stabilization complete");
  state_                 = State::STABLE;
  stabilization_task_id_ = {};
}

void CalibrationSequenceRunner::RecordObservation(const Observation& observation) {
  if (!skip_next_observation_) {
    LOG_INFO("Recording observation at h: {:.2f}, v: {:.2f}", observation.slide_position.horizontal,
             observation.slide_position.vertical);
    observations_.push_back(observation);
  } else {
    LOG_INFO("Not recording observation, this point is for safe movement: h: {:.2f}, v: {:.2f}",
             observation.slide_position.horizontal, observation.slide_position.vertical);
    skip_next_observation_ = false;
  }

  ++current_index_;
  on_progress_(static_cast<double>(current_index_) / static_cast<double>(grid_points_.size()));

  if (current_index_ >= grid_points_.size()) {
    LOG_INFO("Grid measurement completed with {} observations.", observations_.size());
    CancelTimers();
    busy_  = false;
    state_ = State::IDLE;
    on_complete_(std::move(observations_));
  } else {
    state_ = State::MOVING;
    MoveToCurrentPoint();

    CancelTimeout();
    timeout_task_id_ = timer_->Request(
        &CalibrationSequenceRunner::HandleTimeout, this,
        static_cast<uint64_t>(runner_config_.max_time_per_observation * MILLISEC_PER_SEC), "observation_timer");
  }
}

void CalibrationSequenceRunner::HandleTimeout() {
  LOG_ERROR("Calibration point index {} timed out", current_index_);
  CancelTimers();
  busy_ = false;
  on_failure_();
}

void CalibrationSequenceRunner::CancelTimers() {
  CancelTimeout();
  if (stabilization_task_id_) {
    timer_->Cancel(*stabilization_task_id_);
    stabilization_task_id_ = {};
  }
}

void CalibrationSequenceRunner::CancelTimeout() {
  if (timeout_task_id_) {
    timer_->Cancel(*timeout_task_id_);
    timeout_task_id_ = {};
  }
}

auto CalibrationSequenceRunner::NearTarget(const common::Point& diff) const -> bool {
  return std::abs(diff.horizontal) < runner_config_.near_target_delta &&
         std::abs(diff.vertical) < runner_config_.near_target_delta;
}

auto CalibrationSequenceRunner::Busy() const -> bool { return busy_; }

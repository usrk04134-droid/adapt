#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "calibration/calibration_configuration.h"
#include "calibration_grid_generator.h"
#include "calibration_solver.h"
#include "common/zevs/zevs_socket.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "common/groove/point.h"

namespace calibration {

/*
 * Runs a calibration sequence by iterating through a set of GridPoints.
 * For each point, moves to the position, waits for scanner data, and collects observations.
 * Calls on_complete with all observations if successful, or on_failure on error or timeout.
 */
class CalibrationSequenceRunner {
 public:
  using OnFailure  = std::function<void()>;
  using OnComplete = std::function<void(std::vector<Observation>)>;
  using OnProgress = std::function<void(double)>;

  CalibrationSequenceRunner(zevs::Timer* timer, kinematics::KinematicsClient* kinematics_client,
                            std::vector<GridPoint> grid_points, OnFailure on_failure, OnComplete on_complete,
                            OnProgress on_progress, RunnerConfiguration sequence_config);

  void Start();
  void OnScannerDataUpdate(const lpcs::Slice& data, const common::groove::Point& axis_position);
  auto Busy() const -> bool;

 private:
  enum class State { IDLE, MOVING, STABILIZING, STABLE };

  void MoveToCurrentPoint();
  void StartStabilizationTimer();
  void OnStabilizationComplete();
  void RecordObservation(const Observation& observation);
  void HandleTimeout();
  void CancelTimers();
  void CancelTimeout();
  auto NearTarget(const macs::Point& diff) const -> bool;

  zevs::Timer* timer_;
  kinematics::KinematicsClient* kinematics_client_;
  std::vector<GridPoint> grid_points_;
  std::vector<Observation> observations_;
  OnFailure on_failure_;
  OnComplete on_complete_;
  OnProgress on_progress_;
  RunnerConfiguration runner_config_;

  size_t current_index_{0};
  bool busy_{false};
  State state_{State::IDLE};
  bool skip_next_observation_{false};

  std::optional<uint32_t> timeout_task_id_;
  std::optional<uint32_t> stabilization_task_id_;
};

}  // namespace calibration

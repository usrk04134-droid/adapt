#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace kinematics {

enum class State {
  INIT,
  HOMED,
};

struct StateChange {
  State weld_axis_state{State::INIT};
};

enum class EdgeState {
  NOT_AVAILABLE,
  AVAILABLE,
};

using OnGetSlidesPosition = std::function<void(std::uint64_t time_stamp, double horizontal, double vertical)>;
using OnGetSlidesStatus   = std::function<void(bool horizontal_in_position, bool vertical_in_position)>;
using OnGetWeldAxisData = std::function<void(std::uint64_t time_stamp, double position, double velocity, double radius,
                                             double linear_object_distance)>;
using OnStateChange     = std::function<void(const StateChange& data)>;
using OnGetEdgePosition = std::function<void(double position)>;
using EdgeStateSubscriber = std::function<void(const EdgeState& data)>;

class KinematicsClient {
 public:
  virtual ~KinematicsClient() = default;

  virtual void SetSlidesPosition(double horizontal, double vertical, double horizontal_lin_velocity,
                                 double vertical_velocity)                                  = 0;
  virtual void GetSlidesPosition(OnGetSlidesPosition on_response)                           = 0;
  virtual void GetSlidesPosition(std::uint64_t time_stamp, OnGetSlidesPosition on_response) = 0;
  virtual void GetSlidesStatus(OnGetSlidesStatus on_response)                               = 0;
  virtual void GetWeldAxisData(OnGetWeldAxisData on_response)                               = 0;
  virtual void GetWeldAxisData(std::uint64_t time_stamp, OnGetWeldAxisData on_response)     = 0;
  virtual void SetWeldAxisData(double velocity)                                             = 0;
  virtual void Release()                                                                    = 0;
  virtual auto SubscribeStateChanges(OnStateChange on_state_change, EdgeStateSubscriber edge_state_subscriber)
      -> uint32_t                                             = 0;
  virtual void UnSubscribeStateChanges(uint32_t handle)       = 0;
  virtual void GetEdgePosition(OnGetEdgePosition on_response) = 0;
};

auto inline StateToString(kinematics::State state) -> std::string {
  switch (state) {
    case kinematics::State::INIT:
      return "INIT";
    case kinematics::State::HOMED:
      return "HOMED";
    default:
      break;
  }
  return "invalid";
}

auto inline EdgeStateToString(kinematics::EdgeState state) -> std::string {
  switch (state) {
    case kinematics::EdgeState::NOT_AVAILABLE:
      return "not_available";
    case kinematics::EdgeState::AVAILABLE:
      return "available";
    default:
      break;
  }
  return "invalid";
}

}  // namespace kinematics

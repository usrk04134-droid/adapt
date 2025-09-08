#include "kinematics_axis.h"

#include <cfloat>
#include <cstdlib>
#include <string>

#include "common/logging/application_log.h"

using controller::simulation::KinematicsAxis;

KinematicsAxis::KinematicsAxis(const std::string& name) : name_(name) {}

void KinematicsAxis::Update(float time_step) {
  if (commands.execute && commands.stop) {
    LOG_ERROR("{} Invalid combination of execute & stop", name_);
    return;
  }

  if (!last_commands_.execute && commands.execute) {
    LOG_INFO("{}, Starting axis", name_);
  }

  if (!last_commands_.stop && commands.stop) {
    LOG_INFO("{}, Stopping axis", name_);
  }

  if (commands.stop) {
    status.active    = false;
    ordered_speed_   = 0.0;
    actual_velocity_ = 0.0;

    status.position = actual_position_;
    status.velocity = actual_velocity_;
    last_commands_  = commands;
    return;
  }

  if (commands.execute) {
    ordered_position_ = commands.position;
    ordered_speed_    = commands.speed;

    status.active = true;
  }

  auto distance_to_target = std::abs(ordered_position_ - actual_position_);

  if (status.active) {
    if (distance_to_target <= DBL_EPSILON) {
      if (!in_position_) {
        LOG_TRACE("{}, Position reached, {} - {} = {}", name_, ordered_position_, actual_position_, distance_to_target);
      }
      in_position_ = true;
    } else {
      in_position_ = false;
    }

    if (!in_position_) {
      if (ordered_position_ > actual_position_) {
        actual_velocity_ = std::abs(ordered_speed_);
      } else {
        actual_velocity_ = std::abs(ordered_speed_) * -1;
      }

      if (std::abs(actual_velocity_ * time_step) > std::abs(ordered_position_ - actual_position_)) {
        // ordered position is reached, will set in_position_ on next update
        actual_position_ = commands.position;
        actual_velocity_ = 0.0;
      } else {
        actual_position_ = actual_position_ + actual_velocity_ * time_step;
        LOG_TRACE("{}, Actual position is {}", name_, actual_position_);
      }
    } else {
      actual_velocity_ = 0.0;
    }
  }

  status.position = actual_position_;
  status.velocity = actual_velocity_;

  last_commands_ = commands;
}

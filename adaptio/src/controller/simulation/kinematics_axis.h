#pragma once
#include <string>

namespace controller::simulation {

class KinematicsAxis {
 public:
  explicit KinematicsAxis(const std::string& name);
  ~KinematicsAxis() = default;

  /**
   * Updates the axis simulation
   *
   * @param time_step The time since last call in seconds
   */
  void Update(float time_step);

  struct Commands {
    bool execute         = false;
    bool stop            = false;
    bool follow_position = false;
    float position       = 0.0;
    float speed          = 0.0;
  } commands;

  struct Status {
    bool active           = false;
    bool in_position      = false;
    bool follows_position = false;
    float position        = 0.0;
    float velocity        = 0.0;
  } status;

 private:
  struct Commands last_commands_;
  bool in_position_       = false;
  float actual_velocity_  = 0.0;
  float actual_position_  = 0.0;
  float ordered_position_ = 0.0;
  float ordered_speed_    = 0.0;
  std::string name_;
};

}  // namespace controller::simulation

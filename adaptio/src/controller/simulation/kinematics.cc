#include "kinematics.h"

#include <chrono>

using controller::simulation::Kinematics;

Kinematics::Kinematics() = default;

void Kinematics::Update() {
  using std::chrono::duration;
  using std::chrono::high_resolution_clock;
  using namespace std::chrono_literals;

  double const elapsed_time = duration<double>(high_resolution_clock::now() - last_update_).count();
  last_update_              = high_resolution_clock::now();

  x_axis_.commands = commands.x;
  y_axis_.commands = commands.y;
  z_axis_.commands = commands.z;
  a_axis_.commands = commands.a;

  x_axis_.Update(elapsed_time);
  y_axis_.Update(elapsed_time);
  z_axis_.Update(elapsed_time);
  a_axis_.Update(elapsed_time);

  status.x = x_axis_.status;
  status.y = y_axis_.status;
  status.z = z_axis_.status;
  status.a = a_axis_.status;
}

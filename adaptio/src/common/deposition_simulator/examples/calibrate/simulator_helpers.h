#pragma once

#include <fstream>
#include <vector>

#include "point3d.h"
#include "sim-config.h"
#include "simulator_interface.h"

namespace cal_example {

auto Configure(deposition_simulator::ISimulator *simulator) -> deposition_simulator::SimConfig;

auto MoveInGrid(deposition_simulator::ISimulator *simulator) -> void;

auto WriteToFile(std::vector<deposition_simulator::Point3d> &points, std::ofstream &file) -> void;

auto WriteLineToFile(std::string name, deposition_simulator::Point3d &point, std::ofstream &file) -> void;

auto WriteLineToFile(std::string name, std::vector<deposition_simulator::Point3d> &points, std::ofstream &file) -> void;

class ExampleListener : public deposition_simulator::ICollisionListener {
 private:
  bool has_collided_{false};

 public:
  ExampleListener()           = default;
  ~ExampleListener() override = default;
  auto CollisionDetected() const -> bool;
  auto Reset() -> void;
  auto OnTorchCollision(deposition_simulator::Point3d &hit_point) -> void override;
};

}  // Namespace cal_example

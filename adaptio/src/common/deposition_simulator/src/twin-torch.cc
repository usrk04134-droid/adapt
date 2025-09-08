#include "twin-torch.h"

#include <cmath>
#include <numbers>
#include <vector>

#include "point3d.h"

namespace deposition_simulator {

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
TwinTorch::TwinTorch(double wire_diam, double initial_wire_feed_speed)
    : wire_feed_speed_(initial_wire_feed_speed), wire_diameter_(wire_diam) {

      };

// TwinTorch::~TwinTorch() = default;

auto TwinTorch::GetVolumeDepositionRate() const -> double {
  return this->wire_feed_speed_ * std::numbers::pi * std::pow(wire_diameter_ / 2, 2);
}

auto TwinTorch::SetWireFeedSpeed(double wire_feed_speed) -> void { this->wire_feed_speed_ = wire_feed_speed; }

auto TwinTorch::GetWireFeedSpeed() const -> double { return this->wire_feed_speed_; }

auto TwinTorch::GetWireDiameter() const -> double { return wire_diameter_; }

auto TwinTorch::SetTorchGeometry(double tip_diameter, double base_diameter, double cone_length) -> void {
  this->tip_diameter_  = tip_diameter;
  this->base_diameter_ = base_diameter;
  this->cone_length_   = cone_length;
}

auto TwinTorch::GetVertices() const -> std::vector<Point3d> {
  std::vector<Point3d> vertices;
  // vertices.emplace_back()
  return vertices;
}

}  // Namespace deposition_simulator

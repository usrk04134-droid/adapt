#pragma once
#include <array>
#include <cstdint>

namespace scanner::joint_tracking {

enum class SliceConfidence { NO, LOW, MEDIUM, HIGH };

struct Coord {
  double x, z;
};

class JointSlice {
 public:
  JointSlice() = default;
  JointSlice(std::array<Coord, 7>& points, enum SliceConfidence confidence)
      : points_(points), confidence_(confidence) {};
  Coord GetTopLeftCorner() const { return points_.front(); }
  Coord GetTopRightCorner() const { return points_.back(); }
  Coord GetBottomLeftCorner() const { return points_[1]; }
  Coord GetElement(uint32_t index) const { return points_[index]; }
  Coord GetBottomRightCorner() const { return points_[points_.size() - 2]; }
  SliceConfidence GetConfidence() const { return confidence_; }
  double InterpolateZ(double x) const;

 private:
  std::array<Coord, 7> points_;
  [[maybe_unused]] SliceConfidence confidence_;
};
}  // namespace scanner::joint_tracking

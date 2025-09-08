

#include "slice-line.h"

#include <Eigen/Eigen>

#include "line2d.h"
#include "point2d.h"

namespace deposition_simulator {

auto SliceLine::GetJointPart() const -> JointPart { return this->jointPart_; }
auto SliceLine::SetJointPart(const JointPart& joint_part) -> void { this->jointPart_ = joint_part; }

// SliceLine* SliceLine::GetNextLine()
// {
//   return this->nextLine;
// }

// void SliceLine::SetNextLine(SliceLine* nextLine)
// {
//   this->nextLine = nextLine;
// }

SliceLine::SliceLine(const Point2d& start, const Eigen::Vector2d& dir, double len)
    : Line2d(start, dir, len), jointPart_(NOT_KNOWN) {}

SliceLine::SliceLine() : Line2d({0, 0}, {1, 0}, 0.0), jointPart_(NOT_KNOWN) {};

auto SliceLine::FromPoints(const Point2d& start, const Point2d& end) -> SliceLine {
  Eigen::Vector2d dir({end.GetX() - start.GetX(), end.GetY() - start.GetY()});
  const double length = dir.norm();
  dir.normalize();
  return {start, dir, length};
}

}  // namespace deposition_simulator

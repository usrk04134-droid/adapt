#pragma once

#include <sys/types.h>

#include <Eigen/Eigen>

#include "line2d.h"
#include "point2d.h"

namespace deposition_simulator {
enum JointPart : uint {
  NOT_KNOWN         = 0,
  TOP_SURFACE_LEFT  = 1,
  CHAMFER_LEFT      = 2,
  GROOVE_LEFT       = 4,
  ROOT_FACE_LEFT    = 8,
  ROOT_FACE_RIGHT   = 16,
  GROOVE_RIGHT      = 32,
  CHAMFER_RIGHT     = 64,
  TOP_SURFACE_RIGHT = 128,
  WELD_METAL        = 256,
  LEFT              = TOP_SURFACE_LEFT | CHAMFER_LEFT | GROOVE_LEFT | ROOT_FACE_LEFT,
  RIGHT             = TOP_SURFACE_RIGHT | CHAMFER_RIGHT | GROOVE_RIGHT | ROOT_FACE_RIGHT,
};

class SliceLine : public Line2d {
 private:
  // Point2d start_;
  // Eigen::Vector2d direction_;
  // double length_;
  JointPart jointPart_;
  // SliceLine* nextLine;

 public:
  static auto FromPoints(const Point2d &start, const Point2d &end) -> SliceLine;
  SliceLine();
  SliceLine(const Point2d &start, const Eigen::Vector2d &direction, double len);
  auto GetJointPart() const -> JointPart;
  auto SetJointPart(const JointPart &joint_part) -> void;
  // SliceLine* GetNextLine();
  // void SetNextLine(SliceLine* nextLine);
};

}  // namespace deposition_simulator

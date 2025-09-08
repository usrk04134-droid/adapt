#pragma once

#include <array>
#include <Eigen/Eigen>
#include <memory>

#include "circle2d.h"
#include "point2d.h"

namespace deposition_simulator {

class IntersectionPoint;

class Line2d {
 private:
  Point2d start_;
  Eigen::Vector2d direction_;
  double length_;
  Line2d(const Point2d &start, const Point2d &end);

 public:
  Line2d();
  Line2d(const Point2d &start, Eigen::Vector2d dir);
  Line2d(const Point2d &start, Eigen::Vector2d dir, double len);
  ~Line2d() = default;
  auto Reverse() -> void;
  auto GetStart() const -> Point2d;
  auto GetEnd() const -> Point2d;
  auto SetEnd(const Point2d &new_end) -> void;
  auto SetStart(const Point2d &new_start) -> void;
  auto ShiftLine(Eigen::Vector2d &offset) -> void;
  auto GetMinY() const -> double;
  auto GetMaxY() const -> double;
  auto GetMinX() const -> double;
  auto GetMaxX() const -> double;
  auto GetLength() const -> double;
  auto Intersect(const Line2d &other, bool consider_len_this, bool consider_len_other) const
      -> std::unique_ptr<Point2d>;
  auto Intersect(const Circle2d &circle, std::array<IntersectionPoint, 2> &intersection) -> int;
  auto Intersect(const Circle2d &circle, bool consider_len_start, bool consider_len_end)
      -> std::vector<IntersectionPoint>;
  static auto FromPoints(const Point2d &start, const Point2d &end) -> Line2d;
};

}  // namespace deposition_simulator

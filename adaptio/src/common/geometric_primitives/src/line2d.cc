

#include "line2d.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <Eigen/Eigen>
#include <optional>
#include <vector>

#include "circle2d.h"
#include "point2d.h"

namespace geometric_primitives {

const double DOT_PRODUCT_PARALLEL = 1e-12;
const double MIN_LINE_LENGTH      = 1e-6;

using Eigen::Vector2d;

auto Line2d::Reverse() -> void {
  const Point2d new_start = this->GetEnd();
  this->direction_        = -1 * this->direction_;
  this->start_            = new_start;
}

auto Line2d::SetEnd(const Point2d &end) -> void {
  // Set new implicit end point by modifying direction and length. Start point remains the same.
  Vector2d dir(end.GetX() - this->start_.GetX(), end.GetY() - this->start_.GetY());
  this->length_ = dir.norm();
  dir.normalize();
  this->direction_ = dir;
}

auto Line2d::SetStart(const Point2d &start) -> void {
  // Set new start point. Modifies start, direction and length. Implicit end point remains the same
  const Point2d end = this->GetEnd();
  Vector2d dir(end.GetX() - start.GetX(), end.GetY() - start.GetY());
  this->length_ = dir.norm();
  dir.normalize();
  this->start_     = start;
  this->direction_ = dir;
}

auto Line2d::GetStart() const -> Point2d { return this->start_; }

auto Line2d::GetEnd() const -> Point2d {
  const Eigen::Vector2d vec = this->direction_ * this->length_;
  return this->start_ + vec;
}

auto Line2d::GetMinY() const -> double { return std::min(this->GetEnd().GetY(), this->start_.GetY()); }

auto Line2d::GetMaxY() const -> double { return std::max(this->GetEnd().GetY(), this->start_.GetY()); }

auto Line2d::GetMinX() const -> double { return std::min(this->GetEnd().GetX(), this->start_.GetX()); }

auto Line2d::GetMaxX() const -> double { return std::max(this->GetEnd().GetX(), this->start_.GetX()); }

auto Line2d::GetLength() const -> double { return this->length_; }

auto Line2d::ShiftLine(Eigen::Vector2d &offset) -> void { this->start_ = this->start_ + offset; }

auto Line2d::Intersect(const Line2d &other, bool consider_len_this, bool consider_len_other) const
    -> std::optional<Point2d> {
  const Vector2d pp = this->start_.ToVector();
  const Vector2d qq = other.GetStart().ToVector();
  const Vector2d rr = this->direction_ * this->length_;
  const Vector2d ss = other.direction_ * other.length_;

  Eigen::Matrix<double, 2, 2> mm;
  mm << rr, ss;

  const double m_denom = mm.determinant();  // r(0)*s(1) - r(1)*s(0);

  if (std::abs(m_denom) < DOT_PRODUCT_PARALLEL) {
    return std::nullopt;  // No intersection, parallell lines.
  }

  mm << qq - pp, ss;
  const double tt = mm.determinant() / m_denom;
  mm << qq - pp, rr;
  const double uu = mm.determinant() / m_denom;

  if (consider_len_this)  // Check that intersection is within line1 (this)
  {
    if (tt < 0 || tt > 1) {
      return std::nullopt;
    }
  }

  if (consider_len_other)  // Check that intersection is within line2 (other)
  {
    if (uu < 0 || uu > 1) {
      return std::nullopt;
    }
  }

  Vector2d v_int = pp + tt * rr;
  Point2d intersection_point{v_int(0), v_int(1)};
  return intersection_point;
}

auto Line2d::Intersect(const Circle2d &circle, bool consider_len_start = false, bool consider_len_end = false)
    -> std::vector<Point2d> {
  std::vector<Point2d> intersections;
  const Vector2d cc(circle.GetCenter().GetX(), circle.GetCenter().GetY());
  Vector2d bb = this->direction_;
  Vector2d aa(this->start_.GetX(), this->start_.GetY());
  const double rr = circle.GetRadius();

  if (this->length_ < MIN_LINE_LENGTH) {
    return intersections;
  }

  // Parameteric line v(t) = a + t*b
  const double dd =
      pow(2 * (aa - cc).dot(bb), 2) - 4 * pow(bb.norm(), 2) * (pow((aa - cc).norm(), 2) - pow(rr, 2));  // NOLINT

  // if (dd < 0) {  // No intersection
  //   return 0;
  // }
  // A line and a circle can intersect at 0, 1 or 2 points. Check which is the case. Return accordingly.
  const double t1 = (-2 * (aa - cc).dot(bb) + std::sqrt(dd)) / (2 * pow(bb.norm(), 2));
  const double t2 = (-2 * (aa - cc).dot(bb) - std::sqrt(dd)) / (2 * pow(bb.norm(), 2));

  if ((!consider_len_start || t1 >= 0) && (!consider_len_end || t1 <= this->length_)) {
    intersections.emplace_back(aa(0) + t1 * bb(0), aa(1) + t1 * bb(1));  // NOLINT
  }
  if ((!consider_len_start || t2 >= 0) && (!consider_len_end || t2 <= this->length_)) {
    intersections.emplace_back(aa(0) + t2 * bb(0), aa(1) + t2 * bb(1));  // NOLINT
  }

  return intersections;
}

auto Line2d::FromPoints(const Point2d &start, const Point2d &end) -> Line2d { return {start, end}; }

Line2d::Line2d(const Point2d &start, const Point2d &end) : start_(start), direction_({0, 0}), length_(0) {
  Eigen::Vector2d dir({end.GetX() - start.GetX(), end.GetY() - start.GetY()});
  this->length_ = dir.norm();
  dir.normalize();
  this->direction_ = dir;
  // this->start_     = start;
}

Line2d::Line2d(const Point2d &start, Eigen::Vector2d dir) : start_(start), direction_(dir), length_(dir.norm()) {
  // this->length_ = dir.norm();
  dir.normalize();
  // this->direction_ = dir;
}
Line2d::Line2d(const Point2d &start, Eigen::Vector2d dir, double len) : start_(start), direction_(dir), length_(len) {
  dir.normalize();
}
Line2d::Line2d() : start_({0, 0}), direction_({0, 0}), length_(0) {}

}  // namespace geometric_primitives

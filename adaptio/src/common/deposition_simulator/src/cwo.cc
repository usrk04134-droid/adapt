
#include "cwo.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <Eigen/Eigen>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "../point3d.h"
#include "../triangle3d.h"
#include "joint-slice.h"
#include "line3d.h"
#include "plane3d.h"
#include "point2d.h"
#include "sim-config.h"

namespace deposition_simulator {

CircularWeldObject::CircularWeldObject(SimConfig sim_config)
    : sim_config_(sim_config), nbr_abw_points_(sim_config.nbr_abw_points) {}

// Adds a slice and computes additional surfaceelements and extends abw curves
auto CircularWeldObject::PushSlice(const JointSlice &slice) -> void { this->slices_.push_back(slice); }

auto CircularWeldObject::Reset() -> void { this->slices_.clear(); }

auto CircularWeldObject::GetTorchPlaneAngle() const -> double { return this->torch_plane_angle_; }
auto CircularWeldObject::SetTorchPlaneAngle(double angle) -> void { this->torch_plane_angle_ = angle; }

auto CircularWeldObject::GetSlice(int index) -> JointSlice * { return &this->slices_[index]; }

auto CircularWeldObject::MoveToNextSlice() -> JointSlice * {
  size_t slice_index = this->head_slice_index_;
  if (slice_index == this->slices_.size() - 1) {
    slice_index = 0;
  } else {
    slice_index++;
  }

  this->head_slice_index_ = slice_index;
  return &this->slices_.at(slice_index);
}

auto CircularWeldObject::MoveToPrevSlice() -> JointSlice * {
  size_t slice_index = this->head_slice_index_;
  if (slice_index == 0) {
    slice_index = this->slices_.size() - 1;
  } else {
    slice_index--;
  }

  this->head_slice_index_ = slice_index;
  return &this->slices_.at(slice_index);
}

auto CircularWeldObject::AddJointBottom(double curv_radius, double joint_depth, int nbr_arc_points) -> void {
  for (JointSlice &slice : this->slices_) {
    slice.AddJointBottom(joint_depth, curv_radius, nbr_arc_points);
  }
}

// NOLINTNEXTLINE(readability*)
auto CircularWeldObject::GetAbwPointsInPlane(const Plane3d &plane_rocs, const Point3d &closest_point_filter_rocs,
                                             bool allow_cap_points) const -> std::vector<std::optional<Point3d>> {
  if (plane_rocs.GetRefSystem() != ROCS) {
    throw std::runtime_error("Plane not defined in ROCS");
  }

  std::unordered_map<int, Point3d> abw_found{};

  Line3d line;
  const Point3d start;
  const Point3d end;
  const Vector3d dir;
  std::unique_ptr<std::vector<std::optional<Point2d>>> slice_abw_points;
  std::vector<std::optional<Point3d>> end_abw_points;
  std::vector<std::optional<Point3d>> start_abw_points;
  std::unique_ptr<Point3d> p_int;
  // std::vector<Line3d> abwLines;
  Eigen::Matrix3d rotmat;
  JointSlice slice;
  Vector3d startpos_slice;
  Vector3d startpos_rocs;
  Vector3d endpos_slice;
  Vector3d endpos_rocs;
  const size_t n_slices = this->slices_.size();
  bool got_first        = false;
  // size_t slice_index    = 0;
  size_t real_index = 0;

  for (size_t si = 0; si <= this->slices_.size(); si++) {
    real_index = si % n_slices;
    slice      = this->slices_[real_index];

    slice_abw_points = slice.GetAbwPoints(allow_cap_points);
    rotmat           = Eigen::AngleAxisd(slice.GetSliceAngle(), Vector3d(1, 0, 0)).toRotationMatrix();

    if (slice_abw_points == nullptr) {
      throw std::runtime_error(
          "Could not compute abw points for slice");  // TODO(zachjz): Consider changing this from throwing
                                                      // to returning nothing... or something.
    }

    if (!got_first) {
      // Convert 2D slice -> 3D, rotate and store in vector.
      for (auto &it : *slice_abw_points) {
        if (it.has_value()) {
          startpos_slice = Vector3d(it->GetX(), 0, it->GetY());
          startpos_rocs  = rotmat * startpos_slice;
          Point3d start{startpos_rocs(0), startpos_rocs(1), startpos_rocs(2), ROCS};
          start_abw_points.push_back(start);
        } else {
          start_abw_points.push_back(std::nullopt);
        }
      }

      got_first = true;
      // slice_index++;
      continue;
    }

    // Convert 2D slice -> 3D, rotate and store in vector.
    end_abw_points.clear();
    for (auto &it : *slice_abw_points) {
      if (it.has_value()) {
        endpos_slice = Vector3d(it->GetX(), 0, it->GetY());
        endpos_rocs  = rotmat * endpos_slice;
        Point3d end{endpos_rocs(0), endpos_rocs(1), endpos_rocs(2), ROCS};
        end_abw_points.push_back(end);
      } else {
        end_abw_points.push_back(std::nullopt);
      }
    }
    // std::cout << endPos_rocs(1) << "," << endPos_rocs(2) << std::endl;

    // Construct interpolation lines between slices and check for intersection with plane.
    for (int i = 0; i < end_abw_points.size(); i++) {
      if (!start_abw_points.at(i).has_value() || !end_abw_points.at(i).has_value()) {
        continue;
      }

      line  = Line3d::FromPoints(start_abw_points[i].value(), end_abw_points[i].value());
      p_int = line.Intersect(plane_rocs, true);

      // std::cout << i << "," << line.GetStart().GetX() << "," << line.GetEnd().GetX() << "," << line.GetStart().GetY()
      // << "," << line.GetEnd().GetY() << "," << line.GetStart().GetZ() << "," << line.GetEnd().GetZ() << std::endl;

      if (p_int == nullptr) {
        continue;
      }  // Plane is not between start and end slice for this particular ABW[x]

      // std::cout << "Intersection for slice " << si <<std::endl;

      // Since the plane intersects the "abw circle" at two points, check to see which of the intersection is the
      // one of interest.
      if (abw_found.contains(i)) {
        // std::cout << "Key found" << std::endl;
        //  Check distance to lpcs origin to determine if this is the closer of the two possible ABWx points.
        // TODO(zachjz): change this to instead use the dot product to determine direction of intersection
        if (std::abs(abw_found[i].GetZ() - closest_point_filter_rocs.GetZ()) >
            std::abs(p_int->GetZ() - closest_point_filter_rocs.GetZ())) {
          abw_found[i] = *p_int;
          // std::cout << "Updating abw found" << std::endl;
        }
        // continue;
      } else {
        abw_found[i] = *p_int;
        // std::cout << "Adding to abw found" << std::endl;
      }
    }

    // if (abwFound.size() == nbrAbw)
    //   break;

    start_abw_points = end_abw_points;
    // slice_index++;
  }

  // if (abw_found.size() != nbr_abw_points_) {
  //   throw std::runtime_error("Did not find the specified number of ABW points.");
  // }

  std::vector<std::optional<Point3d>> abw;
  abw.reserve(nbr_abw_points_);
  for (int i = 0; i < nbr_abw_points_; i++) {
    if (abw_found.contains(i)) {
      abw.emplace_back(abw_found.at(i));
    } else {
      abw.emplace_back(std::nullopt);
    }
  }

  return abw;
}

// Returns the distance from the center to the surface of the weld object at a
// given cylindrical coordinate.
auto CircularWeldObject::GetSurfaceDistance(double x_coord, double slice_angle) const -> double {  // NOLINT
  double r_left               = this->sim_config_.joint_def_left.outer_diameter / 2;
  double r_right              = this->sim_config_.joint_def_right.outer_diameter / 2;
  const double z_offset_left  = this->sim_config_.joint_def_left.radial_offset;
  const double z_offset_right = this->sim_config_.joint_def_right.radial_offset;
  double long_weld_addition   = NAN;

  Eigen::Vector2d surface_point_vec;
  Eigen::Vector2d z_offset;
  Eigen::Vector2d radial_vec;

  if (x_coord >= 0) {
    long_weld_addition  = JointSlice::ComputeLongWeldAddition(slice_angle, this->sim_config_.joint_def_left);
    r_left             += long_weld_addition;
    radial_vec          = {-std::sin(slice_angle), std::cos(slice_angle)};
    radial_vec          = r_left * radial_vec;
    z_offset            = {0, z_offset_left};

  } else {
    long_weld_addition  = JointSlice::ComputeLongWeldAddition(slice_angle, this->sim_config_.joint_def_right);
    r_right            += long_weld_addition;
    radial_vec          = {-std::sin(slice_angle), std::cos(slice_angle)};
    radial_vec          = r_right * radial_vec;
    z_offset            = {0, z_offset_right};
  }

  surface_point_vec = z_offset + radial_vec;

  return surface_point_vec.norm();
}

auto CircularWeldObject::GetMinX() const -> double {
  double x_min = std::numeric_limits<double>::infinity();

  for (const auto &slice : this->slices_) {
    x_min = std::min(x_min, slice.GetMinX());
  }

  return x_min;
}

auto CircularWeldObject::GetMaxX() const -> double {
  double x_max = -std::numeric_limits<double>::infinity();

  for (const auto &slice : this->slices_) {
    x_max = std::max(x_max, slice.GetMaxX());
  }

  return x_max;
}

auto CircularWeldObject::GetMaxRadius() const -> double {
  double r_max = -std::numeric_limits<double>::infinity();

  for (const auto &slice : this->slices_) {
    r_max = std::max(r_max, slice.GetMaxY());
  }

  return r_max;
}

auto CircularWeldObject::ToTriangleMesh() const -> std::vector<Triangle3d> {
  std::vector<Triangle3d> mesh;
  std::vector<Point3d> curr_slice_points;
  std::vector<Point3d> prev_slice_points;

  for (const auto &slice : this->slices_) {
    curr_slice_points = slice.GetSlicePoints();

    if (prev_slice_points.empty()) {
      prev_slice_points = curr_slice_points;
      continue;
    }

    for (int i = 0; i < curr_slice_points.size() - 1; i++) {
      mesh.emplace_back(prev_slice_points.at(i), curr_slice_points.at(i), curr_slice_points.at(i + 1));
      mesh.emplace_back(prev_slice_points.at(i), curr_slice_points.at(i + 1), prev_slice_points.at(i + 1));
    }
  }

  return mesh;
}

}  // namespace deposition_simulator

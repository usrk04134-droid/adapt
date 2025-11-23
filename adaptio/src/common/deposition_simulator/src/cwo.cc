
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

  const size_t n_slices = slices_.size();
  const int N           = nbr_abw_points_;

  // Faster than unordered_map for small fixed size keys (0..N)
  std::vector<std::optional<Point3d>> abw_found(N);

  // Storage for slice start/end
  std::vector<std::optional<Point3d>> start_abw(N);
  std::vector<std::optional<Point3d>> end_abw(N);

  std::vector<std::optional<Point2d>> slice_abw_points;
  slice_abw_points.reserve(N);

  Eigen::Matrix3d rotmat;

  bool got_first = false;

  for (size_t si = 0; si <= n_slices; si++) {
    const JointSlice &slice = slices_[si == n_slices ? 0 : si];

    // Get points from slice
    slice_abw_points = slice.GetAbwPoints(allow_cap_points);

    // Create the rotation transform to go from slice CS to ROCS
    rotmat = Eigen::AngleAxisd(slice.GetSliceAngle(), Vector3d(1, 0, 0)).toRotationMatrix();

    // First slice → Just fill start_abw
    if (!got_first) {
      // Convert 2D slice -> 3D, rotate and store in vector.
      int idx = 0;
      for (const auto &pt2d : slice_abw_points) {
        if (pt2d.has_value()) {
          const Vector3d p_slice(pt2d->GetX(), 0.0, pt2d->GetY());
          const Vector3d p_rocs = rotmat * p_slice;

          start_abw[idx].emplace(p_rocs.x(), p_rocs.y(), p_rocs.z(), ROCS);
        } else {
          start_abw[idx].reset();
        }
        idx++;
      }

      got_first = true;
      continue;
    }

    // Convert 2D slice -> 3D, rotate and store in vector.
    int idx_end = 0;
    for (const auto &pt2d : slice_abw_points) {
      if (pt2d.has_value()) {
        const Vector3d p_slice(pt2d->GetX(), 0.0, pt2d->GetY());
        const Vector3d p_rocs = rotmat * p_slice;

        end_abw[idx_end].emplace(p_rocs.x(), p_rocs.y(), p_rocs.z(), ROCS);
      } else {
        end_abw[idx_end].reset();
      }
      idx_end++;
    }

    // Construct interpolation lines between slices and check for intersection with plane.
    for (int i = 0; i < N; i++) {
      // skip missing ABW points
      if (!start_abw[i].has_value() || !end_abw[i].has_value()) continue;

      Line3d line = Line3d::FromPoints(*start_abw[i], *end_abw[i]);

      std::unique_ptr<Point3d> p_int = line.Intersect(plane_rocs, true);
      if (!p_int) continue;  // Plane is not between start and end slice for this particular ABW[x]

      // Since the plane intersects the "abw circle" at two points,
      // check to see which of the intersection is the one of interest.
      if (abw_found[i].has_value()) {
        // Check distance to lpcs origin to determine if this is the closer of the two possible ABWx points.
        const double old_d = std::abs(abw_found[i]->GetZ() - closest_point_filter_rocs.GetZ());
        const double new_d = std::abs(p_int->GetZ() - closest_point_filter_rocs.GetZ());

        if (new_d < old_d) abw_found[i] = *p_int;

      } else {
        abw_found[i] = *p_int;
      }
    }

    // Move end slice to start slice for next iteration
    start_abw = end_abw;
  }

  // Final output
  std::vector<std::optional<Point3d>> result;
  result.reserve(N);

  for (const auto &pt : abw_found) {
    result.emplace_back(pt);
  }

  return result;
}

auto CircularWeldObject::GetLatestDepositedSlice(double wire_tip_angle) const -> std::vector<Point3d> {
  double slice_angle = 0.0;
  double curr_delta_cos{NAN};
  double max_delta_cos{-INFINITY};
  double curr_determinant{NAN};

  // Normalized torch direction in ROCS yz plane
  double t_y = -std::sin(wire_tip_angle);
  double t_z = std::cos(wire_tip_angle);

  // Normalized slice direction in ROCS yz plane
  double s_y{NAN};
  double s_z{NAN};

  const size_t n_slices       = this->slices_.size();
  size_t real_index           = 0;
  size_t idx_of_closest_slice = 0;

  for (size_t si = 0; si <= n_slices; si++) {
    real_index       = si % n_slices;
    slice_angle      = this->slices_[real_index].GetSliceAngle();
    s_y              = -std::sin(slice_angle);
    s_z              = std::cos(slice_angle);
    curr_delta_cos   = t_y * s_y + t_z * s_z;  // scalar prod <=> cos of angle
    curr_determinant = t_y * s_z - t_z * s_y;  // Sign gives which vector leads/trails

    // If determinant > 0 then s is less than 180 ahead of t
    // If determinant < 0 then s is less than 180 behind t
    // Together with the scalar product this means that we can calculate
    // how many degrees has passed since a slice s passed the torch.
    // I.e. if determinant < 0 then the angle = 360 - sp_angle
    // otherwise the angle = sp_angle

    if (curr_determinant > 0 && curr_delta_cos > max_delta_cos) {
      max_delta_cos        = curr_delta_cos;
      idx_of_closest_slice = real_index;
    }
  }

  return this->slices_[idx_of_closest_slice].GetSlicePoints();
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

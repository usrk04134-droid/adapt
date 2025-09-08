#pragma once

#include <vector>

#include "../point3d.h"
#include "../triangle3d.h"
#include "joint-slice.h"
#include "line3d.h"
#include "plane3d.h"
#include "sim-config.h"

namespace deposition_simulator {

const int DEFAULT_NBR_ABW_POINTS = 7;

struct BeadSpec {
  double bead_area;
  double bead_radius;
};

class CircularWeldObject {
 private:
  SimConfig sim_config_;
  int nbr_abw_points_{DEFAULT_NBR_ABW_POINTS};
  // double current_rot_angle_{0.0};
  double torch_plane_angle_{0.0};
  size_t head_slice_index_{0};
  std::vector<JointSlice> slices_;

 public:
  explicit CircularWeldObject(SimConfig sim_config);
  ~CircularWeldObject() = default;
  auto PushSlice(const JointSlice &slice)
      -> void;  // Adds a slice and computes additional surfaceelements and extends abw curves
  auto Reset() -> void;
  auto GetTorchPlaneAngle() const -> double;
  auto SetTorchPlaneAngle(double angle) -> void;
  auto GetSlice(int index) -> JointSlice *;
  auto MoveToNextSlice() -> JointSlice *;
  auto MoveToPrevSlice() -> JointSlice *;
  auto AddJointBottom(double curv_radius, double joint_depth, int nbr_arc_points) -> void;
  // auto GetAbwCurve(int abw_index) const -> std::vector<Line3d>;
  auto GetAbwPointsInPlane(const Plane3d &plane_rocs, const Point3d &closest_point_filter_rocs,
                           bool allow_cap_points = false) const -> std::vector<std::optional<Point3d>>;
  auto GetSurfaceDistance(double x_coord, double slice_angle) const -> double;
  auto GetMinX() const -> double;
  auto GetMaxX() const -> double;
  auto GetMaxRadius() const -> double;
  auto ToTriangleMesh() const -> std::vector<Triangle3d>;
};
}  // namespace deposition_simulator

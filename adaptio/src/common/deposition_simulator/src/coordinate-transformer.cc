
#include "coordinate-transformer.h"

#include <boost/proto/traits.hpp>
#include <cerrno>
#include <cmath>
#include <Eigen/Eigen>
#include <numbers>
#include <optional>
#include <stdexcept>

#include "../point3d.h"
#include "../sim-config.h"
#include "circle3d.h"
#include "plane3d.h"
#include "point2d.h"

namespace deposition_simulator {

CoordinateTransformer::CoordinateTransformer(LpcsConfig &lpcs_config, OpcsConfig &opcs_config,
                                             WeldMovementType weld_movement_type)
    : weld_movement_type_(weld_movement_type), lpcs_config_(lpcs_config), opcs_config_(opcs_config) {
  this->opcs_rel_to_rocs_ = Eigen::Matrix4d::Zero();
}

CoordinateTransformer::CoordinateTransformer() : lpcs_config_({}), opcs_config_({}) {
  this->opcs_rel_to_rocs_ = Eigen::Matrix4d::Zero();
};

auto CoordinateTransformer::SetWeldObjectOrientation(Matrix4d &opcs_rel_to_rocs) -> void {
  this->opcs_rel_to_rocs_ = Eigen::Matrix4d(opcs_rel_to_rocs);
}

auto CoordinateTransformer::ProjectToSlicePlane(const Point3d &point_any) const -> Point2d {
  // This transformation is independent of how the slice is rotated around ROCS x-axis.
  const Point3d point_rocs = this->Transform(point_any, ROCS);

  // Change to cylindrical coordinates. Drop the angle --> project to slice plane.
  const double radius  = sqrt(pow(point_rocs.GetZ(), 2) + pow(point_rocs.GetY(), 2));
  const double x_coord = point_rocs.GetX();

  return {x_coord, radius};
}

// Project a point on a plane with the projection given by the current weld movement mode (longitudinal or
// circumferential)
auto CoordinateTransformer::DoWeldMovementProjectionToPlane(const Point3d &point_any, const Plane3d &plane_any,
                                                            CoordinateSystem ref_system) -> std::optional<Point3d> {
  auto point_rocs = this->Transform(point_any, ROCS);
  auto plane_rocs = this->Transform(plane_any, ROCS);

  if (weld_movement_type_ == CIRCUMFERENTIAL) {
    // Do projection by converting to cylindrical coordinates in ROCS
    double dist_to_axis = std::sqrt(point_rocs.GetY() * point_rocs.GetY() + point_rocs.GetZ() * point_rocs.GetZ());
    Point3d center_point_rocs = Point3d::FromVector(ORIGIN, ROCS);
    center_point_rocs.SetX(point_rocs.GetX());
    auto projection_circle_rocs   = Circle3d(ROTATION_AXIS_ROCS, dist_to_axis, center_point_rocs);
    auto intersection_points_rocs = projection_circle_rocs.Intersect(plane_rocs);

    // Return intersection point closest to original (non-projected) point
    if (intersection_points_rocs.size() == 2) {
      size_t point_idx = 0;
      if (intersection_points_rocs.at(0).DistanceTo(point_rocs) >
          intersection_points_rocs.at(1).DistanceTo(point_rocs)) {
        point_idx = 1;
      }

      return Transform(intersection_points_rocs.at(point_idx), ref_system);
    }

  } else if (weld_movement_type_ == LONGITUDINAL) {
    // TODO(zachjz): Implement for linear motion. ==> project along movement direction.
    // Movement should always be in MACS y.
    // OPCS rel. to MACS can be configured as for circumferential.
    // ROCS rel. to OPCS determined by object positioner.
    // Misaligned joint modelled by OPCS-MACS relationship,
  }

  return std::nullopt;  // No intersection, projection undefined.
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto CoordinateTransformer::InternalTransform(Vector4d &vec, CoordinateSystem orig_system,
                                              CoordinateSystem target_system, bool pure_rotation) const -> Vector3d {
  Matrix4d transform_from_macs;
  Matrix4d transform_to_macs;
  Vector4d vec_macs;
  Vector4d vec_target;
  const Point3d transformed_point;

  if (orig_system == target_system) {
    return vec.block<3, 1>(0, 0);
  }

  // First transform to MACS from current ref system
  switch (orig_system) {
    // case CLCS: {
    //   transform_to_macs = GetMacsToClcsTransform().inverse();
    //   vec_macs          = transform_to_macs * vec;
    //   break;
    // }
    case MACS: {
      vec_macs = vec;
      break;
    }
    case LPCS: {
      transform_to_macs = GetMacsToLpcsTransform(pure_rotation).inverse();
      vec_macs          = transform_to_macs * vec;
      break;
    }
    case ROCS: {
      transform_to_macs = GetMacsToRocsTransform(pure_rotation).inverse();
      vec_macs          = transform_to_macs * vec;
      break;
    }
    case TCS: {
      transform_to_macs = GetMacsToTcsTransform(pure_rotation).inverse();
      vec_macs          = transform_to_macs * vec;
      break;
    }
    case OPCS: {
      transform_to_macs = GetMacsToOpcsTransform(pure_rotation).inverse();
      vec_macs          = transform_to_macs * vec;
      break;
    }
    default:
      throw std::runtime_error("Unsupported coordinate transformation.");
  }

  // Then go from MACS to target ref system
  switch (target_system) {
    // case CLCS: {
    //   transform_from_macs = GetMacsToClcsTransform();
    //   vec_target          = transform_from_macs * vec_macs;
    //   break;
    // }
    case MACS: {
      vec_target = vec_macs;
      break;
    }
    case ROCS: {
      transform_from_macs = GetMacsToRocsTransform(pure_rotation);
      vec_target          = transform_from_macs * vec_macs;
      break;
    }
    case LPCS: {
      transform_from_macs = GetMacsToLpcsTransform(pure_rotation);
      vec_target          = transform_from_macs * vec_macs;
      break;
    }
    case TCS: {
      transform_from_macs = GetMacsToTcsTransform(pure_rotation);
      vec_target          = transform_from_macs * vec_macs;
      break;
    }
    case OPCS: {
      transform_from_macs = GetMacsToOpcsTransform(pure_rotation);
      vec_target          = transform_from_macs * vec_macs;
      break;
    }
    default: {
      throw std::runtime_error("Unsupported coordinate transformation.");
    }
  }

  return vec_target.block<3, 1>(0, 0);
}

auto CoordinateTransformer::Transform(const Plane3d &plane, CoordinateSystem target_system) const -> Plane3d {
  Vector4d normal              = plane.GetHomNormal();
  const Vector3d normal_target = InternalTransform(normal, plane.GetRefSystem(), target_system, true);
  Vector4d point               = plane.GetPointInPlane().ToHomVec();
  Vector3d point_target        = InternalTransform(point, plane.GetRefSystem(), target_system, false);
  return {normal_target, Point3d(point_target(0), point_target(1), point_target(2), target_system)};
}

auto CoordinateTransformer::Transform(const Point3d &orig_point, CoordinateSystem target_system) const -> Point3d {
  Vector4d vec        = orig_point.ToHomVec();
  Vector3d vec_target = InternalTransform(vec, orig_point.GetRefSystem(), target_system, false);
  return {vec_target(0), vec_target(1), vec_target(2), target_system};
}

// auto CoordinateTransformer::GetMacsToClcsTransform() const -> Matrix4d  // MACS to CLCS
// {
//   Vector3d rot_axis(1, 0, 0);  // Not used -- no rotation for this transform
//   // TODO(zachjz): Must change the translation since this will depend on roller bed and object shape.
//   Vector3d clcs_translation_from_macs(macs_config_.x, macs_config_.y, macs_config_.z);
//   Matrix4d transform;
//   transform = CreateTransform(rot_axis, NAN, clcs_translation_from_macs, false);
//   transform = transform.inverse().eval();
//   return transform;
// }

auto CoordinateTransformer::GetMacsToOpcsTransform(bool pure_rotation) const -> Matrix4d {
  Vector3d rot_axis{1.0, 0.0, 0.0};                                         // Not used
  Vector3d translation{-opcs_config_.x, -opcs_config_.y, -opcs_config_.z};  // T_9->1

  Matrix4d transform;
  transform = CreateTransform(rot_axis, NAN, translation, pure_rotation);  // M_9->1
  return transform;                                                        // M_9->1
}

auto CoordinateTransformer::GetMacsToRocsTransform(bool pure_rotation) const -> Matrix4d  // MACS to ROCS
{
  Matrix4d macs_to_opcs;  // M_9->1
  macs_to_opcs = GetMacsToOpcsTransform(pure_rotation);

  Matrix4d opcs_to_rocs(this->opcs_rel_to_rocs_);  // M_2->9
  // opcs_to_rocs = this->opcs_rel_to_rocs_;

  if (pure_rotation) {
    opcs_to_rocs(0, 3) = 0.0;
    opcs_to_rocs(1, 3) = 0.0;
    opcs_to_rocs(2, 3) = 0.0;
  }

  Matrix4d macs_to_rocs;
  // M_2->1     = M_2->9       * M_9->1
  macs_to_rocs = opcs_to_rocs * macs_to_opcs;

  return macs_to_rocs.eval();
  // Vector3d rot_axis(1, 0, 0);
  // const double rot_ang = this->weld_object_rotation_;
  // Vector3d rocs_translation_from_macs(macs_config_.x, macs_config_.y, macs_config_.z);
  // Matrix4d transform;
  // transform = CreateTransform(rot_axis, rot_ang, rocs_translation_from_macs, pure_rotation);
  // transform =
  // transform.inverse().eval(); return transform;
}

auto CoordinateTransformer::GetMacsToLpcsTransform(bool pure_rotation) const -> Matrix4d  // MACS to LPCS
{
  const Matrix4d transform_tcs2macs = GetMacsToTcsTransform(pure_rotation).inverse();

  // LPCS is rotated and translated relative to TCS. Rotation is composed of two fundamental rotations. Extrisnic
  // convention. Starting from TCS. Rz -> Rx
  Vector3d translation(0.0, 0.0, 0.0);
  Vector3d rot_axis(0.0, 0.0, 1.0);
  const Matrix4d rot_z = CreateTransform(rot_axis, std::numbers::pi, translation, pure_rotation);
  rot_axis             = {1.0, 0.0, 0.0};
  const Matrix4d rot_x =
      CreateTransform(rot_axis, (-std::numbers::pi / 2) - this->lpcs_config_.alpha, translation, pure_rotation);

  // Translation obtained from calibration
  translation                         = Vector3d(lpcs_config_.x, lpcs_config_.y, lpcs_config_.z);
  const Matrix4d translation_lpcs2tcs = CreateTransform(rot_axis, NAN, translation, pure_rotation);
  const Matrix4d transform_lpcs2tcs   = translation_lpcs2tcs * (rot_x * rot_z).transpose();
  return (transform_tcs2macs * transform_lpcs2tcs).inverse().eval();
}

auto CoordinateTransformer::GetMacsToTcsTransform(bool pure_rotation) const
    -> Matrix4d  // MACS to TCS (slide move since zeroing)
{
  Vector3d rot_axis    = {0, 0, 0};
  Vector3d translation = -this->torchpos_macs_;
  Matrix4d transform =
      CreateTransform(rot_axis, NAN, translation, pure_rotation);  // Note the minus sign! I.e. the inverse translation.
  return transform;
}

auto CoordinateTransformer::CreateTransform(Vector3d &rot_axis, double rot_ang, Vector3d &translation,
                                            bool pure_rotation) -> Matrix4d {
  rot_axis.normalize();
  Matrix4d transform;
  transform = Matrix4d::Zero();

  double factor = 0.0;
  if (!std::isnan(rot_ang)) {
    factor = 1.0 - std::cos(rot_ang);

    // This can be initilized much easier in Eigen. Doing this way to ensure that definitions are exactly the same as
    // in Scilab simulator. This definition of the transformation is equivalent to T = Trans4D * Rotation4D.
    transform(0, 0) = factor * std::pow(rot_axis(0), 2) + std::cos(rot_ang);
    transform(0, 1) = factor * rot_axis(0) * rot_axis(1) - rot_axis(2) * std::sin(rot_ang);
    transform(0, 2) = factor * rot_axis(0) * rot_axis(2) + rot_axis(1) * std::sin(rot_ang);
    transform(1, 0) = factor * rot_axis(0) * rot_axis(1) + rot_axis(2) * std::sin(rot_ang);
    transform(1, 1) = factor * std::pow(rot_axis(1), 2) + std::cos(rot_ang);
    transform(1, 2) = factor * rot_axis(1) * rot_axis(2) - rot_axis(0) * std::sin(rot_ang);
    transform(2, 0) = factor * rot_axis(2) * rot_axis(0) - rot_axis(1) * std::sin(rot_ang);
    transform(2, 1) = factor * rot_axis(2) * rot_axis(1) + rot_axis(0) * std::sin(rot_ang);
    transform(2, 2) = factor * std::pow(rot_axis(2), 2) + std::cos(rot_ang);
  } else {
    transform(0, 0) = 1.0;
    transform(1, 1) = 1.0;
    transform(2, 2) = 1.0;
  }

  if (!pure_rotation) {
    transform(0, 3) = translation(0);
    transform(1, 3) = translation(1);
    transform(2, 3) = translation(2);
  }
  transform(3, 0) = 0.0;
  transform(3, 1) = 0.0;
  transform(3, 2) = 0.0;
  transform(3, 3) = 1.0;

  return transform;
}

auto CoordinateTransformer::SetTorchPos(Point3d &torchpos_any) -> void {
  this->torchpos_macs_ = this->Transform(torchpos_any, MACS).ToVec();
  // std::cout << "New torchpos: " << torchPos.GetX() << "," << torchPos.GetY() << "," << torchPos.GetZ() << std::endl;
}

auto CoordinateTransformer::GetTorchPos(CoordinateSystem target_system) const -> Point3d {
  const Point3d point_macs   = Point3d(torchpos_macs_(0), torchpos_macs_(1), torchpos_macs_(2), MACS);
  const Point3d point_target = this->Transform(point_macs, target_system);
  return point_target;
}

// auto CoordinateTransformer::GetTorchClockAngle() const -> double {
//   const Point3d torchpos_rocs = this->GetTorchPos(ROCS);
//   return atan2(torchpos_rocs.GetZ(), -torchpos_rocs.GetY()) - this->weld_object_rotation_;  // tan(beta) = z/y
// }

// auto CoordinateTransformer::GetTorchPlaneNormal() const -> Eigen::Vector3d {
//   const double angle_of_normal = GetTorchClockAngle() + (std::numbers::pi / 2);
//   return {0, -std::sin(angle_of_normal), std::cos(angle_of_normal)};
// }

}  // namespace deposition_simulator

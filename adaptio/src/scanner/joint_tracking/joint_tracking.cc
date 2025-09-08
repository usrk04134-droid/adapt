#include "scanner/joint_tracking/joint_tracking.h"

#include "scanner/joint_tracking/joint_slice.h"

namespace scanner::joint_tracking {
Coord JointTracking::CalculateRelativePosition(const Coord& currentPosition, const JointSlice& jointSlice) {
  auto targetPosition = CalculateTargetPosition_(currentPosition, jointSlice);
  return Coord{targetPosition.x - currentPosition.x, targetPosition.z - currentPosition.z};
}

Coord JointTracking::CalculateTargetPosition_(const Coord& currentPosition, const JointSlice& jointSlice) {
  double xTarget, zTarget;
  if (trackHorizontal_) {
    xTarget = GetHorizontalPosition_(jointSlice);
  } else {
    xTarget = currentPosition.x;
  }

  if (trackVertical_) {
    zTarget = jointSlice.InterpolateZ(xTarget) + stickout_;
  } else {
    zTarget = currentPosition.z;
  }
  return Coord{xTarget, zTarget};
}

double JointTracking::GetHorizontalPosition_(const JointSlice& jointSlice) {
  auto leftX   = jointSlice.GetBottomLeftCorner().x;
  auto rightX  = jointSlice.GetBottomRightCorner().x;
  leftX       += wallOffset_;
  rightX      -= wallOffset_;

  if (leftX >= rightX) {
    return (leftX + rightX) / 2.0;
  }

  auto placement = horizontalPlacement_;

  if (!trackFromLeft_) {
    placement = 1.0 - placement;
  }
  return leftX + placement * (rightX - leftX);
}
}  // namespace scanner::joint_tracking

#pragma once
#include "scanner/joint_tracking/joint_slice.h"

namespace scanner::joint_tracking {

class JointTracking {
 public:
  JointTracking(double wallOffset, double stickout, double horizontalPlacement, bool trackHorizontal,
                bool trackVertical, bool trackFromLeft)
      : wallOffset_(wallOffset),
        stickout_(stickout),
        horizontalPlacement_(horizontalPlacement),
        trackHorizontal_(trackHorizontal),
        trackVertical_(trackVertical),
        trackFromLeft_(trackFromLeft) {}

  void SetWallOffset(double wallOffset) { wallOffset_ = wallOffset; }
  void SetStickout(double stickout) { stickout_ = stickout; }
  void SetHorizontalPlacement(double horizontalPlacement) { horizontalPlacement_ = horizontalPlacement; }
  void SetTrackHorizontal(double trackHorizontal) { trackHorizontal_ = trackHorizontal; }
  void SetTrackVertical(double trackVertical) { trackVertical_ = trackVertical; }
  void SetTrackingSide(bool trackFromLeft) { trackFromLeft_ = trackFromLeft; }

  Coord CalculateRelativePosition(const Coord& currentPosition, const JointSlice& jointSlice);

 private:
  Coord CalculateTargetPosition_(const Coord& currentPosition, const JointSlice& jointSlice);
  double GetHorizontalPosition_(const JointSlice& jointSlice);

  double wallOffset_;
  double stickout_;
  double horizontalPlacement_;
  bool trackHorizontal_;
  bool trackVertical_;
  bool trackFromLeft_;
};

}  // namespace scanner::joint_tracking

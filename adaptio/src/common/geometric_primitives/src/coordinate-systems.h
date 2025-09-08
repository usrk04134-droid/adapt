#pragma once

namespace geometric_primitives {

enum CoordinateSystem {
  UNKNOWN = 0,
  MACS    = 1,
  ROCS    = 2,
  TCS     = 3,
  LPCS    = 4,
  CCS     = 5,
  IPCS    = 6,
  ICS     = 7,
  SCS     = 8,  // Slice Coorindate system (2D)
  OPCS    = 9   // Object positioner
};

}  // Namespace geometric_primitives

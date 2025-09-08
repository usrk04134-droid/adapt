#include <vector>

#include "point3d.h"
#include "src/point2d.h"

namespace deposition_simulator {

// Implementation should be provided outside simulator
class IHistoryProvider {
 public:
  virtual ~IHistoryProvider()                                                                      = 0;
  virtual auto RetrieveHistory(double total_rotation_angle) -> std::vector<Point2d>                = 0;
  virtual auto RecordHistory(std::vector<Point2d> abw_points, double total_rotation_angle) -> void = 0;
};

// Implementation should be provided outside simulator
// Can be based on sensor readings or prediction from scanner data etc.
class IDriftManager {
 public:
  virtual ~IDriftManager()                    = 0;
  virtual auto GetDriftSinceStart() -> double = 0;
};

class IAbwPredictor {
 public:
  virtual ~IAbwPredictor()                                                  = 0;
  virtual auto PredictEmptyPoints(std::vector<Point2d> &abw_points) -> void = 0;
};

}  // Namespace deposition_simulator

#pragma once

#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include "weld_system_client/weld_system_types.h"

namespace calibration {

class CalibrationMetrics {
 public:
  explicit CalibrationMetrics(prometheus::Registry* registry);

  void IncrementLaserTorchCalCount();
  void IncrementWeldObjectCalCount();

 private:
  prometheus::Counter& laser_torch_cal_count_;
  prometheus::Counter& weld_object_cal_count_;
};

}  // namespace calibration

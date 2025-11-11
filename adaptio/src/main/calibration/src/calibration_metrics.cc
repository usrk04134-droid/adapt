#include "calibration/src/calibration_metrics.h"

#include <prometheus/counter.h>
#include <prometheus/registry.h>

namespace calibration {

CalibrationMetrics::CalibrationMetrics(prometheus::Registry* registry)
    : laser_torch_cal_count_{prometheus::BuildCounter()
                                 .Name("adaptio_laser_torch_calibration_starts_total")
                                 .Help("Incremented when Laser to Torch calibration function is started")
                                 .Register(*registry)
                                 .Add({{"version", "1"}})},
      weld_object_cal_count_{prometheus::BuildCounter()
                                 .Name("adaptio_weld_object_calibration_starts_total")
                                 .Help("Incremented when Weld Object calibration function is started")
                                 .Register(*registry)
                                 .Add({{"version", "1"}})} {}

void CalibrationMetrics::IncrementLaserTorchCalCount() { laser_torch_cal_count_.Increment(); }

void CalibrationMetrics::IncrementWeldObjectCalCount() { weld_object_cal_count_.Increment(); }

}  // namespace calibration

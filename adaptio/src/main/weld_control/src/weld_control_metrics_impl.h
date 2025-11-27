#pragma once

#include "weld_control/src/weld_control_metrics.h"

namespace weld_control {

class WeldControlMetricsImpl : public WeldControlMetrics {
 public:
  explicit WeldControlMetricsImpl(prometheus::Registry* registry);

  struct Groove {
    prometheus::Gauge* top_width_mm;
    prometheus::Gauge* bottom_width_mm;
    prometheus::Gauge* area_mm;
    prometheus::Gauge* top_height_diff_mm;
  };

  struct ConfidentSlice {
    prometheus::Counter* ok;
    prometheus::Counter* no_data;
    prometheus::Counter* translation_failed;
  };

  void ObserveLatency(int count) override;
  void SetGroove(const std::optional<common::Groove>& groove) override;
  void Setup(ConfidentSliceBuffer& confident_slice_buffer) override;
  void SetConfidentSliceBuffer(ConfidentSliceBuffer& confident_slice_buffer) override;
  void ResetGroove() override;
  void IncConfidentSliceNoData() override;
  void IncConfidentSliceOk() override;
  void IncConfidentSliceTranslationFailed() override;
  void IncSliceConfidence(lpcs::SliceConfidence& confidence) override;

 private:
  std::map<lpcs::SliceConfidence, prometheus::Counter*> slice_confidence_;
  ConfidentSlice confident_slice_;
  prometheus::Gauge* confident_slice_buffer_fill_ratio_;
  prometheus::Histogram* abw_latency_lpcs_seconds_;
  Groove groove_;
  prometheus::Registry* registry_;
};
}  // namespace weld_control

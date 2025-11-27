#pragma once

#include <prometheus/histogram.h>
#include <prometheus/registry.h>

#include "lpcs/lpcs_slice.h"
#include "weld_control/src/confident_slice_buffer.h"

namespace weld_control {

class WeldControlMetrics {
 public:
  virtual ~WeldControlMetrics() = default;

  virtual void ObserveLatency(int count)                                             = 0;
  virtual void SetGroove(const std::optional<common::Groove>& groove)                = 0;
  virtual void SetConfidentSliceBuffer(ConfidentSliceBuffer& confident_slice_buffer) = 0;
  virtual void Setup(ConfidentSliceBuffer& confident_slice_buffer)                   = 0;
  virtual void ResetGroove()                                                         = 0;
  virtual void IncConfidentSliceNoData()                                             = 0;
  virtual void IncConfidentSliceOk()                                                 = 0;
  virtual void IncConfidentSliceTranslationFailed()                                  = 0;
  virtual void IncSliceConfidence(lpcs::SliceConfidence& confidence)                 = 0;
};
}  // namespace weld_control

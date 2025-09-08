#pragma once

#include "lpcs/lpcs_slice.h"
#include "macs/macs_point.h"
#include "macs/macs_slice.h"

namespace slice_translator {

class SliceObserver {
 public:
  virtual ~SliceObserver() = default;

  virtual void Receive(const macs::Slice& machine_data, const lpcs::Slice& scanner_data,
                       const macs::Point& axis_position, const double angle_from_torch_to_scanner) = 0;
};

}  // namespace slice_translator

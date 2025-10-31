#include "coordinates_translator.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/logging/application_log.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_groove.h"
#include "macs/macs_point.h"
#include "macs/macs_slice.h"
#include "slice_translator/slice_observer.h"
#include "slice_translator/slice_translator_service_v2.h"

namespace {

auto GrooveHasValidPoints(const std::vector<lpcs::Point>& groove) -> bool {
  if (groove.size() < 2) {
    return false;
  }

  return std::any_of(groove.begin(), groove.end(), [](const lpcs::Point& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           (std::abs(point.x) > std::numeric_limits<double>::epsilon() ||
            std::abs(point.y) > std::numeric_limits<double>::epsilon());
  });
}

}  // namespace

using slice_translator::CoordinatesTranslator;

CoordinatesTranslator::CoordinatesTranslator(SliceTranslatorServiceV2* slice_translator_v2)
    : slice_translator_v2_(slice_translator_v2) {}

void CoordinatesTranslator::AddObserver(SliceObserver* observer) { observers_.push_back(observer); }

void CoordinatesTranslator::OnScannerDataUpdate(const lpcs::Slice& data, const macs::Point& axis_position) {
  OnScannerDataUpdateV2(data, axis_position);
}

void CoordinatesTranslator::OnScannerDataUpdateV2(const lpcs::Slice& data, const macs::Point& axis_position) {
  macs::Slice machine_slice{.time_stamp = data.time_stamp};
  double angle_from_torch_to_scanner = 0.0;

  if (data.groove.has_value()) {
    const auto& groove_lpcs = data.groove.value();
    if (GrooveHasValidPoints(groove_lpcs)) {
      auto groove_mcs = slice_translator_v2_->LPCSToMCS(groove_lpcs, axis_position);
      if (groove_mcs.has_value()) {
        machine_slice.groove = macs::Groove(groove_mcs.value());
        LOG_DEBUG("Transformed points with axis positions hori: {:.2f}, vert: {:.2f}", axis_position.horizontal,
                  axis_position.vertical);
        LOG_DEBUG("Machine points: {}", machine_slice.Describe());
      } else {
        LOG_TRACE("LPCSToMCS() failed");
      }

      auto angle = slice_translator_v2_->AngleFromTorchToScanner(groove_lpcs, axis_position);
      if (angle.has_value()) {
        angle_from_torch_to_scanner = angle.value();
      }
    } else {
      LOG_TRACE("Skipping groove conversion due to insufficient or degenerate points");
    }
  }

  auto line_mcs = slice_translator_v2_->LPCSToMCS(data.line, axis_position);
  if (!line_mcs.has_value()) {
    LOG_TRACE("LPCSToMCS() for line failed");
    return;
  }

  machine_slice.line = line_mcs.value();

  for (auto* observer : observers_) {
    observer->Receive(machine_slice, data, axis_position, angle_from_torch_to_scanner);
  }
}

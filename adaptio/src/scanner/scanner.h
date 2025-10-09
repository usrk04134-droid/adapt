#pragma once

#include <boost/outcome.hpp>
#include <memory>
#include <optional>

#include "scanner/image/image.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/joint_tracking/joint_slice.h"
#include "scanner/scanner_types.h"

namespace scanner {

enum class ScannerErrorCode : uint32_t {
  NO_ERROR            = 0,
  NO_JOINT_PROPERTIES = 1,
};
// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(ScannerErrorCode) -> std::error_code;

class ScannerOutputCB {
 public:
  virtual void ScannerOutput(const joint_tracking::JointSlice& joint_slice, const std::optional<double> area,
                             uint64_t time_stamp, joint_tracking::SliceConfidence confidence) = 0;
};

class Scanner {
 public:
  virtual ~Scanner() = default;

  /**
   * Start the scanner.
   */
  virtual auto Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> = 0;

  /**
   * Stops the scanner.
   */
  virtual void Stop() = 0;

  /**
   * Try to fetch image and calculate a joint slice
   */
  virtual void Update() = 0;

  /**
   * Update Joint Approximation Data
   */
  virtual void UpdateJointApproximation(const joint_model::JointProperties& properties,
                                        const std::tuple<double, double>& abw0_abw6_horizontal) = 0;

  /**
   * Receive an image from an image provider
   */
  virtual void ImageGrabbed(std::unique_ptr<image::Image>) = 0;

  /**
   * Check how many images have been received from the
   * image provider
   */
  virtual auto CountOfReceivedImages() -> size_t = 0;
};

using ScannerPtr = std::unique_ptr<Scanner>;
}  // namespace scanner

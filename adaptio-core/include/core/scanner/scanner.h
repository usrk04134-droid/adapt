#pragma once

#include <boost/outcome.hpp>
#include <memory>
#include <optional>

#include "core/image/image.h"
#include "core/joint_tracking/joint_slice.h"
#include "core/scanner/image_provider.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/scanner_types.h"

namespace core::scanner {

using core::joint_tracking::Coord;

enum class ScannerErrorCode : uint32_t {
  NO_ERROR            = 0,
  NO_JOINT_PROPERTIES = 1,
};
// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(ScannerErrorCode) -> std::error_code;

class ScannerOutputCB {
 public:
  virtual void ScannerOutput(const core::joint_tracking::JointSlice& joint_slice, const std::array<Coord, 15>& line,
                             const std::optional<double> area, uint64_t time_stamp) = 0;
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
   * Update Joint Geometry
   */
  virtual void UpdateJointGeometry(const JointProperties& properties) = 0;

  /**
   * Receive an image from an image provider
   */
  virtual void ImageGrabbed(std::unique_ptr<core::image::Image>) = 0;

  /**
   * Check how many images have been received from the
   * image provider
   */
  virtual size_t CountOfReceivedImages() = 0;
};

using ScannerPtr = std::unique_ptr<Scanner>;
}  // namespace core::scanner

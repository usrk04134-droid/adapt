#pragma once

#include <boost/outcome.hpp>
#include <cstdint>
#include <system_error>
#include <type_traits>

#include "calibration/calibration_types.h"
#include "lpcs/lpcs_point.h"
#include "macs/macs_point.h"
#include "scanner_client/scanner_client.h"

namespace slice_translator {

enum class SliceTranslatorErrorCode : uint32_t {
  NO_ERROR = 0,
  FAILED_TO_GET_POSITION,
  INCORRECT_INPUT_SLICE,
  MISSING_CALIBRATION,
  COMPUTE_ERROR
};

class SliceTranslator {
 public:
  virtual ~SliceTranslator() = default;

  virtual auto CalibrateLaserToTorch(double angle, double offset, double stickout, const lpcs::Slice& scanner_data)
      -> std::optional<calibration::LaserTorchCalibration> = 0;
  virtual auto CalibrateWeldObject(double radius, double stickout, const lpcs::Slice& scanner_data,
                                   const macs::Point& axis_position)
      -> std::optional<calibration::WeldObjectCalibration>                                             = 0;
  virtual auto GetLaserToTorchCalibration() const -> std::optional<calibration::LaserTorchCalibration> = 0;
  virtual auto SetLaserToTorchCalibration(const calibration::LaserTorchCalibration& data)
      -> boost::outcome_v2::result<void>                                                             = 0;
  virtual auto GetWeldObjectCalibration() const -> std::optional<calibration::WeldObjectCalibration> = 0;
  virtual auto SetWeldObjectCalibration(const calibration::WeldObjectCalibration& data)
      -> boost::outcome_v2::result<void> = 0;
  virtual auto AngleFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
      -> boost::outcome_v2::result<double> = 0;
};

auto MakeErrorCode(SliceTranslatorErrorCode) -> std::error_code;
}  // namespace slice_translator

namespace std {

template <>
struct is_error_code_enum<slice_translator::SliceTranslatorErrorCode> : true_type {};

}  // namespace std

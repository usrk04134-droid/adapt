#pragma once
#include <functional>
#include <optional>

#include "kinematics_server.h"

namespace controller {

class AxisOutputPlcAdapter : public KinematicsServerObserver {
 public:
  explicit AxisOutputPlcAdapter(const std::function<void(AxisOutput)>& plc_writer);

  void OnPlcCycleWrite();

  // KinematicsServerObserver
  void OnAxisOutput(AxisOutput data) override;
  void Release() override;

 private:
  std::optional<AxisOutput> horizontal_output_to_write_;
  std::optional<AxisOutput> vertical_output_to_write_;
  std::optional<AxisOutput> weld_axis_output_to_write_;
  std::optional<AxisOutput> horizontal_last_written_;
  std::optional<AxisOutput> vertical_last_written_;
  std::optional<AxisOutput> weld_axis_last_written_;
  bool release_ = false;
  std::function<void(AxisOutput)> plc_writer_;
};

}  // namespace controller

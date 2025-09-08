#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "abp_parameters.h"
#include "weld_data_set.h"
#include "weld_process_parameters.h"
#include "weld_program.h"

namespace weld_control {

class WeldSequenceConfig {
 public:
  virtual ~WeldSequenceConfig() = default;

  virtual auto GetABPParameters() const -> std::optional<ABPParameters>               = 0;
  virtual auto GetWeldProgram() const -> std::optional<WeldProgram>                   = 0;
  virtual auto GetWeldPrograms() const -> std::vector<WeldProgram>                    = 0;
  virtual auto GetWeldDataSets() const -> std::vector<WeldDataSet>                    = 0;
  virtual auto GetWeldProcessParameters() const -> std::vector<WeldProcessParameters> = 0;

  virtual void SubscribeToUpdates(std::function<void()>) = 0;
};

}  // namespace weld_control

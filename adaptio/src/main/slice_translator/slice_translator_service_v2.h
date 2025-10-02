#pragma once

#include <nlohmann/json.hpp>

#include <optional>

#include "common/logging/application_log.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "slice_translator/coordinates_translator.h"
#include "slice_translator/model.h"
#include "slice_translator/model_config.h"
#include "slice_translator/slice_translator.h"

namespace slice_translator {

class SliceTranslatorServiceV2 : public SliceTranslator {
 public:
  explicit SliceTranslatorServiceV2(Model* model, ModelConfig* model_config, kinematics::KinematicsClient* kinematics)
      : model_(model), model_config_(model_config), kinematics_(kinematics) {}

  void AddObserver(SliceObserver* observer) override;
  auto LPCSToMCS(const std::vector<lpcs::Point>& vec, const common::groove::Point& axis_position)
      -> std::optional<std::vector<macs::Point>> override;
  auto LPCSToMCS(const lpcs::Groove& groove, const common::groove::Point& axis_position)
      -> std::optional<std::vector<macs::Point>> override;
  auto MCSToLPCS(const std::vector<macs::Point>& vec, const common::groove::Point& axis_position)
      -> std::optional<std::vector<lpcs::Point>> override;
  auto AngleFromTorchToScanner(const lpcs::Groove& groove, const common::groove::Point& axis_position)
      -> std::optional<double> override;

 private:
  Model* model_;
  ModelConfig* model_config_;
  kinematics::KinematicsClient* kinematics_;
  std::vector<SliceObserver*> observers_;
};

}  // namespace slice_translator

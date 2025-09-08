#pragma once

#include <SQLiteCpp/Database.h>

#include "common/storage/sql_multi_storage.h"
#include "common/storage/sql_single_storage.h"
#include "web_hmi/web_hmi.h"
#include "weld_sequence_config.h"

namespace weld_control {

class WeldSequenceConfigImpl : public WeldSequenceConfig {
 public:
  WeldSequenceConfigImpl(SQLite::Database* db, web_hmi::WebHmi* web_hmi);

  auto GetABPParameters() const -> std::optional<ABPParameters> override;
  auto GetWeldProgram() const -> std::optional<WeldProgram> override;
  auto GetWeldPrograms() const -> std::vector<WeldProgram> override;
  auto GetWeldDataSets() const -> std::vector<WeldDataSet> override;
  auto GetWeldProcessParameters() const -> std::vector<WeldProcessParameters> override;
  void SubscribeToUpdates(std::function<void()> on_update) override;

 private:
  void SubscribeWebHmi();

  web_hmi::WebHmi* web_hmi_;

  storage::SqlMultiStorage<WeldDataSet> weld_data_set_storage_;
  storage::SqlMultiStorage<WeldProgram> weld_program_storage_;
  storage::SqlSingleStorage<ABPParameters> abp_parameters_storage_;
  storage::SqlMultiStorage<WeldProcessParameters> weld_process_parameters_storage_;
  std::function<void()> on_update_;
};

}  // namespace weld_control

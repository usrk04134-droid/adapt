#pragma once

#include <SQLiteCpp/Database.h>

#include "common/storage/sql_single_storage.h"
#include "settings.h"
#include "web_hmi/web_hmi.h"

namespace weld_control {

class SettingsProvider {
 public:
  SettingsProvider(SQLite::Database* db, web_hmi::WebHmi* web_hmi);

  auto GetSettings() const -> std::optional<Settings>;
  void SubscribeToUpdates(std::function<void()> on_update);

 private:
  void SubscribeWebHmi();

  web_hmi::WebHmi* web_hmi_;
  storage::SqlSingleStorage<Settings> settings_storage_;
  std::function<void()> on_update_;
};

}  // namespace weld_control

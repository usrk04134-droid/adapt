#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>

#include "common/storage/sql_multi_storage.h"
#include "configuration/config_manager.h"
#include "joint_geometry/joint_geometry.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "stored_joint_geometry.h"
#include "web_hmi/web_hmi.h"

namespace joint_geometry {

class JointGeometryProviderImpl : public JointGeometryProvider {
 public:
  JointGeometryProviderImpl(configuration::ConfigManager* configuration, SQLite::Database* db);

  auto GetFixtureJointGeometry() const -> joint_geometry::JointGeometry override;
  auto GetJointGeometry() const -> std::optional<joint_geometry::JointGeometry> override;
  void Subscribe(std::function<void()> on_update) override;

  void SetWebHmi(web_hmi::WebHmi* web_hmi);

 private:
  void SubscribeWebHmi();
  void OnGetJointGeometry();
  void OnSetJointGeometry(nlohmann::json const& payload);

  static auto ConvertToDbJointGeometryFormat(const nlohmann::json& payload) -> nlohmann::json;
  static auto ConvertToWebJointGeometryFormat(const nlohmann::json& payload) -> nlohmann::json;

  web_hmi::WebHmi* web_hmi_;
  storage::SqlMultiStorage<StoredJointGeometry> joint_geometry_storage_;
  configuration::ConfigManager* config_manager_;
  std::function<void()> on_update_;
};

};  // namespace joint_geometry

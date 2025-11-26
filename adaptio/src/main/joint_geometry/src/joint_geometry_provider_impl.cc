#include "joint_geometry_provider_impl.h"

#include <SQLiteCpp/Database.h>

#include <algorithm>
#include <functional>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <vector>

#include "common/logging/application_log.h"
#include "configuration/config_manager.h"
#include "joint_geometry/joint_geometry.h"
#include "stored_joint_geometry.h"
#include "web_hmi/web_hmi.h"

namespace joint_geometry {
const auto SUCCESS_PAYLOAD = nlohmann::json{
    {"result", "ok"}
};
const auto FAILURE_PAYLOAD = nlohmann::json{
    {"result", "fail"}
};

JointGeometryProviderImpl::JointGeometryProviderImpl(configuration::ConfigManager* configuration, SQLite::Database* db)
    : joint_geometry_storage_(db, StoredJointGeometry::CreateTable, StoredJointGeometry::StoreFn(),
                              StoredJointGeometry::RemoveFn(), StoredJointGeometry::GetAllFn()),
      config_manager_(configuration) {
  if (auto joint_geometry = GetJointGeometry()) {
    LOG_INFO("Joint geometry: {}", joint_geometry.value().ToString());
  }
}

void JointGeometryProviderImpl::SetWebHmi(web_hmi::WebHmi* web_hmi) {
  web_hmi_ = web_hmi;
  this->SubscribeWebHmi();
}

auto JointGeometryProviderImpl::GetFixtureJointGeometry() const -> joint_geometry::JointGeometry {
  return config_manager_->GetCalibrationFixtureJointGeometry();
}

auto JointGeometryProviderImpl::GetJointGeometry() const -> std::optional<joint_geometry::JointGeometry> {
  auto gjg = joint_geometry_storage_.GetAll();

  int target_id = 1;
  auto it       = std::ranges::find_if(gjg, [target_id](StoredJointGeometry& item) { return item.Id() == target_id; });

  if (it != gjg.end()) {
    const StoredJointGeometry& stored = *it;

    JointGeometry jg{};
    jg.upper_joint_width_mm        = stored.UpperJointWidth();
    jg.groove_depth_mm             = stored.GrooveDepth();
    jg.left_joint_angle_rad        = stored.LeftJointAngle();
    jg.right_joint_angle_rad       = stored.RightJointAngle();
    jg.left_max_surface_angle_rad  = stored.LeftMaxSurfaceAngle();
    jg.right_max_surface_angle_rad = stored.RightMaxSurfaceAngle();
    jg.welding_type                = stored.WeldingType();

    return jg;
  }
  return std::nullopt;
}

void JointGeometryProviderImpl::Subscribe(std::function<void()> on_update) { on_update_ = on_update; }

void JointGeometryProviderImpl::SubscribeWebHmi() {
  web_hmi_->Subscribe("GetJointGeometry",
                      [this](std::string const&, const nlohmann::json&) { this->OnGetJointGeometry(); });

  web_hmi_->Subscribe("SetJointGeometry",
                      [this](std::string const&, const nlohmann::json& payload) { this->OnSetJointGeometry(payload); });
}

void JointGeometryProviderImpl::OnGetJointGeometry() {
  auto gjg               = joint_geometry_storage_.GetAll();
  nlohmann::json payload = nlohmann::json::array();
  for (auto const& sj : gjg) {
    payload.push_back(sj.ToJson());
  }
  web_hmi_->Send("GetJointGeometryRsp", SUCCESS_PAYLOAD, ConvertToWebJointGeometryFormat(payload));
}

void JointGeometryProviderImpl::OnSetJointGeometry(nlohmann::json const& payload) {
  auto sjg = StoredJointGeometry::FromJson(ConvertToDbJointGeometryFormat(payload));
  bool ok  = sjg.has_value() && joint_geometry_storage_.Store(sjg.value());
  web_hmi_->Send("SetJointGeometryRsp", ok ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD,
                 ok ? std::optional<std::string>{} : "Unable to set joint geometry", std::nullopt);
  if (ok && on_update_) {
    on_update_();
  }
}

auto JointGeometryProviderImpl::ConvertToDbJointGeometryFormat(const nlohmann::json& payload) -> nlohmann::json {
  nlohmann::json stored_jg;

  if (payload.is_object()) {
    stored_jg["id"]                          = 1;
    stored_jg["name"]                        = "default";
    stored_jg["upper_joint_width_mm"]        = payload.at("upperJointWidthMm");
    stored_jg["groove_depth_mm"]             = payload.at("grooveDepthMm");
    stored_jg["left_joint_angle_rad"]        = payload.at("leftJointAngleRad");
    stored_jg["right_joint_angle_rad"]       = payload.at("rightJointAngleRad");
    stored_jg["left_max_surface_angle_rad"]  = payload.at("leftMaxSurfaceAngleRad");
    stored_jg["right_max_surface_angle_rad"] = payload.at("rightMaxSurfaceAngleRad");
    
    if (payload.contains("weldingType")) {
      stored_jg["welding_type"] = payload.at("weldingType");
    } else {
      stored_jg["welding_type"] = "longitudinal";  // Default
    }
  }
  return stored_jg;
}

auto JointGeometryProviderImpl::ConvertToWebJointGeometryFormat(const nlohmann::json& payload) -> nlohmann::json {
  nlohmann::json web_jg;

  for (const auto& item : payload) {
    if (item.at("id") == 1) {
      web_jg["upperJointWidthMm"]       = item.at("upper_joint_width_mm");
      web_jg["grooveDepthMm"]           = item.at("groove_depth_mm");
      web_jg["leftJointAngleRad"]       = item.at("left_joint_angle_rad");
      web_jg["rightJointAngleRad"]      = item.at("right_joint_angle_rad");
      web_jg["leftMaxSurfaceAngleRad"]  = item.at("left_max_surface_angle_rad");
      web_jg["rightMaxSurfaceAngleRad"] = item.at("right_max_surface_angle_rad");
      
      if (item.contains("welding_type")) {
        web_jg["weldingType"] = item.at("welding_type");
      } else {
        web_jg["weldingType"] = "longitudinal";  // Default
      }
    }
  }
  return web_jg;
}
}  // namespace joint_geometry

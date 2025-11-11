#include "settings_provider.h"

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <utility>

#include "web_hmi/web_hmi.h"
#include "weld_control/src/settings.h"

namespace weld_control {

const auto SUCCESS_PAYLOAD = nlohmann::json{
    {"result", "ok"}
};
const auto FAILURE_PAYLOAD = nlohmann::json{
    {"result", "fail"}
};

SettingsProvider::SettingsProvider(SQLite::Database* db, web_hmi::WebHmi* web_hmi)
    : web_hmi_(web_hmi), settings_storage_(db, Settings::CreateTable, Settings::StoreFn(), Settings::GetFn()) {
  web_hmi_->Subscribe("SetSettings", [this](std::string const&, const nlohmann::json& payload) {
    auto settings = Settings::FromJson(payload);
    bool ok       = settings.has_value() && settings_storage_.Store(settings.value());
    web_hmi_->Send("SetSettingsRsp", ok ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD,
                   ok ? std::optional<std::string>{} : std::make_optional("Unable to set the settings"), std::nullopt);

    if (ok && on_update_) {
      on_update_();
    }
  });

  web_hmi_->Subscribe("GetSettings", [this](std::string const&, const nlohmann::json&) {
    auto settings = settings_storage_.Get().value_or(Settings{});
    web_hmi_->Send("GetSettingsRsp", SUCCESS_PAYLOAD, settings.ToJson());
  });
}

auto SettingsProvider::GetSettings() const -> std::optional<Settings> { return settings_storage_.Get(); }

void SettingsProvider::SubscribeToUpdates(std::function<void()> on_update) { on_update_ = std::move(on_update); }

}  // namespace weld_control

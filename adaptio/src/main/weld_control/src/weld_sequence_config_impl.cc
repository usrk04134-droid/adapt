#include "weld_sequence_config_impl.h"

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <vector>

#include "web_hmi/web_hmi.h"
#include "weld_control/src/abp_parameters.h"
#include "weld_control/src/weld_data_set.h"
#include "weld_control/src/weld_process_parameters.h"
#include "weld_control/src/weld_program.h"

namespace weld_control {

const auto SUCCESS_PAYLOAD = nlohmann::json{
    {"result", "ok"}
};
const auto FAILURE_PAYLOAD = nlohmann::json{
    {"result", "fail"}
};

WeldSequenceConfigImpl::WeldSequenceConfigImpl(SQLite::Database* db, web_hmi::WebHmi* web_hmi)
    : web_hmi_(web_hmi),
      weld_data_set_storage_(db, WeldDataSet::CreateTable, WeldDataSet::StoreFn(), WeldDataSet::RemoveFn(),
                             WeldDataSet::GetAllFn()),
      weld_program_storage_(db, WeldProgram::CreateTable, WeldProgram::StoreFn(), WeldProgram::RemoveFn(),
                            WeldProgram::GetAllFn()),
      abp_parameters_storage_(db, ABPParameters::CreateTable, ABPParameters::StoreFn(), ABPParameters::GetFn()),
      weld_process_parameters_storage_(db, WeldProcessParameters::CreateTable, WeldProcessParameters::StoreFn(),
                                       WeldProcessParameters::RemoveFn(), WeldProcessParameters::GetAllFn()) {
  SubscribeWebHmi();
}

auto WeldSequenceConfigImpl::GetABPParameters() const -> std::optional<ABPParameters> {
  return abp_parameters_storage_.Get();
}

auto WeldSequenceConfigImpl::GetWeldProgram() const -> std::optional<WeldProgram> {
  auto all = weld_program_storage_.GetAll();
  if (!all.empty()) {
    return all.front();
  }
  return std::nullopt;
}

auto WeldSequenceConfigImpl::GetWeldPrograms() const -> std::vector<WeldProgram> {
  return weld_program_storage_.GetAll();
}

auto WeldSequenceConfigImpl::GetWeldDataSets() const -> std::vector<WeldDataSet> {
  return weld_data_set_storage_.GetAll();
}

auto WeldSequenceConfigImpl::GetWeldProcessParameters() const -> std::vector<WeldProcessParameters> {
  return weld_process_parameters_storage_.GetAll();
}

void WeldSequenceConfigImpl::SubscribeToUpdates(std::function<void()> on_update) { on_update_ = on_update; }

void WeldSequenceConfigImpl::SubscribeWebHmi() {
  web_hmi_->Subscribe("AddWeldDataSet", [this](std::string const&, const nlohmann::json& payload) {
    auto ds = WeldDataSet::FromJson(payload);
    bool ok = ds.has_value() && weld_data_set_storage_.Store(ds.value());
    web_hmi_->Send("AddWeldDataSetRsp", ok ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD);
    if (ok) {
      on_update_();
    }
  });

  web_hmi_->Subscribe("RemoveWeldDataSet", [this](std::string const&, const nlohmann::json& payload) {
    int id  = 0;
    bool ok = payload.contains("id") && payload.at("id").get_to(id) && weld_data_set_storage_.Remove(id);
    web_hmi_->Send("RemoveWeldDataSetRsp", ok ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD);
    if (ok) {
      on_update_();
    }
  });

  web_hmi_->Subscribe("GetWeldDataSets", [this](std::string const&, const nlohmann::json&) {
    auto sets              = GetWeldDataSets();
    nlohmann::json payload = nlohmann::json::array();
    for (auto const& s : sets) {
      payload.push_back(s.ToJson());
    }
    web_hmi_->Send("GetWeldDataSetsRsp", payload);
  });

  web_hmi_->Subscribe("StoreWeldProgram", [this](std::string const&, const nlohmann::json& payload) {
    auto program = WeldProgram::FromJson(payload);
    bool ok      = program.has_value() && weld_program_storage_.Store(program.value());
    web_hmi_->Send("StoreWeldProgramRsp", ok ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD);
    if (ok) {
      on_update_();
    }
  });

  web_hmi_->Subscribe("GetWeldPrograms", [this](std::string const&, const nlohmann::json&) {
    auto programs          = GetWeldPrograms();
    nlohmann::json payload = nlohmann::json::array();
    for (auto const& p : programs) {
      payload.push_back(p.ToJson());
    }
    web_hmi_->Send("GetWeldProgramsRsp", payload);
  });

  web_hmi_->Subscribe("StoreABPParameters", [this](std::string const&, const nlohmann::json& payload) {
    auto abp = ABPParameters::FromJson(payload);
    bool ok  = abp.has_value() && abp_parameters_storage_.Store(abp.value());
    web_hmi_->Send("StoreABPParametersRsp", ok ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD);
    if (ok) {
      on_update_();
    }
  });

  web_hmi_->Subscribe("GetABPParameters", [this](std::string const&, const nlohmann::json&) {
    auto abp = abp_parameters_storage_.Get().value_or(ABPParameters{});
    web_hmi_->Send("GetABPParametersRsp", abp.ToJson());
  });

  web_hmi_->Subscribe("AddWeldProcessParameters", [this](std::string const&, const nlohmann::json& payload) {
    auto wpp = WeldProcessParameters::FromJson(payload);
    bool ok  = wpp.has_value() && weld_process_parameters_storage_.Store(wpp.value());
    web_hmi_->Send("AddWeldProcessParametersRsp", ok ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD);
    if (ok) {
      on_update_();
    }
  });

  web_hmi_->Subscribe("GetWeldProcessParameters", [this](std::string const&, const nlohmann::json&) {
    auto all               = GetWeldProcessParameters();
    nlohmann::json payload = nlohmann::json::array();
    for (auto const& p : all) {
      payload.push_back(p.ToJson());
    }
    web_hmi_->Send("GetWeldProcessParametersRsp", payload);
  });
}

}  // namespace weld_control

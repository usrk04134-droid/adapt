#include "cli_handler/src/log_level_cli.h"

#include <fmt/core.h>

#include <nlohmann/json_fwd.hpp>
#include <string>
#include <unordered_map>

#include "common/logging/application_log.h"
#include "cli_handler/json_payload.h"
#include "web_hmi/web_hmi.h"

using cli_handler::LogLevelCli;

LogLevelCli::LogLevelCli(web_hmi::WebHmi* web_hmi, int loglevel) : web_hmi_(web_hmi), loglevel_(loglevel) {
  auto set_loglevel = [this](const std::string& /*name*/, const nlohmann::json& payload) {
    auto result = false;
    if (payload.contains("logLevel")) {
      result = this->SetLoglevel(payload.at("logLevel").get<std::string>());
    }
    web_hmi_->Send("SetLogLevelRsp", ResultToPayload(result));
    return;
  };
  auto get_loglevel = [this](const std::string& /*name*/, const nlohmann::json& /*payload*/) {
    std::string loglevel                              = "error";
    std::unordered_map<int, std::string> loglevel_map = {
        {-1, "error"  },
        {0,  "warning"},
        {1,  "info"   },
        {2,  "debug"  },
        {3,  "trace"  },
    };
    if (loglevel_map.contains(loglevel_)) {
      loglevel = loglevel_map[loglevel_];
    }

    auto json_obj = LogLevelToPayload(loglevel);
    web_hmi_->Send("GetLogLevelRsp", json_obj);

    return;
  };

  web_hmi_->Subscribe("SetLogLevel", set_loglevel);
  web_hmi_->Subscribe("GetLogLevel", get_loglevel);
}

auto LogLevelCli::SetLoglevel(const std::string& loglevel) -> bool {
  std::unordered_map<std::string, int> loglevel_map = {
      {"error",   -1},
      {"warning", 0 },
      {"info",    1 },
      {"debug",   2 },
      {"trace",   3 },
  };
  if (loglevel_map.contains(loglevel)) {
    loglevel_ = loglevel_map[loglevel];
    LOG_TRACE("SetLogLevel {}", loglevel_);
    common::logging::SetLogLevel(loglevel_);
  } else {
    LOG_ERROR("Verbosity: Unknow Value {}", loglevel);
    return false;
  }

  return true;
}


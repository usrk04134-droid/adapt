#include "event_handler_impl.h"

#include <fmt/core.h>
#include <prometheus/counter.h>
#include <prometheus/registry.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

#include "common/logging/application_log.h"
#include "common/logging/component_logger.h"
#include "common/time/format.h"

// NOLINTNEXTLINE(misc-include-cleaner)
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "common/clock_functions.h"
#include "configuration/conf_factory.h"
#include "event_handler/event_codes.h"
#include "event_types.h"
#include "web_hmi/web_hmi.h"

namespace {
auto const EVENT_ACKNOWLEDGEMENT_ID_PATTERN = std::regex("EventHandlerAck/id=(\\d+)");
auto const EVENT_ID_OFFSET                  = 10000;
auto const MAX_RESOLVED_EVENTS              = 200;
auto const RESOLVED_EVENTS_MAX_AGE          = std::chrono::hours(48);
}  // namespace

namespace event {

EventHandlerImpl::EventHandlerImpl(clock_functions::SystemClockNowFunc system_clock_now_func,
                                   std::optional<std::filesystem::path> path_logs, prometheus::Registry* registry)
    : fh_(configuration::GetFactory()->CreateFileHandler()),
      system_clock_now_func_(system_clock_now_func),
      path_logs_(path_logs),
      registry_(registry) {
  if (path_logs_.has_value()) {
    auto const config = common::logging::ComponentLoggerConfig{
        .component      = "event",
        .path_directory = path_logs.value() / "event",
        .file_name      = "events.log",
        .max_file_size  = 100 * 1000 * 1000, /* 100 MB */
        .max_nb_files   = 1,
    };

    event_logger_ = common::logging::ComponentLogger(config);
  }

  /* use seconds since epoch as the base for the id so that event ids do not overlap when application is restarted -
   * use a fixed offset to make it easy to see when the system was restarted */
  event_id_counter_ =
      std::chrono::duration_cast<std::chrono::seconds>(system_clock_now_func_().time_since_epoch()).count() *
      EVENT_ID_OFFSET;
}

void EventHandlerImpl::SetupMetrics(prometheus::Registry* registry) {
  auto& event_family = prometheus::BuildCounter()
                           .Name("event_handler_events_raised_total")
                           .Help("Number of events raised by event code")
                           .Register(*registry);

  for (const auto& code : EVENT_CODES) {
    auto const& event = events_.find(code)->second;

    metrics_.raised_events[code] = &event_family.Add({
        {"code",        code             },
        {"title",       event.title      },
        {"group",       event.group      },
        {"description", event.description}
    });
  }
}

void EventHandlerImpl::SetWebHmi(web_hmi::WebHmi* web_hmi) {
  web_hmi_ = web_hmi;

  auto event_acknowledgement = [this](std::string const& topic, const nlohmann::json& /*payload*/) {
    std::smatch match;

    int64_t id = 0;
    if (std::regex_search(topic, match, EVENT_ACKNOWLEDGEMENT_ID_PATTERN)) {
      if (match.size() <= 1) {
        LOG_ERROR("Invalid event acknowledgement id topic={}", topic);
      }
      id = std::stoll(match[1].str());
    }

    if (id) {
      AckEvent(id);
    }
  };
  web_hmi_->SubscribePattern(EVENT_ACKNOWLEDGEMENT_ID_PATTERN, event_acknowledgement);

  auto get_events = [this](std::string const& /*topic*/, const nlohmann::json& /*payload*/) {
    CleanupOldResolvedEvents();

    nlohmann::json response = nlohmann::json::array();

    for (auto const& ei : active_events_) {
      response.push_back(EventInternalToJson(ei));
    }

    for (auto const& ei : resolved_events_) {
      response.push_back(EventInternalToJson(ei));
    }

    web_hmi_->Send("GetEventsRsp", response);
  };
  web_hmi_->Subscribe("GetEvents", get_events);

  /* send empty event list */
  SendEvents();
}

auto EventHandlerImpl::LoadEventsFromFile(std::filesystem::path const& path) -> bool {
  YAML::Node const config = YAML::LoadFile(path);

  auto ok = true;
  try {
    for (auto it = config.begin(); it != config.end(); ++it) {
      auto const& yml_event = *it;

      auto const version = yml_event["version"].as<std::string>();
      if (version != "1.0") {
        LOG_ERROR("Unsupported event version: {}", version);
        ok = false;
        break;
      }

      auto const str_resolution_type = yml_event["resolution_type"].as<std::string>();
      auto const resolution_type     = ResolutionTypeFromString(str_resolution_type);
      if (!resolution_type.has_value()) {
        LOG_ERROR("Invalid resolution_type: {}", str_resolution_type);
        ok = false;
        break;
      }

      auto const str_action = yml_event["action"].as<std::string>();
      auto const action     = ActionFromString(str_action);
      if (!action.has_value()) {
        LOG_ERROR("Invalid action: {}", str_action);
        ok = false;
        break;
      }

      auto event = Event{
          .version         = version,
          .code            = yml_event["code"].as<std::string>(),
          .group           = yml_event["group"].as<std::string>(),
          .title           = yml_event["title"].as<std::string>(),
          .description     = yml_event["description"].as<std::string>(),
          .action          = action.value(),
          .severity        = yml_event["severity"].as<std::string>(),
          .resolution_type = resolution_type.value(),
          .source          = yml_event["source"].as<std::string>(),
          .remarks         = yml_event["remarks"].as<std::string>(),
      };

      if (event.Valid()) {
        events_[event.code] = event;
      } else {
        LOG_ERROR("Invalid event: {}", event.ToString());
        ok = false;
        break;
      }
    }
  } catch (std::exception const& ex) {
    LOG_ERROR("Event yaml parsing error: {}", ex.what());
    ok = false;
  }

  LOG_INFO("Read {} event definitions from {}", events_.size(), path.string());

  if (ok) {
    SetupMetrics(registry_);
  }

  return ok;
}

void EventHandlerImpl::AddEvent(Event const& event) { events_[event.code] = event; }

auto EventHandlerImpl::EventInternalToJson(EventInternal const& ei) -> nlohmann::json {
  /* id exist in events_ already checked before calling this function */
  auto const& event = events_.find(ei.code)->second;

  nlohmann::json json({
      {"version", event.version},
      {"code", event.code},
      {"group", event.group},
      {"title", event.title},
      {"description", event.description},
      {"action", ActionToString(event.action)},
      {"severity", event.severity},
      {"resolutionType", ResolutionTypeToString(event.resolution_type)},
      {"source", event.source},
      {"remarks", event.remarks},

      {"time", common::time::TimePointToString(ei.timestamp, common::time::FMT_TS_MS)},
      {"session", "-"},
      {"resources", "-"},
      {"awsId", "-"},
      {"status", EventStatusToString(ei.status)},
  });

  if (ei.detail.has_value()) {
    json.push_back({"detail", ei.detail.value()});
  }

  if (ei.pending_ack) {
    json.push_back({"ackPath", fmt::format("EventHandlerAck/id={}", ei.id)});
  }

  return json;
}

void EventHandlerImpl::SendEvents() {
  nlohmann::json json = nlohmann::json::array();

  for (auto const& ei : active_events_) {
    if (ei.status == EventStatus::ACTIVE) {
      json.push_back(EventInternalToJson(ei));
    }
  }

  web_hmi_->Send("events", json);
}

void EventHandlerImpl::SendEventInternal(const Event& event, const Code& code, std::optional<std::string> detail) {
  auto it = std::find_if(active_events_.begin(), active_events_.end(),
                         [code](EventInternal const& ei) { return ei.code == code; });

  if (it != active_events_.end()) {
    /* event already active - ignore it */
    return;
  }

  LOG_ERROR("NEW event [{}]", event.ToString());

  auto ei = EventInternal{
      .id            = ++event_id_counter_,
      .code          = event.code,
      .detail        = std::move(detail),
      .timestamp     = system_clock_now_func_(),
      .pending_clear = ResolutionTypeNeedClear(event.resolution_type),
      .pending_ack   = ResolutionTypeNeedAck(event.resolution_type),
  };

  LogCreatedEvent(ei);

  active_events_.push_back(ei);

  auto itm = metrics_.raised_events.find(code);
  if (itm != metrics_.raised_events.end()) {
    itm->second->Increment();
  }

  SendEvents();
}

void EventHandlerImpl::RaiseEvent(const Code& code, std::optional<std::string> detail) {
  auto it = events_.find(code);
  if (it == events_.end()) {
    LOG_ERROR("Invalid event!");
    return;
  }

  const auto& event = it->second;
  switch (event.resolution_type) {
    case ResolutionType::WAIT:
    case ResolutionType::WAIT_AND_ACK:
    case ResolutionType::WAIT_OR_ACK:
      break;
    case ResolutionType::ACKABLE:
    case ResolutionType::RESTART_REQUIRED:
    default:
      LOG_ERROR("Invalid operation - cannot Raise event with resolution type: {}",
                ResolutionTypeToString(event.resolution_type));
      return;
  }

  SendEventInternal(event, code, detail);
}

void EventHandlerImpl::ClearEvent(const Code& code) {
  auto it = events_.find(code);
  if (it == events_.end()) {
    LOG_ERROR("Invalid event!");
    return;
  }

  auto it2 = std::find_if(active_events_.begin(), active_events_.end(),
                          [code](EventInternal const& ie) { return ie.code == code; });

  if (it2 == active_events_.end()) {
    /* event not active - ignore it */
    return;
  }

  auto& event = it->second;

  if (!it2->pending_clear) {
    return;
  }

  (*it2).pending_clear = false;

  LogUpdatedEvent(*it2);
  LOG_INFO("Clearing event with code={} title={} pending-ack={}", event.code, event.title,
           it2->pending_ack ? "yes" : "no");

  if (!it2->pending_ack) {
    it2->status = EventStatus::RESOLVED;
    resolved_events_.push_back(*it2);
    active_events_.erase(it2);
    CleanupOldResolvedEvents();
  }

  SendEvents();
}

void EventHandlerImpl::AckEvent(int64_t id) {
  auto it =
      std::find_if(active_events_.begin(), active_events_.end(), [id](EventInternal const& ie) { return ie.id == id; });

  if (it == active_events_.end()) {
    /* event not active - ignore it */
    return;
  }

  auto it2 = events_.find(it->code);
  if (it2 == events_.end()) {
    LOG_ERROR("Invalid event!");
    return;
  }

  auto& event = it2->second;

  if (!it->pending_ack) {
    return;
  }

  (*it).pending_ack = false;

  LogUpdatedEvent(*it);
  LOG_INFO("Ack event with code={} title={} pending-clear={}", event.code, event.title,
           it->pending_clear ? "yes" : "no");

  if (!it->pending_clear) {
    it->status = EventStatus::RESOLVED;
    resolved_events_.push_back(*it);
    active_events_.erase(it);
    CleanupOldResolvedEvents();
  }

  SendEvents();
}

void EventHandlerImpl::SendEvent(const Code& code, std::optional<std::string> detail) {
  auto it = events_.find(code);
  if (it == events_.end()) {
    LOG_ERROR("Invalid event!");
    return;
  }

  const auto& event = it->second;
  switch (event.resolution_type) {
    case ResolutionType::ACKABLE:
    case ResolutionType::RESTART_REQUIRED:
      break;
    case ResolutionType::WAIT:
    case ResolutionType::WAIT_AND_ACK:
    case ResolutionType::WAIT_OR_ACK:
    default:
      LOG_ERROR("Invalid operation - cannot Send event with resolution type: {}",
                ResolutionTypeToString(event.resolution_type));
      return;
  }

  SendEventInternal(event, code, detail);
}

auto EventHandlerImpl::ActiveBlockingEvents() const -> bool {
  for (const auto& ei : active_events_) {
    auto const it = events_.find(ei.code);
    if (it == events_.end()) {
      /* this should never happen */
      continue;
    }

    const auto& event = it->second;
    switch (event.resolution_type) {
      case ResolutionType::WAIT:
      case ResolutionType::WAIT_AND_ACK:
      case ResolutionType::RESTART_REQUIRED:
        /* event is active if it exists */
        return true;
      case ResolutionType::WAIT_OR_ACK:
        if (ei.pending_clear && ei.pending_ack) {
          return true;
        }
      case ResolutionType::ACKABLE:
        /* no operation should be restricted */
      default:
        break;
    }
  }

  return false;
}

void EventHandlerImpl::LogEvent(const nlohmann::json& json) {
  auto const str = json.dump();
  if (path_logs_.has_value()) {
    event_logger_.Log(str);
  } else {
    LOG_INFO("event-handler-log: {}", str);
  }
}

void EventHandlerImpl::LogCreatedEvent(const EventInternal& ei) {
  LogEvent(nlohmann::json{
      {"status",       "new"                  },
      {"eventId",      ei.id                  },
      {"pendingAck",   ei.pending_ack         },
      {"pendingClear", ei.pending_clear       },
      {"data",         EventInternalToJson(ei)},
  });
}

void EventHandlerImpl::LogUpdatedEvent(const EventInternal& ei) {
  LogEvent(nlohmann::json{
      {"status",       "updated"       },
      {"eventId",      ei.id           },
      {"pendingAck",   ei.pending_ack  },
      {"pendingClear", ei.pending_clear},
  });
}

void EventHandlerImpl::CleanupOldResolvedEvents() {
  auto const now         = system_clock_now_func_();
  auto const cutoff_time = now - RESOLVED_EVENTS_MAX_AGE;

  resolved_events_.erase(std::remove_if(resolved_events_.begin(), resolved_events_.end(),
                                        [cutoff_time](const EventInternal& ei) { return ei.timestamp < cutoff_time; }),
                         resolved_events_.end());

  if (resolved_events_.size() > MAX_RESOLVED_EVENTS) {
    std::sort(resolved_events_.begin(), resolved_events_.end(),
              [](const EventInternal& a, const EventInternal& b) { return a.timestamp > b.timestamp; });
    resolved_events_.resize(MAX_RESOLVED_EVENTS);
  }
}

}  // namespace event

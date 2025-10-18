
#include "weld_control_impl.h"

#include <prometheus/counter.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "abp_parameters.h"
#include "bead_control/bead_control.h"
#include "bead_control/bead_control_types.h"
#include "common/clock_functions.h"
#include "common/containers/relative_position_buffer.h"
#include "common/groove/groove.h"
#include "common/groove/point.h"
#include "common/logging/application_log.h"
#include "common/logging/component_logger.h"
#include "common/math/math.h"
#include "common/time/format.h"
#include "common/zevs/zevs_socket.h"
#include "delay_buffer.h"
#include "event_handler/event_codes.h"
#include "event_handler/event_handler.h"
#include "image_logging/image_logging_manager.h"
#include "joint_geometry/joint_geometry.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_point.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_slice.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/slice_translator_service_v2.h"
#include "tracking/tracking_manager.h"
#include "web_hmi/web_hmi.h"
#include "weld_calculations.h"
#include "weld_control/src/confident_slice_buffer.h"
#include "weld_control/src/settings_provider.h"
#include "weld_control/src/weld_sequence_config.h"
#include "weld_control/weld_control.h"
#include "weld_control/weld_control_types.h"
#include "weld_process_parameters.h"
#include "weld_system_client/weld_system_client.h"
#include "weld_system_client/weld_system_types.h"

namespace {
// With 1 seconds rate-limit there will be ~175MB data stored per day if each log entry contains 2kB data
auto const LOG_RATE_LIMIT                             = std::chrono::milliseconds(1000);
auto const LOG_1_DECIMALS                             = 1;
auto const LOG_3_DECIMALS                             = 3;
auto const HORIZONTAL_VELOCITY                        = 3.667;                             // 22 cm/min
auto const VERTICAL_VELOCITY                          = 3.667;                             // 22 cm/min
auto const VERTICAL_VELOCITY_ARCING                   = 2.0;                               // 12 cm/min
auto const REPOSITION_IN_POSITION_TOLERANCE           = 2.;                                // in mm
auto const WELD_OBJECT_MOVING_THRESHOLD               = common::math::CmMinToMmSec(0.25);  // linear velocity in mm/sec
auto const WELD_AXIS_POSITION_MAX_REVERSE_THRESHOLD   = 10;                                // in mm
auto const WELD_OBJECT_DISTANCE_MAX_REVERSE_THRESHOLD = 5;                                 // in mm
auto const WELD_AXIS_POSITION_OUT_OF_BOUNDS_THRESHOLD = (2 * std::numbers::pi) + common::math::DegToRad(3.0);
auto const FIXED_HANDOVER_GRACE                       = std::chrono::seconds(3);
auto const READY_FOR_CAP_CONFIDENT_BUFFER_FILL_RATIO  = 0.95;

auto StateToString(weld_control::State state) -> std::string {
  switch (state) {
    case weld_control::State::IDLE:
      return "idle";
    case weld_control::State::WELDING:
      return "welding";
    default:
      break;
  }
  return "invalid";
}

}  // namespace

namespace weld_control {

WeldControlImpl::WeldControlImpl(
    const Configuration& config, WeldSequenceConfig* weld_sequence_config, SettingsProvider* settings_provider,
    web_hmi::WebHmi* web_hmi, kinematics::KinematicsClient* kinematics_client, std::filesystem::path const& path_logs,
    weld_system::WeldSystemClient* weld_system_client, tracking::TrackingManager* tracking_manager,
    scanner_client::ScannerClient* scanner_client, zevs::Timer* timer, event::EventHandler* event_handler,
    bead_control::BeadControl* bead_control, DelayBuffer* delay_buffer,
    clock_functions::SystemClockNowFunc system_clock_now_func,
    clock_functions::SteadyClockNowFunc steady_clock_now_func, prometheus::Registry* registry,
    image_logging::ImageLoggingManager* image_logging_manager,
    slice_translator::SliceTranslatorServiceV2* slice_translator_v2, SQLite::Database* db)
    : config_(config),
      weld_sequence_config_(weld_sequence_config),
      settings_provider_(settings_provider),
      web_hmi_(web_hmi),
      kinematics_client_(kinematics_client),
      weld_system_client_(weld_system_client),
      tracking_manager_(tracking_manager),
      scanner_client_(scanner_client),
      timer_(timer),
      event_handler_(event_handler),
      image_logging_manager_(image_logging_manager),
      slice_translator_v2_(slice_translator_v2),
      bead_control_(bead_control),
      delay_buffer_(delay_buffer),
      confident_slice_buffer_(config.weld_data_persistent_storage ? db : nullptr),
      weld_systems_{
          {weld_system::WeldSystemId::ID1, {}},
          {weld_system::WeldSystemId::ID2, {}}
},
      system_clock_now_func_(system_clock_now_func),
      steady_clock_now_func_(steady_clock_now_func),
      smooth_weld_speed_(config.adaptivity.gaussian_filter.kernel_size, config.adaptivity.gaussian_filter.sigma),
      smooth_ws2_current_(config.adaptivity.gaussian_filter.kernel_size, config.adaptivity.gaussian_filter.sigma) {
  weld_sequence_config_->SubscribeToUpdates([this] {
    UpdateBeadControlParameters();
    CheckReady();
  });

  LOG_INFO("config: {{{}}}", ConfigurationToString(config_));

  SetupMetrics(registry);

  settings_provider_->SubscribeToUpdates([this] {
    auto const settings = settings_provider_->GetSettings();
    if (!settings.has_value()) {
      LOG_ERROR("no settings available!");
      return;
    }

    if (settings->UseEdgeSensor() == settings_.UseEdgeSensor() &&
        settings->EdgeSensorPlacementValue() == settings_.EdgeSensorPlacementValue()) {
      return;
    }

    settings_ = settings.value();
    LOG_INFO("Settings updated: {}", settings_.ToString());
    UpdateReady();
  });

  settings_ = settings_provider_->GetSettings().value_or(Settings{});

  kinematics_client_->SubscribeStateChanges(
      [this](const kinematics::StateChange& data) {
        if (data.weld_axis_state != weld_axis_state_) {
          LOG_INFO("Weld-axis state changed from {} -> {}",
                   weld_axis_state_ ? kinematics::StateToString(*weld_axis_state_) : "unknown",
                   kinematics::StateToString(data.weld_axis_state));
        }

        if (weld_axis_state_) {
          switch (data.weld_axis_state) {
            case kinematics::State::HOMED:
              /* stored position are no longer valid -> reset groove data */
              last_weld_axis_position_   = {};
              cached_weld_axis_position_ = 0.0;
              ResetGrooveDataHomed();
              break;
            case kinematics::State::INIT:
            default:
              break;
          }
        }

        weld_axis_state_ = data.weld_axis_state;
        CheckReady();
      },
      [this](const kinematics::EdgeState& state) {
        if (edge_state_ && state == *edge_state_) {
          return;
        }

        if (settings_.UseEdgeSensor() && edge_state_ && state == kinematics::EdgeState::NOT_AVAILABLE) {
          confident_slice_buffer_.Clear();

          if (state_ == State::WELDING) {
            LOG_ERROR("Edge sensor is no longer available when in welding state!");
            event_handler_->SendEvent(event::EDGE_SENSOR_LOST, std::nullopt);
            observer_->OnError();
          }
        }

        LOG_INFO("Edge state changed from {} -> {}",
                 edge_state_ ? kinematics::EdgeStateToString(*edge_state_) : "unknown",
                 kinematics::EdgeStateToString(state));

        edge_state_ = state;

        if (settings_.UseEdgeSensor()) {
          CheckReady();
        }
      });

  auto weld_control_status = [this](std::string const& /*topic*/, const nlohmann::json& /*payload*/) {
    this->GetWeldControlStatus();
  };
  web_hmi_->Subscribe("GetWeldControlStatus", weld_control_status);

  auto clear_weld_session = [this](std::string const& /*topic*/, const nlohmann::json& /*payload*/) {
    auto ok = false;
    nlohmann::json response_payload;
    std::string msg;

    if (mode_ == Mode::AUTOMATIC_BEAD_PLACEMENT) {
      msg = "Unable to clear weld session with ABP active.";
    } else {
      ok = true;
    }

    if (ok) {
      ClearWeldSession();

      response_payload = {
          {"result", "ok"}
      };
    } else {
      LOG_ERROR("Failed to clear weld session ({})", msg);
      response_payload = {
          {"result",  "fail"},
          {"message", msg   }
      };
    }

    web_hmi_->Send("ClearWeldSessionRsp", response_payload);
  };
  web_hmi_->Subscribe("ClearWeldSession", clear_weld_session);

  auto weld_system_state_change = [this](weld_system::WeldSystemId id, weld_system::WeldSystemState state) {
    this->WeldSystemStateChange(id, state);
  };

  weld_system_client_->SubscribeWeldSystemStateChanges(weld_system_state_change);

  UpdateBeadControlParameters();

  // Set storage so that at least one week of logging can be stored before logs are being rotated
  auto const wcl_config = common::logging::ComponentLoggerConfig{
      .component      = "weldcontrol",
      .path_directory = path_logs / "weldcontrol",
      .file_name      = "%Y%m%d_%H%M%S.log",
      .max_file_size  = 1300 * 1000 * 1000, /* 1.3 GB */
      .max_nb_files   = 10,
  };

  weld_logger_ = common::logging::ComponentLogger(wcl_config);
}

void WeldControlImpl::SetupMetrics(prometheus::Registry* registry) {
  {
    auto& counter = prometheus::BuildCounter()
                        .Name("weld_control_input_slice_confidences_total")
                        .Help("Confidence level for scanner input slices.")
                        .Register(*registry);

    metrics_.slice_confidence.emplace(lpcs::SliceConfidence::NO, &counter.Add({
                                                                     {"confidence", "no"}
    }));
    metrics_.slice_confidence.emplace(lpcs::SliceConfidence::LOW, &counter.Add({
                                                                      {"confidence", "low"}
    }));
    metrics_.slice_confidence.emplace(lpcs::SliceConfidence::MEDIUM, &counter.Add({
                                                                         {"confidence", "medium"}
    }));
    metrics_.slice_confidence.emplace(lpcs::SliceConfidence::HIGH, &counter.Add({
                                                                       {"confidence", "high"}
    }));
  }

  {
    auto& counter = prometheus::BuildCounter()
                        .Name("weld_control_confident_slice_results_total")
                        .Help("Confident slice handling results.")
                        .Register(*registry);

    metrics_.confident_slice.ok                 = &counter.Add({
        {"result", "ok"}
    });
    metrics_.confident_slice.no_data            = &counter.Add({
        {"result", "no_data"}
    });
    metrics_.confident_slice.translation_failed = &counter.Add({
        {"result", "translation_failed"}
    });
  }

  {
    metrics_.confident_slice_buffer_fill_ratio = &prometheus::BuildGauge()
                                                      .Name("weld_control_confident_slice_buffer_fill_ratio")
                                                      .Help("Confident slice buffer ratio of number of filled slots.")
                                                      .Register(*registry)
                                                      .Add({});
  }

  metrics_.confident_slice_buffer_fill_ratio->Set(static_cast<double>(confident_slice_buffer_.FilledSlots()) /
                                                  static_cast<double>(confident_slice_buffer_.Slots()));

  {
    const std::vector<double> buckets = {
        0.030,  // 30 ms
        0.040,  // 40 ms
        0.050,  // 50 ms
        0.060,  // 60 ms
        0.070,  // 70 ms
        0.080,  // 80 ms
        0.090,  // 90 ms
        0.100,  // 100 ms
        0.150,  // 150 ms
        0.200,  // 200 ms
        0.300,  // 300 ms
        0.400,  // 400 ms
    };

    auto& histogram = prometheus::BuildHistogram()
                          .Name("adaptio_abw_latency_lpcs_seconds")
                          .Help("ABW data latency from image taken")
                          .Register(*registry);
    metrics_.abw_latency_lpcs_seconds = &histogram.Add({}, buckets);
  }

  {
    metrics_.groove.top_width_mm = &prometheus::BuildGauge()
                                        .Name("weld_control_groove_top_width")
                                        .Help("Joint Top Width.")
                                        .Register(*registry)
                                        .Add({});
    metrics_.groove.bottom_width_mm = &prometheus::BuildGauge()
                                           .Name("weld_control_groove_bottom_width")
                                           .Help("Joint Bottom Width.")
                                           .Register(*registry)
                                           .Add({});
    metrics_.groove.area_mm =
        &prometheus::BuildGauge().Name("weld_control_groove_area").Help("Joint area.").Register(*registry).Add({});
    metrics_.groove.top_height_diff_mm = &prometheus::BuildGauge()
                                              .Name("weld_control_groove_top_height_diff")
                                              .Help("Joint Top Height Diff.")
                                              .Register(*registry)
                                              .Add({});
  }
}

void WeldControlImpl::UpdateBeadControlParameters() {
  auto const abp_parameters = weld_sequence_config_->GetABPParameters();

  if (abp_parameters.has_value()) {
    bead_control_->SetWallOffset(abp_parameters->WallOffset());
    bead_control_->SetBeadOverlap(abp_parameters->BeadOverlap());
    bead_control_->SetStepUpValue(abp_parameters->StepUpValue());
    bead_control_->SetBeadSwitchAngle(abp_parameters->BeadSwitchAngle());
    bead_control_->SetKGain(abp_parameters->KGain());
    bead_control_->SetCapBeads(abp_parameters->CapBeads());
    bead_control_->SetCapCornerOffset(abp_parameters->CapCornerOffset());

    std::vector<bead_control::BeadControl::BeadTopWidthData> data;
    for (auto bead_no = 3;; ++bead_no) {
      auto const width = abp_parameters->StepUpLimit(bead_no);
      if (!width.has_value()) {
        break;
      }
      data.push_back({
          .beads_allowed  = bead_no,
          .required_width = width.value(),
      });
    }
    bead_control_->SetTopWidthToNumBeads(data);

    switch (abp_parameters->FirstBeadPositionValue()) {
      case ABPParameters::FirstBeadPosition::LEFT:
        bead_control_->SetFirstBeadPosition(bead_control::WeldSide::LEFT);
        break;
      case ABPParameters::FirstBeadPosition::RIGHT:
        bead_control_->SetFirstBeadPosition(bead_control::WeldSide::RIGHT);
        break;
      default:
        break;
    }
  }
}

void WeldControlImpl::OnScannerStarted(bool success) {
  if (mode_ == Mode::IDLE) {
    // Scanner started from another class
    return;
  }

  if (!success) {
    LOG_ERROR("Scanner start failed");
    event_handler_->SendEvent(event::SCANNER_START_FAILED, std::nullopt);
    observer_->OnError();
  }
}

void WeldControlImpl::OnScannerStopped(bool success) {
  if (!pending_scanner_stop_) {
    // Scanner stopped from another class
    return;
  }
  pending_scanner_stop_ = false;

  if (success) {
    LOG_DEBUG("Scanner stopped successfully");
  } else {
    // TODO: handle this system state?
    LOG_ERROR("Scanner stop failed");
  }
}

void WeldControlImpl::GetWeldControlStatus() const {
  auto const bead_control_status = bead_control_->GetStatus();
  auto response                  = nlohmann::json{
                       {"mode",          ModeToString(mode_)                                   },
                       {"weldingState",  StateToString(state_)                                 },
                       {"beadOperation", bead_control::StateToString(bead_control_status.state)},
                       {"layerNumber",   bead_control_status.layer_number                      },
                       {"beadNumber",    bead_control_status.bead_number                       },
                       {"progress",      bead_control_status.progress                          },
  };

  if (bead_control_status.total_beads.has_value()) {
    response["totalBeads"] = bead_control_status.total_beads.value();
  }

  web_hmi_->Send("GetWeldControlStatusRsp", response);
};

void WeldControlImpl::LogData(std::optional<std::string> annotation = std::nullopt) {
  auto const now = system_clock_now_func_();

  auto floor = [](double value, int decimals) -> double {
    double const md = pow(10, decimals);
    return std::floor(value * md) / md;
  };

  auto const tp_scanner = std::chrono::system_clock::time_point{std::chrono::nanoseconds{cached_lpcs_.time_stamp}};

  auto logdata = nlohmann::json{
      {"timestamp", common::time::TimePointToString(now, common::time::FMT_TS_MS)},
      {"type", "data"},
      {"timestampScanner", common::time::TimePointToString(tp_scanner, common::time::FMT_TS_MS)},
      {"mode", ModeToString(mode_)},
      {"grooveArea", cached_groove_area_},
      {"weldAxis",
       {
           {"position", cached_weld_axis_position_},
           {
               "velocity",
               {
                   {"actual", cached_weld_axis_ang_velocity_},
                   {"desired", weld_axis_velocity_desired_},
               },
           },
           {"radius", cached_weld_object_radius_},
           {"distance", cached_linear_object_distance_.has_value() ? cached_linear_object_distance_.value() : 0.0},

       }},
  };

  if (annotation.has_value()) {
    logdata["annotation"] = annotation.value();
  }

  nlohmann::json mcs = nlohmann::json::array();
  if (cached_mcs_.groove.has_value()) {
    for (auto const& point : cached_mcs_.groove.value()) {
      mcs.push_back({
          {"x", floor(point.horizontal, LOG_3_DECIMALS)},
          {"z", floor(point.vertical,   LOG_3_DECIMALS)},
      });
    }
  }

  nlohmann::json mcs_delayed = nlohmann::json::array();
  if (cached_delayed_mcs_.has_value()) {
    for (auto const& point : cached_delayed_mcs_.value()) {
      mcs_delayed.push_back({
          {"x", floor(point.horizontal, LOG_3_DECIMALS)},
          {"z", floor(point.vertical,   LOG_3_DECIMALS)},
      });
    }
  }

  nlohmann::json lpcs = nlohmann::json::array();
  if (cached_lpcs_.groove.has_value()) {
    for (auto const& point : cached_lpcs_.groove.value()) {
      lpcs.push_back({
          {"x", floor(point.x, LOG_3_DECIMALS)},
          {"y", floor(point.y, LOG_3_DECIMALS)},
      });
    }
  }
  logdata["mcs"]        = mcs;
  logdata["mcsDelayed"] = mcs_delayed;
  logdata["lpcs"]       = lpcs;

  if (slides_desired_.has_value()) {
    logdata["slides"]["desired"] = {
        {"horizontal", slides_desired_->horizontal_pos},
        {"vertical",   slides_desired_->vertical_pos  }
    };
    logdata["slides"]["actual"] = {
        {"horizontal", slides_actual_.horizontal},
        {"vertical",   slides_actual_.vertical  }
    };
    ;
  }

  auto log_weld_system_data = [logdata, floor](const WeldSystem& ws) -> nlohmann::json {
    return nlohmann::json{
        {"state", weld_system::WeldSystemStateToString(ws.state)},
        {"current",
         {
             {"desired", floor(ws.settings.current, LOG_1_DECIMALS)},
             {"actual", floor(ws.data.current, LOG_1_DECIMALS)},
         }},
        {"voltage", floor(ws.data.voltage, LOG_1_DECIMALS)},
        {"wireSpeed", floor(ws.data.wire_lin_velocity, LOG_1_DECIMALS)},
        {"depositionRate", floor(ws.data.deposition_rate, LOG_1_DECIMALS)},
        {"heatInput", floor(ws.data.heat_input, LOG_3_DECIMALS)},
        {"wireDiameter", floor(ws.data.wire_diameter, LOG_1_DECIMALS)},
        {"twinWire", ws.data.twin_wire},
    };
  };

  logdata["weldSystems"] = {log_weld_system_data(weld_systems_[weld_system::WeldSystemId::ID1]),
                            log_weld_system_data(weld_systems_[weld_system::WeldSystemId::ID2])};

  auto const bead_control_status = bead_control_->GetStatus();
  logdata["beadControl"]         = {
      {"state",              bead_control::StateToString(bead_control_status.state)},
      {"layerNo",            bead_control_status.layer_number                      },
      {"beadNo",             bead_control_status.bead_number                       },
      {"progress",           bead_control_status.progress                          },
      {"beadSliceAreaRatio", bead_slice_area_ratio_                                },
      {"grooveAreaRatio",    groove_area_ratio_                                    },
      {"horizontalOffset",   bead_control_horizontal_offset_                       },
  };

  if (settings_.UseEdgeSensor()) {
    logdata["edgePosition"] = cached_edge_position_;
  }

  weld_logger_.Log(logdata.dump());

  last_log_ = now;
}

void WeldControlImpl::LogDataRateLimited() {
  auto const elapsed = system_clock_now_func_() - last_log_;
  if (elapsed > LOG_RATE_LIMIT) {
    LogData();
  }
}

void WeldControlImpl::LogModeChange() {
  auto logdata = nlohmann::json{
      {"timestamp", common::time::TimePointToString(system_clock_now_func_(), common::time::FMT_TS_MS)},
      {"type", "modeChange"},
      {"mode", ModeToString(mode_)},
  };

  switch (mode_) {
    case Mode::AUTOMATIC_BEAD_PLACEMENT: {
      auto const abp_parameters = weld_sequence_config_->GetABPParameters();

      assert(abp_parameters.has_value());

      logdata["abpParameters"] = abp_parameters->ToJson();
    }
    case Mode::JOINT_TRACKING: {
      logdata["verticalOffset"] = vertical_offset_;
      break;
    }
    case Mode::IDLE:
    default:
      break;
  };

  weld_logger_.Log(logdata.dump());
}

auto WeldControlImpl::JTReady() const -> bool {
  auto const edge_sensor_ok = settings_.UseEdgeSensor() ? edge_state_.value_or(kinematics::EdgeState::NOT_AVAILABLE) ==
                                                              kinematics::EdgeState::AVAILABLE
                                                        : true;

  return edge_sensor_ok;
}

auto WeldControlImpl::ABPReady() const -> bool {
  auto const weld_axis_ok   = weld_axis_state_ && *weld_axis_state_ == kinematics::State::HOMED;
  auto const edge_sensor_ok = settings_.UseEdgeSensor() ? edge_state_.value_or(kinematics::EdgeState::NOT_AVAILABLE) ==
                                                              kinematics::EdgeState::AVAILABLE
                                                        : true;

  auto const abp_parameters_ok = weld_sequence_config_->GetABPParameters().has_value();
  auto const weld_session_ok   = !weld_session_.resume_blocked;

  return weld_axis_ok && edge_sensor_ok && abp_parameters_ok && weld_session_ok;
}

void WeldControlImpl::UpdateReady() {
  if (on_ready_update_) {
    auto ready_modes = std::vector<std::pair<Mode, LayerType>>{};
    if (JTReady()) {
      ready_modes.emplace_back(Mode::JOINT_TRACKING, LayerType::NOT_APPLICABLE);
    }

    if (ABPReady()) {
      if (!weld_session_.active || !weld_session_.ready_for_cap) {
        ready_modes.emplace_back(Mode::AUTOMATIC_BEAD_PLACEMENT, LayerType::FILL);
      }

      if ((weld_session_.active && weld_session_.ready_for_cap) ||
          (!weld_session_.active && ready_for_jt_to_auto_cap_)) {
        ready_modes.emplace_back(Mode::AUTOMATIC_BEAD_PLACEMENT, LayerType::CAP);
      }
    };

    on_ready_update_(ready_modes);
  }
}
void WeldControlImpl::CheckReady() {
  if (mode_ == Mode::AUTOMATIC_BEAD_PLACEMENT) {
    return;
  }

  if (ABPReady()) {
    auto on_weld_axis_response = [this](std::uint64_t /*time_stamp*/, double /*position*/, double /*ang_velocity*/,
                                        double radius, double /*linear_object_distance*/) {
      if (mode_ != Mode::AUTOMATIC_BEAD_PLACEMENT) {
        cached_weld_object_radius_ = radius;
        UpdateOutput(1., 1.);
      }
    };
    kinematics_client_->GetWeldAxisData(on_weld_axis_response);
  }

  UpdateReady();
}

void WeldControlImpl::WeldSystemStateChange(weld_system::WeldSystemId id, weld_system::WeldSystemState state) {
  auto const arcing = state == weld_system::WeldSystemState::ARCING;

  LOG_INFO("Weld system {} state change {} -> {}", weld_system::WeldSystemIdToString(id),
           weld_system::WeldSystemStateToString(weld_systems_[id].state), weld_system::WeldSystemStateToString(state));

  switch (mode_) {
    case Mode::AUTOMATIC_BEAD_PLACEMENT: {
      /* compare previous state with new state to see if arcing is lost */
      auto const arcing_lost = weld_systems_[id].state == weld_system::WeldSystemState::ARCING && !arcing;

      if (arcing_lost && !weld_systems_[id].arcing_lost_timestamp) {
        LOG_ERROR("Unexpected weld-system-{} arcing lost", weld_system::WeldSystemIdToString(id));
        weld_systems_[id].arcing_lost_timestamp = steady_clock_now_func_();
      } else if (arcing && weld_systems_[id].arcing_lost_timestamp.has_value()) {
        auto const now                     = steady_clock_now_func_();
        auto const duration_without_arcing = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - weld_systems_[id].arcing_lost_timestamp.value());
        LOG_INFO("Regained weld-system-{} arcing after {} ms", weld_system::WeldSystemIdToString(id),
                 duration_without_arcing.count());
      }
      break;
    }
    case Mode::IDLE:
    case Mode::JOINT_TRACKING:
      if (arcing) {
        ClearWeldSession();
      }
      break;
  }

  if (arcing) {
    weld_systems_[id].arcing_lost_timestamp = {};
  }

  weld_systems_[id].state = state;

  // Log all weld-system state changes to make it possible to determine how much time is spent welding
  // for the different weld-control state/modes (Manual, JT, ABP).
  LogData("weld-system-state-change");
}

auto WeldControlImpl::ValidateInput() -> std::optional<event::Code> {
  if (weld_axis_state_ != kinematics::State::HOMED) {
    return {};
  }

  if (cached_weld_object_radius_ <= 0.) {
    LOG_ERROR("Invalid weld object radius: {}", cached_weld_object_radius_);
    return event::ABP_INVALID_INPUT;
  }

  if (cached_weld_axis_position_ < 0.0) {
    LOG_ERROR("Invalid weld axis position: {}", cached_weld_axis_position_);
    return event::ABP_INVALID_INPUT;
  }

  if (cached_weld_axis_position_ >= WELD_AXIS_POSITION_OUT_OF_BOUNDS_THRESHOLD) {
    LOG_ERROR("Weld axis position out of bounds: {}", cached_weld_axis_position_);
    return event::WELD_AXIS_INVALID_POSITION;
  }

  if (cached_weld_axis_position_ >= 2 * std::numbers::pi) {
    LOG_TRACE("Weld axis position > 2*pi: {:.7f} set to 0.0", cached_weld_axis_position_);
    cached_weld_axis_position_ = 0.0;
  }

  auto const distance_since_last_pos = last_weld_axis_position_.has_value()
                                           ? common::math::WrappedDist(last_weld_axis_position_.value(),
                                                                       cached_weld_axis_position_, 2 * std::numbers::pi)
                                           : 0.0;

  if (distance_since_last_pos < 0.0 && cached_weld_axis_ang_velocity_ > 0.0) {
    auto const distance_linear = common::math::AngularToLinear(distance_since_last_pos, cached_weld_object_radius_);
    if (state_ == State::WELDING && -distance_linear > WELD_AXIS_POSITION_MAX_REVERSE_THRESHOLD) {
      LOG_ERROR("Invalid weld-axis position change from {} -> {} diff(mm): {:.1f}", last_weld_axis_position_.value(),
                cached_weld_axis_position_, distance_linear);
      return event::ABP_INVALID_INPUT;
    }

    /* within tolerance - use last valid position */
    cached_weld_axis_position_ = last_weld_axis_position_.value();
  }

  last_weld_axis_position_ = cached_weld_axis_position_;

  return {};
}

auto WeldControlImpl::CheckSupervision() -> std::optional<event::Code> {
  for (auto [id, ws] : weld_systems_) {
    auto const now = steady_clock_now_func_();
    if (ws.arcing_lost_timestamp.has_value() &&
        now - ws.arcing_lost_timestamp.value() > config_.supervision.arcing_lost_grace) {
      LOG_ERROR("Failed weld-system-{} arcing supervision", weld_system::WeldSystemIdToString(id));
      return event::ARCING_LOST;
    }
  }

  return {};
}

void WeldControlImpl::ChangeMode(Mode new_mode) {
  LOG_INFO("Mode change {} -> {} (state: {})", ModeToString(mode_), ModeToString(new_mode), StateToString(state_));

  mode_ = new_mode;

  LogModeChange();
}

void WeldControlImpl::ChangeState(State new_state) {
  LOG_INFO("State change {} -> {} (mode: {})", StateToString(state_), StateToString(new_state), ModeToString(mode_));

  for (auto* observer : weld_state_observers_) {
    observer->OnWeldStateChanged(new_state == State::WELDING ? WeldStateObserver::State::WELDING
                                                             : WeldStateObserver::State::IDLE);
  }

  state_ = new_state;
}

void WeldControlImpl::UpdateConfidentSlice() {
  if (weld_axis_state_ != kinematics::State::HOMED) {
    return;
  }

  metrics_.confident_slice_buffer_fill_ratio->Set(static_cast<double>(confident_slice_buffer_.FilledSlots()) /
                                                  static_cast<double>(confident_slice_buffer_.Slots()));

  auto const use_edge_sensor = settings_.UseEdgeSensor();
  if (cached_lpcs_.confidence == lpcs::SliceConfidence::HIGH && use_edge_sensor) {
    if (!confident_slice_buffer_.Available()) {
      confident_slice_buffer_.Init(cached_weld_object_radius_, config_.storage_resolution);
    }

    confident_slice_buffer_.Store(cached_weld_axis_position_,
                                  {.edge_position = cached_edge_position_, .groove = cached_mcs_.groove.value()});
  }

  auto const data = confident_slice_buffer_.Get(cached_weld_axis_position_);
  if (!data) {
    metrics_.confident_slice.no_data->Increment();
    return;
  }

  auto const fill_ratio = confident_slice_buffer_.FillRatio();
  LOG_TRACE("confident_slice_buffer filled-ratio: {:.1f}%", fill_ratio * 100);

  auto groove                       = data.value().second.groove;
  auto const edge_sensor_adjustment = data.value().second.edge_position - cached_edge_position_;

  groove.Move(common::Point{
      .horizontal = settings_.EdgeSensorPlacementValue() == Settings::EdgeSensorPlacement::RIGHT
                        ? -edge_sensor_adjustment
                        : edge_sensor_adjustment,
  });

  auto const upper_width           = groove.TopWidth();
  auto const left_wall_angle       = groove.LeftWallAngle();
  auto const right_wall_angle      = groove.RightWallAngle();
  auto const upper_width_tolerance = config_.scanner_groove_geometry_update.tolerance.upper_width;
  auto const wall_angle_tolerance  = config_.scanner_groove_geometry_update.tolerance.wall_angle;
  auto const lpcs_groove =
      use_edge_sensor ? slice_translator_v2_->MCSToLPCS(groove.ToVector(), slides_actual_) : std::nullopt;
  LOG_TRACE(
      "Update scanner groove geometry - upper_width: {:.2f} wall_angle(left/right): {:.3f}/{:.3f} "
      "tolerance(width/wall angle): {:.3f}/{:.3f} edge_sensor_adjustment: {:.3f} translation_ok: {}",
      upper_width, left_wall_angle, right_wall_angle, upper_width_tolerance, wall_angle_tolerance,
      edge_sensor_adjustment, lpcs_groove.has_value() ? "yes" : "no");

  scanner_client_->Update({
      .upper_width      = upper_width,
      .left_wall_angle  = left_wall_angle,
      .right_wall_angle = right_wall_angle,
      .tolerance{.upper_width = upper_width_tolerance, .wall_angle = wall_angle_tolerance},
      .abw0_horizontal = lpcs_groove ? lpcs_groove.value()[common::ABW_UPPER_LEFT].x : 0.0,
      .abw6_horizontal = lpcs_groove ? lpcs_groove.value()[common::ABW_UPPER_RIGHT].x : 0.0,
  });

  if (lpcs_groove.has_value()) {
    metrics_.confident_slice.ok->Increment();
  } else {
    metrics_.confident_slice.translation_failed->Increment();
  }
}

void WeldControlImpl::UpdateReadyForABPCap() {
  /* this function handles ready-for-ABP-CAP when in JT mode - once ready_for_jt_to_auto_cap_ is set we do not allow it
   * to go back to avoid toggling the value back and forth */

  if (weld_axis_state_ != kinematics::State::HOMED) {
    return;
  }

  if (mode_ != Mode::JOINT_TRACKING) {
    return;
  }

  if (ready_for_jt_to_auto_cap_) {
    return;
  }

  auto const abp_parameters = weld_sequence_config_->GetABPParameters();
  if (!abp_parameters.has_value()) {
    return;
  }

  if (confident_slice_buffer_.FillRatio() < READY_FOR_CAP_CONFIDENT_BUFFER_FILL_RATIO) {
    return;
  }

  if (cached_mcs_.groove->AvgDepth() > abp_parameters->CapInitDepth()) {
    return;
  }

  ready_for_jt_to_auto_cap_ = true;
  UpdateReady();
}

void WeldControlImpl::ProcessInput() {
  /* check pending operations */
  if (pending_get_weld_axis_data_ || pending_get_weld_system1_data_ || pending_get_weld_system2_data_ ||
      pending_get_edge_position_) {
    return;
  }

  if (mode_ == Mode::IDLE) {
    return;
  }

  if (!UpdateSliceConfidence()) {
    return;
  }

  auto const event_invalid_input = ValidateInput();
  if (event_invalid_input.has_value()) {
    event_handler_->SendEvent(event_invalid_input.value(), std::nullopt);
    observer_->OnError();
    LogData("invalid-input");
    return;
  }

  auto const event_failed_supervision = CheckSupervision();
  if (event_failed_supervision.has_value()) {
    event_handler_->SendEvent(event_failed_supervision.value(), std::nullopt);
    observer_->OnError();
    LogData("failed-supervision-input");
    return;
  }

  auto const welding = weld_systems_[weld_system::WeldSystemId::ID1].state == weld_system::WeldSystemState::ARCING &&
                       cached_weld_axis_ang_velocity_ > 0.;

  if (welding && state_ == State::IDLE) {
    ChangeState(State::WELDING);
  } else if (!welding && state_ == State::WELDING) {
    ChangeState(State::IDLE);
  }

  UpdateTrackingPosition();
  UpdateConfidentSlice();
  UpdateReadyForABPCap();
  LogDataRateLimited();
}

void WeldControlImpl::StoreGrooveInDelayBuffer() {
  if (cached_linear_object_distance_.has_value()) {
    delay_buffer_->Store(cached_linear_object_distance_.value(), cached_mcs_.groove.value());
  }

  if (cached_weld_axis_ang_velocity_ < 0.0) {
    delay_buffer_->Clear();
    cached_linear_object_distance_ = {};
  }
}

auto WeldControlImpl::GetDelayedGrooveMCS() -> common::Groove {
  auto const current = cached_mcs_.groove.value();

  auto const delay = GetSampleToTorchDist(cached_lpcs_.time_stamp, cached_weld_axis_ang_velocity_,
                                          cached_distance_from_torch_to_scanner_);

  auto delayed_groove = delay_buffer_->Get(delay).value_or(current);

  return delayed_groove;
}

auto WeldControlImpl::GetHybridGrooveMCS() const -> common::Groove {
  auto const current_groove = cached_mcs_.groove.value();
  auto const delayed_groove = delay_buffer_->Get(cached_distance_from_torch_to_scanner_).value_or(current_groove);

  // Use ABW0,6 from current groove
  // Attempt to update the y values for the bottom of the groove.
  // Translate them in y according to the current movement of abw0 in y.
  // abw1_y = abw1_y_prev + abw0_y - abw0_y_prev

  // When a longitudinal weld is encountered, one of abw0 or abw6 moves upward in y
  // and we need to use the top corner with the lowest *absolute* abw_y - abw_y_prev

  // The edge case would be if the object tips to the side at the site of a longitudinal weld,
  // (not unlikely). For best results the bead starts should be far away from the longitudinal welds

  auto merged = current_groove;
  auto idx =
      fabs(current_groove[common::ABW_UPPER_LEFT].vertical - delayed_groove[common::ABW_UPPER_LEFT].vertical) <
              fabs(current_groove[common::ABW_UPPER_RIGHT].vertical - delayed_groove[common::ABW_UPPER_RIGHT].vertical)
          ? common::ABW_UPPER_LEFT
          : common::ABW_UPPER_RIGHT;

  const double dx = current_groove[idx].horizontal - delayed_groove[idx].horizontal;
  const double dy = current_groove[idx].vertical - delayed_groove[idx].vertical;

  for (int i = common::ABW_LOWER_LEFT; i <= common::ABW_LOWER_RIGHT; i++) {
    merged[i].horizontal = delayed_groove[i].horizontal + dx;
    merged[i].vertical   = delayed_groove[i].vertical + dy;
  }
  LOG_TRACE(
      "Delay: torch_to_scanner(mm): {:.2f}. ABW1 ({:4f},{:4f},{:4f},{:4f}), ABW5 "
      "({:4f},{:4f},{:4f},{:4f})",
      cached_distance_from_torch_to_scanner_, delayed_groove[common::ABW_LOWER_LEFT].horizontal,
      merged[common::ABW_LOWER_LEFT].horizontal, delayed_groove[common::ABW_LOWER_LEFT].vertical,
      merged[common::ABW_LOWER_LEFT].vertical, delayed_groove[common::ABW_LOWER_RIGHT].horizontal,
      merged[common::ABW_LOWER_RIGHT].horizontal, delayed_groove[common::ABW_LOWER_RIGHT].vertical,
      merged[common::ABW_LOWER_RIGHT].vertical);

  return merged;
}

auto WeldControlImpl::GetSampleToTorchDist(uint64_t ts_sample, double ang_velocity, double torch_to_scanner_distance)
    -> double {
  if (weld_axis_state_ != kinematics::State::HOMED) {
    return std::max(torch_to_scanner_distance - 5.0, 0.);
  }

  /* return distance from sample to torch in mm */
  auto const now        = system_clock_now_func_();
  auto const tp_scanner = std::chrono::system_clock::time_point{std::chrono::nanoseconds{ts_sample}};
  auto const duration_seconds =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now - tp_scanner).count()) / 1000000.;

  auto const dist_since_sample = std::max(ang_velocity * cached_weld_object_radius_ * duration_seconds, 0.);
  auto const delay             = std::max(torch_to_scanner_distance - dist_since_sample, 0.);

  return delay;
}

void WeldControlImpl::UpdateOutput(double bead_slice_area_ratio, double area_ratio) {
  auto const abp_parameters = weld_sequence_config_->GetABPParameters();
  if (!abp_parameters.has_value()) {
    return;
  }

  if (mode_ == Mode::AUTOMATIC_BEAD_PLACEMENT && state_ == State::WELDING) {
    auto const result = WeldCalc::CalculateAdaptivity(WeldCalc::CalculateAdaptivityInput{
        .weld_current_ratio = bead_slice_area_ratio,
        .weld_speed_ratio   = area_ratio,
        .heat_input_min     = abp_parameters->HeatInputMin(),
        .heat_input_max     = abp_parameters->HeatInputMax(),
        .ws1 =
            {
                  .current = weld_systems_[weld_system::WeldSystemId::ID1].data.current,
                  .voltage = weld_systems_[weld_system::WeldSystemId::ID1].data.voltage,
                  },
        .ws2 =
            {
                  .current_min = abp_parameters->WS2CurrentMin(),
                  .current_max = abp_parameters->WS2CurrentMax(),
                  .voltage     = weld_systems_[weld_system::WeldSystemId::ID2].data.voltage,
                  },
        .weld_object = {
                  .weld_speed_min = abp_parameters->WeldSpeedMin(),
                  .weld_speed_max = abp_parameters->WeldSpeedMax(),
                  }
    });

    auto const smooth_weld_speed  = smooth_weld_speed_.Update(result.weld_speed);
    auto const smooth_ws2_current = smooth_ws2_current_.Update(result.ws2_current);

    weld_systems_[weld_system::WeldSystemId::ID2].settings.current = smooth_ws2_current;
    weld_axis_velocity_desired_                                    = smooth_weld_speed / cached_weld_object_radius_;

  } else {
    /* set weld-speed and ws2 current to make the transition to ABP smooth */
    weld_systems_[weld_system::WeldSystemId::ID2].settings.current = abp_parameters->WS2CurrentAvg();
    weld_axis_velocity_desired_ = abp_parameters->WeldSpeedAvg() / cached_weld_object_radius_;
  }

  LOG_TRACE(
      "output: ws2-current: {:.1f}, weld-axis velocity(rad/sec)/velocity(cm/min)/radius(mm): {:.4f}/{:.1f}/{:.1f}",
      weld_systems_[weld_system::WeldSystemId::ID2].settings.current, weld_axis_velocity_desired_,
      common::math::MmSecToCmMin(weld_axis_velocity_desired_ * cached_weld_object_radius_), cached_weld_object_radius_);

  weld_system_client_->SetWeldSystemData(weld_system::WeldSystemId::ID2,
                                         weld_systems_[weld_system::WeldSystemId::ID2].settings);

  kinematics_client_->SetWeldAxisData(weld_axis_velocity_desired_);
}

void WeldControlImpl::UpdateTrackingPosition() {
  double slide_horizontal_lin_velocity = HORIZONTAL_VELOCITY;
  double slide_vertical_velocity       = VERTICAL_VELOCITY;

  StoreGrooveInDelayBuffer();

  auto const hybrid_mcs = GetHybridGrooveMCS();

  tracking::TrackingManager::Input tracking_input{
      .mode                   = tracking_mode_,
      .horizontal_offset      = horizontal_offset_,
      .vertical_offset        = vertical_offset_,
      .groove                 = hybrid_mcs,
      .axis_position          = slides_actual_,
      .smooth_vertical_motion = state_ == State::WELDING,
  };

  cached_delayed_mcs_ = GetDelayedGrooveMCS();

  switch (mode_) {
    case Mode::AUTOMATIC_BEAD_PLACEMENT: {
      auto const input = bead_control::BeadControl::Input{
          .weld_object_angle        = cached_weld_axis_position_,
          .weld_object_ang_velocity = cached_weld_axis_ang_velocity_,
          .weld_object_radius       = cached_weld_object_radius_,
          .weld_system1 = {.wire_lin_velocity = weld_systems_[weld_system::WeldSystemId::ID1].data.wire_lin_velocity,
                           .current           = weld_systems_[weld_system::WeldSystemId::ID1].data.current,
                           .wire_diameter     = weld_systems_[weld_system::WeldSystemId::ID1].data.wire_diameter,
                           .twin_wire         = weld_systems_[weld_system::WeldSystemId::ID1].data.twin_wire},
          .weld_system2 = {.wire_lin_velocity = weld_systems_[weld_system::WeldSystemId::ID2].data.wire_lin_velocity,
                           .current           = weld_systems_[weld_system::WeldSystemId::ID2].data.current,
                           .wire_diameter     = weld_systems_[weld_system::WeldSystemId::ID2].data.wire_diameter,
                           .twin_wire         = weld_systems_[weld_system::WeldSystemId::ID2].data.twin_wire},
          .groove       = cached_delayed_mcs_.value(),
          .in_horizontal_position =
              std::fabs(slides_actual_.horizontal - slides_desired_->horizontal_pos) < REPOSITION_IN_POSITION_TOLERANCE,
          .paused              = state_ == State::IDLE,
          .look_ahead_distance = 0,
      };

      auto const [result, output] = bead_control_->Update(input);
      switch (result) {
        case bead_control::BeadControl::Result::OK:
          break;
        case bead_control::BeadControl::Result::ERROR:
          event_handler_->SendEvent(event::ABP_CALCULATION_ERROR, std::nullopt);
          observer_->OnError();
          LogData("abp-calculation-error");
          return;
        case bead_control::BeadControl::Result::FINISHED:
          LOG_INFO("Groove finished!");
          observer_->OnGracefulStop();
          return;
      }

      UpdateOutput(output.bead_slice_area_ratio, output.groove_area_ratio);

      bead_slice_area_ratio_          = output.bead_slice_area_ratio;
      groove_area_ratio_              = output.groove_area_ratio;
      bead_control_horizontal_offset_ = output.horizontal_offset;

      tracking_input.mode              = output.tracking_mode;
      tracking_input.reference         = output.tracking_reference;
      tracking_input.horizontal_offset = output.horizontal_offset;

      if (output.horizontal_lin_velocity.has_value()) {
        slide_horizontal_lin_velocity = output.horizontal_lin_velocity.value();
      }
    }  // fallthrough
    case Mode::JOINT_TRACKING: {
      slides_desired_ = tracking_manager_->Update(tracking_input);
      if (!slides_desired_.has_value()) {
        LOG_ERROR("tracking-manager update failed!");
        return;
      }

      if (state_ == State::WELDING) {
        slide_vertical_velocity = VERTICAL_VELOCITY_ARCING;
      }
      kinematics_client_->SetSlidesPosition(slides_desired_->horizontal_pos, slides_desired_->vertical_pos,
                                            slide_horizontal_lin_velocity, slide_vertical_velocity);
      break;
    }
    case Mode::IDLE:
    default:
      break;
  }
}

auto WeldControlImpl::CheckHandover() -> bool {
  auto const now = steady_clock_now_func_();
  if (handover_to_abp_cap_timestamp_ && now > handover_to_abp_cap_timestamp_.value() + config_.handover_grace) {
    LOG_ERROR("Handover to ABP CAP failed");
    event_handler_->SendEvent(event::HANDOVER_FAILED, std::nullopt);
    observer_->OnError();
    return false;
  }

  if (handover_to_manual_timestamp_ && now > handover_to_manual_timestamp_.value() + config_.handover_grace) {
    LOG_ERROR("Handover to manual failed");
    event_handler_->SendEvent(event::HANDOVER_FAILED, std::nullopt);
    observer_->OnError();
    return false;
  }

  return true;
}

auto WeldControlImpl::UpdateSliceConfidence() -> bool {
  auto const now = steady_clock_now_func_();

  metrics_.slice_confidence[cached_lpcs_.confidence]->Increment();

  switch (cached_lpcs_.confidence) {
    case lpcs::SliceConfidence::NO:
      if (!scanner_no_confidence_timestamp_) {
        scanner_no_confidence_timestamp_ = now;
      } else if (now > scanner_no_confidence_timestamp_.value() + config_.scanner_no_confidence_grace) {
        LOG_ERROR("Scanner NO confidence grace timeout!");
        event_handler_->SendEvent(event::GROOVE_DETECTION_ERROR, std::nullopt);
        scanner_client_->FlushImageBuffer();
        observer_->OnGrooveDataTimeout();
        return false;
      }

      if (!scanner_low_confidence_timestamp_) {
        scanner_low_confidence_timestamp_ = now;
      }

      break;
    case lpcs::SliceConfidence::LOW:
      if (!scanner_low_confidence_timestamp_) {
        scanner_low_confidence_timestamp_ = now;
      } else if (!handover_to_manual_timestamp_ &&
                 now > scanner_low_confidence_timestamp_.value() + config_.scanner_low_confidence_grace) {
        LOG_INFO("Scanner LOW confidence grace timeout - Handover-to-manual triggered with {} seconds timeout",
                 config_.handover_grace);
        handover_to_manual_timestamp_ = steady_clock_now_func_();
        observer_->OnNotifyHandoverToManual();
      }

      scanner_no_confidence_timestamp_ = {};

      break;
    case lpcs::SliceConfidence::MEDIUM:
    case lpcs::SliceConfidence::HIGH:
      scanner_no_confidence_timestamp_  = {};
      scanner_low_confidence_timestamp_ = {};
      break;
  }

  if (cached_lpcs_.confidence == lpcs::SliceConfidence::NO) {
    if (!last_confident_mcs_.has_value() || !last_confident_lpcs_.has_value()) {
      /* Update Confident slice to that we are not stuck with NO confidence when close to the top surface */
      UpdateConfidentSlice();

      LOG_INFO("Skipping input due to NO confidence and no cached data.");
      return false;
    }
    cached_lpcs_ = last_confident_lpcs_.value();
    cached_mcs_  = last_confident_mcs_.value();
  } else {
    last_confident_mcs_  = cached_mcs_;
    last_confident_lpcs_ = cached_lpcs_;
  }

  return true;
}

void WeldControlImpl::Receive(const macs::Slice& machine_data, const lpcs::Slice& scanner_data,
                              const common::Point& slides_actual, const double distance_from_torch_to_scanner) {
  if (mode_ == Mode::IDLE) {
    return;
  }

  if (!machine_data.groove.has_value()) {
    LOG_ERROR("No groove mcs values");
    return;
  }

  if (!CheckHandover()) {
    return;
  }

  {
    auto const now        = system_clock_now_func_();
    auto const tp_scanner = std::chrono::system_clock::time_point{std::chrono::nanoseconds{scanner_data.time_stamp}};
    std::chrono::duration<double> const latency = now - tp_scanner;
    if (metrics_.abw_latency_lpcs_seconds) {
      metrics_.abw_latency_lpcs_seconds->Observe(latency.count());
    }
  }

  slides_actual_ = slides_actual;

  if (pending_get_weld_axis_data_ || pending_get_weld_system1_data_ || pending_get_weld_system2_data_) {
    LOG_ERROR("Got new scanner input with pending operations weld-axis/ws1/ws2: {}/{}/{}", pending_get_weld_axis_data_,
              pending_get_weld_system1_data_, pending_get_weld_system2_data_);
    return;
  }

  cached_mcs_                            = machine_data;
  cached_lpcs_                           = scanner_data;
  cached_distance_from_torch_to_scanner_ = distance_from_torch_to_scanner;
  cached_groove_area_                    = scanner_data.groove_area;

  const auto& groove         = machine_data.groove.value();
  auto const top_width       = groove.TopWidth();
  auto const bottom_width    = groove.BottomWidth();
  auto const area            = groove.Area();
  auto const top_height_diff = groove[common::ABW_UPPER_LEFT].vertical - groove[common::ABW_UPPER_RIGHT].vertical;

  if (metrics_.groove.top_width_mm) {
    metrics_.groove.top_width_mm->Set(top_width);
  }
  if (metrics_.groove.bottom_width_mm) {
    metrics_.groove.bottom_width_mm->Set(bottom_width);
  }
  if (metrics_.groove.area_mm) {
    metrics_.groove.area_mm->Set(area);
  }
  if (metrics_.groove.top_height_diff_mm) {
    metrics_.groove.top_height_diff_mm->Set(top_height_diff);
  }

  auto on_weld_axis_response = [this](std::uint64_t /*time_stamp*/, double position, double ang_velocity, double radius,
                                      double linear_object_distance) {
    if (weld_axis_state_ == kinematics::State::HOMED &&
        radius > 0.0) { /* invalid radius handled in ValidateInput function */
      auto const ang_velocity_moving_threshold = common::math::LinearToAngular(WELD_OBJECT_MOVING_THRESHOLD, radius);
      if (fabs(ang_velocity) < ang_velocity_moving_threshold) {
        /* weld-object velocity is below the threshold -> set veclocity to 0.0 */
        ang_velocity = 0.0;
      }

      if (cached_weld_axis_position_ == 0.0 && ang_velocity > 0.0) {
        LOG_INFO("Weld object rotation started");
      } else if (cached_weld_axis_position_ > 0.0 && ang_velocity == 0.0) {
        LOG_INFO("Weld object rotation stopped");
      }
    }
    cached_weld_axis_position_     = position;
    cached_weld_axis_ang_velocity_ = ang_velocity;
    cached_weld_object_radius_     = radius;

    if (!cached_linear_object_distance_.has_value()) {
      cached_linear_object_distance_ = linear_object_distance;
    }

    auto const distance_since_last_pos = linear_object_distance - cached_linear_object_distance_.value();

    if (linear_object_distance >= cached_linear_object_distance_.value()) {
      cached_linear_object_distance_ = linear_object_distance;
    } else if (-distance_since_last_pos > WELD_OBJECT_DISTANCE_MAX_REVERSE_THRESHOLD) {
      LOG_INFO("Weld axis rotated backwards from {:.5f} to {:.5f}", cached_linear_object_distance_.value(),
               linear_object_distance);
      if (weld_axis_state_ != kinematics::State::HOMED) {
        delay_buffer_->Clear();
        cached_linear_object_distance_ = linear_object_distance;
      }
    }

    pending_get_weld_axis_data_ = false;

    ProcessInput();
  };

  auto on_weld_system_response = [this](weld_system::WeldSystemId id, weld_system::WeldSystemData const& data) {
    weld_systems_[id].data = data;

    switch (id) {
      case weld_system::WeldSystemId::ID1:
        pending_get_weld_system1_data_ = false;
        break;
      case weld_system::WeldSystemId::ID2:
        pending_get_weld_system2_data_ = false;
        break;
      case weld_system::WeldSystemId::INVALID:
      default:
        return;
    }

    ProcessInput();
  };

  auto on_edge_position_response = [this](double position) {
    pending_get_edge_position_ = false;
    cached_edge_position_      = position;
    ProcessInput();
  };

  pending_get_weld_axis_data_    = true;
  pending_get_weld_system1_data_ = true;
  pending_get_weld_system2_data_ = true;
  pending_get_edge_position_     = settings_.UseEdgeSensor();

  kinematics_client_->GetWeldAxisData(machine_data.time_stamp, on_weld_axis_response);
  weld_system_client_->GetWeldSystemData(weld_system::WeldSystemId::ID1, on_weld_system_response);
  weld_system_client_->GetWeldSystemData(weld_system::WeldSystemId::ID2, on_weld_system_response);

  if (settings_.UseEdgeSensor()) {
    kinematics_client_->GetEdgePosition(on_edge_position_response);
  } else {
    cached_edge_position_ = 0.0;
  }
}

void WeldControlImpl::JointTrackingStart(const joint_geometry::JointGeometry& joint_geometry,
                                         tracking::TrackingMode tracking_mode, double horizontal_offset,
                                         double vertical_offset) {
  switch (mode_) {
    case Mode::IDLE: {
      LOG_INFO("JT Start with mode: {} horizontal-offset: {} vertical-offset: {}",
               tracking::TrackingModeToString(tracking_mode), horizontal_offset, vertical_offset);
      scanner_client_->Start({.interval = config_.scanner_input_interval}, joint_geometry);
      break;
    }
    case Mode::AUTOMATIC_BEAD_PLACEMENT:
      bead_control_->Reset();
      break;
    case Mode::JOINT_TRACKING:
    default:
      LOG_ERROR("Not allowed in current state: {}", StateToString(state_));
      return;
  }

  tracking_mode_     = tracking_mode;
  horizontal_offset_ = horizontal_offset;
  vertical_offset_   = vertical_offset;

  ChangeMode(Mode::JOINT_TRACKING);
  LogData("adaptio-state-change");
}

void WeldControlImpl::JointTrackingUpdate(tracking::TrackingMode tracking_mode, double horizontal_offset,
                                          double vertical_offset) {
  switch (mode_) {
    case Mode::JOINT_TRACKING:
      tracking_mode_     = tracking_mode;
      horizontal_offset_ = horizontal_offset;
      vertical_offset_   = vertical_offset;
      LOG_INFO("JT Update with mode: {} horizontal-offset: {:.3f} vertical-offset: {:.3f}",
               tracking::TrackingModeToString(tracking_mode), horizontal_offset, vertical_offset);
      break;
    case Mode::AUTOMATIC_BEAD_PLACEMENT:
    case Mode::IDLE:
    default:
      LOG_ERROR("Not allowed in current state: {}", StateToString(state_));
      return;
  }
}

void WeldControlImpl::AutoBeadPlacementStart(LayerType layer_type) {
  if (!ABPReady()) {
    LOG_ERROR("ABP not ready!");
    return;
  }

  auto const abp_parameters = weld_sequence_config_->GetABPParameters();
  smooth_weld_speed_.Fill(abp_parameters->WeldSpeedAvg());
  smooth_ws2_current_.Fill(abp_parameters->WS2CurrentAvg());

  ready_for_jt_to_auto_cap_ = false;
  UpdateReady();

  auto on_cap_notification = [this]() {
    auto const fill_ratio = confident_slice_buffer_.FillRatio();
    LOG_INFO("CAP notification - fill-ratio: {:.1f}%", fill_ratio * 100);

    if (fill_ratio < READY_FOR_CAP_CONFIDENT_BUFFER_FILL_RATIO) {
      LOG_INFO("Handover to manual");
      handover_to_manual_timestamp_ = steady_clock_now_func_();
      observer_->OnNotifyHandoverToManual();
      weld_session_.resume_blocked = true;
    } else {
      LOG_INFO("Handover to ABP-CAP");
      handover_to_abp_cap_timestamp_ = steady_clock_now_func_();
      weld_session_.ready_for_cap    = true;
      UpdateReady();
    }
  };

  switch (mode_) {
    case Mode::JOINT_TRACKING: {
      LOG_INFO("ABP Start with parameters: {} session: {}",
               weld_sequence_config_->GetABPParameters().value_or(ABPParameters{}).ToString(),
               weld_session_.active ? "resume" : "new");
      ChangeMode(Mode::AUTOMATIC_BEAD_PLACEMENT);
      weld_session_.active = true;
      break;
    }
    case Mode::AUTOMATIC_BEAD_PLACEMENT:
      handover_to_abp_cap_timestamp_ = {};
      break;
    case Mode::IDLE:
    default:
      LOG_ERROR("Not allowed in current state: {}", StateToString(state_));
      return;
  }

  switch (layer_type) {
    case LayerType::FILL:
      bead_control_->RegisterCapNotification(config_.handover_grace + FIXED_HANDOVER_GRACE,
                                             abp_parameters->CapInitDepth(), on_cap_notification);
      break;
    case LayerType::CAP:
      handover_to_abp_cap_timestamp_ = {};
      bead_control_->UnregisterCapNotification();
      bead_control_->NextLayerCap();
      weld_session_.ready_for_cap = true;
      break;
    case LayerType::NOT_APPLICABLE:
      /* not a valid ABP layer type */
      return;
  }

  LogData("adaptio-state-change");
}

void WeldControlImpl::AutoBeadPlacementStop() {
  switch (mode_) {
    case Mode::AUTOMATIC_BEAD_PLACEMENT:
      weld_systems_[weld_system::WeldSystemId::ID1].arcing_lost_timestamp = {};
      weld_systems_[weld_system::WeldSystemId::ID2].arcing_lost_timestamp = {};
      handover_to_abp_cap_timestamp_                                      = {};
      ChangeMode(Mode::JOINT_TRACKING);

      if (state_ == State::WELDING) {
        ClearWeldSession();
      }

      LOG_INFO("ABP stop - weld object pos: {:.4f}", cached_weld_axis_position_);

      break;
    case Mode::JOINT_TRACKING:
    case Mode::IDLE:
    default:
      LOG_ERROR("AutoBeadPlacementStop not allowed in current state: {}", StateToString(state_));
      return;
  }

  LogData("adaptio-state-change");

  UpdateReady();
}

void WeldControlImpl::Stop() {
  LOG_INFO("Stop - weld object pos: {:.4f}", cached_weld_axis_position_);
  scanner_client_->Stop();
  pending_scanner_stop_     = true;
  last_weld_axis_position_  = {};
  ready_for_jt_to_auto_cap_ = false;

  kinematics_client_->Release();
  tracking_manager_->Reset();
  delay_buffer_->Clear();
  weld_systems_[weld_system::WeldSystemId::ID1].arcing_lost_timestamp = {};
  weld_systems_[weld_system::WeldSystemId::ID2].arcing_lost_timestamp = {};
  handover_to_abp_cap_timestamp_                                      = {};
  handover_to_manual_timestamp_                                       = {};
  bead_control_->UnregisterCapNotification();

  scanner_no_confidence_timestamp_  = {};
  scanner_low_confidence_timestamp_ = {};
  handover_to_manual_timestamp_     = {};

  last_confident_lpcs_ = {};
  last_confident_mcs_  = {};

  cached_linear_object_distance_ = {};
  ChangeMode(Mode::IDLE);
  ChangeState(State::IDLE);

  LogData("adaptio-state-change");

  UpdateReady();
}

void WeldControlImpl::SetObserver(WeldControlObserver* observer) { observer_ = observer; }

auto WeldControlImpl::GetObserver() const -> WeldControlObserver* { return observer_; }

void WeldControlImpl::SubscribeReady(
    std::function<void(const std::vector<std::pair<Mode, LayerType>>&)> on_ready_update) {
  on_ready_update_ = on_ready_update;

  CheckReady();
}

void WeldControlImpl::ResetGrooveData() {
  delay_buffer_->Clear();
  ResetGrooveDataHomed();
}

void WeldControlImpl::ResetGrooveDataHomed() {
  bead_control_->ResetGrooveData();
  confident_slice_buffer_.Clear();
  ClearWeldSession();
}

void WeldControlImpl::AddWeldStateObserver(WeldStateObserver* observer) { weld_state_observers_.push_back(observer); }

void WeldControlImpl::ClearWeldSession() {
  if (weld_session_.active) {
    LOG_INFO("Clearing active weld session!");
  }

  weld_session_ = {};

  bead_control_->Reset();

  UpdateReady();
}

}  // namespace weld_control

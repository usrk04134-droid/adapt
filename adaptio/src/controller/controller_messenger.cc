#include "controller_messenger.h"

#include <fmt/core.h>

#include <boost/outcome/result.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "common/clock_functions.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "common/zevs/zevs_socket.h"
#include "controller/axis_output_plc_adapter.h"
#include "controller/controller.h"
#include "controller/controller_data.h"
#include "controller/management_client.h"
#include "controller/slide_position_buffer.h"
#include "controller/weld_axis_buffer.h"
#include "controller/weld_axis_filter.h"
#include "controller/weld_system_server_impl.h"
#include "kinematics_server_impl.h"

using controller::ControllerMessenger;

ControllerMessenger::ControllerMessenger(ControllerPtr controller, uint32_t cycle_time,
                                         clock_functions::SystemClockNowFunc system_clock_now_func,
                                         std::string endpoint_base_url)
    : controller_(std::move(controller)), cycle_time_(cycle_time), system_clock_now_func_(system_clock_now_func) {
  management_endpoint_url_  = fmt::format("inproc://{}/management", endpoint_base_url);
  kinematics_endpoint_url_  = fmt::format("inproc://{}/kinematics", endpoint_base_url);
  weld_system_endpoint_url_ = fmt::format("inproc://{}/weld-system", endpoint_base_url);

  axis_output_plc_adapter_ = std::make_unique<AxisOutputPlcAdapter>(
      [this](AxisOutput axis_output) { this->OnKinematicAxisOutputUpdate(axis_output); });

  controller_->SetCallbackInterface(this);
}

ControllerMessenger::~ControllerMessenger() {
  LOG_DEBUG("Destroying Controller Messenger");
  controller_->Disconnect();
}

void ControllerMessenger::StartThread(const std::string& event_loop_name) {
  thread_ = std::thread(&ControllerMessenger::ThreadEntry, this, event_loop_name);
}

void ControllerMessenger::JoinThread() { thread_.join(); }

void ControllerMessenger::OnAdaptioInputUpdate(AdaptioInput data) {
  previous_heartbeat_ = heartbeat_;
  heartbeat_          = data.get_heartbeat();

  auto input_data =
      ManagementClient::InputData{.start         = data.get_commands_start(),
                                  .stop          = data.get_commands_stop(),
                                  .shutdown      = data.get_commands_shutdown(),
                                  .sequence_type = static_cast<ManagementClient::Sequence>(data.get_sequence_type())};

  if (input_data_cache_ != input_data) {
    management_client_->OnAdaptioInput(input_data);
    input_data_cache_ = input_data;
  }
}

void ControllerMessenger::OnSlideCrossXInputUpdate(AxisInput data) {
  if (slide_cross_x_input_data_cache_ != data) {
    data.set_axis_id(1);
    kinematics_server_->OnAxisInput(data);
    slide_cross_x_input_data_cache_ = data;
  }
}

void ControllerMessenger::OnSlideCrossYInputUpdate(AxisInput data) {
  if (slide_cross_y_input_data_cache_ != data) {
    data.set_axis_id(2);
    kinematics_server_->OnAxisInput(data);
    slide_cross_y_input_data_cache_ = data;
  }
}

void ControllerMessenger::OnWeldAxisInputUpdate(AxisInput data) {
  data.set_velocity(weld_axis_median_filter_->ProcessSignal(data.get_velocity()));
  if (weld_axis_input_data_cache_ != data) {
    data.set_axis_id(3);
    kinematics_server_->OnAxisInput(data);
    weld_axis_input_data_cache_ = data;
  }
}

void ControllerMessenger::OnPowerSource1InputUpdate(PowerSourceInput data) {
  weld_system_server_->OnPowerSourceInput(1, data);
}

void ControllerMessenger::OnPowerSource2InputUpdate(PowerSourceInput data) {
  weld_system_server_->OnPowerSourceInput(2, data);
}

void ControllerMessenger::OnTrackingInputUpdate(TrackInput data) {
  TrackingControlData tracking_control{.horizontal_offset   = data.get_horizontal_offset(),
                                       .vertical_offset     = data.get_vertical_offset(),
                                       .joint_tracking_mode = data.get_joint_tracking_mode()};

  if (tracking_control_cache_ != tracking_control) {
    management_client_->OnTrackingInput(tracking_control);
    tracking_control_cache_ = tracking_control;
  }

  kinematics_server_->OnWeldObjectRadius(data.get_weld_object_radius());
  kinematics_server_->OnEdgePositionAvailableStatus(data.get_status_edge_tracker_value_valid());
  kinematics_server_->OnEdgePosition(data.get_edge_tracker_value());
}

void ControllerMessenger::OnHeartbeatLost() { management_client_->HeartbeatLost(); }

void ControllerMessenger::OnDisconnected(uint32_t reason_code) { management_client_->Disconnected(reason_code); }

auto ControllerMessenger::ValidateHeartbeat() -> bool { return heartbeat_ != previous_heartbeat_; }

void ControllerMessenger::OnPowerSourceOutput(uint32_t index, PowerSourceOutput const& data) {
  if (index == 1) {
    controller_->SetPowerSource1(data);
  } else if (index == 2) {
    controller_->SetPowerSource2(data);
  } else {
    LOG_ERROR("Invalid power-souce index: {} - ignored", index);
  }
}

void ControllerMessenger::ThreadEntry(const std::string& name) {
  LOG_DEBUG("Starting Controller Messenger");
  event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(name);

  management_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  management_socket_->Bind(management_endpoint_url_);
  management_client_ = std::make_unique<ManagementClient>(management_socket_.get(), this);

  horizontal_position_buffer_ = std::make_unique<SlidePositionBufferImpl>();
  vertical_position_buffer_   = std::make_unique<SlidePositionBufferImpl>();
  weld_axis_buffer_           = std::make_unique<WeldAxisBufferImpl>();
  weld_axis_median_filter_    = std::make_unique<WeldAxisFilterMedianImpl>(filter_median_window_size_);
  weld_axis_median_filter_->ClearSignalBuffer();

  kinematics_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  kinematics_socket_->Bind(kinematics_endpoint_url_);
  kinematics_server_ = std::make_unique<KinematicsServerImpl>(
      kinematics_socket_.get(), axis_output_plc_adapter_.get(), horizontal_position_buffer_.get(),
      vertical_position_buffer_.get(), weld_axis_buffer_.get(), system_clock_now_func_);

  weld_system_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  weld_system_socket_->Bind(weld_system_endpoint_url_);
  weld_system_server_ = std::make_unique<WeldSystemServerImpl>(weld_system_socket_.get(), this);

  auto result = Connect();
  if (result.has_error()) {
    LOG_ERROR("Failed connect: {}", result.error().to_string());
  }

  management_client_->Init();
  StartTimer();
  event_loop_->Run();
}

void ControllerMessenger::AdaptioOutputUpdate(const AdaptioOutput& data) {
  AdaptioOutput data_with_heartbeat{data};
  data_with_heartbeat.set_heartbeat(heartbeat_);
  OnAdaptioOutputUpdate(data_with_heartbeat);
}

void ControllerMessenger::TrackOutputUpdate(const TrackOutput& data) { OnTrackOutputUpdate(data); }

auto ControllerMessenger::Connect() -> boost::outcome_v2::result<bool> {
  const std::uint32_t max_number_of_attempts    = 10;
  const std::uint32_t sleep_before_next_attempt = 10;  // sec
  for (std::uint32_t attempt = 1; attempt < max_number_of_attempts + 1; attempt++) {
    auto result = controller_->Connect();
    if (!result.has_error()) {
      return result;
    }
    LOG_ERROR("Connection attempt {} failed: {}", attempt, result.error().to_string());
    std::this_thread::sleep_for(std::chrono::seconds(sleep_before_next_attempt));
  }

  return ControllerErrorCode::MAX_NUMBER_OF_ATTEMPTS;
}

void ControllerMessenger::StartTimer() {
  uint32_t duration_ms = cycle_time_;
  timer_               = zevs::GetFactory()->CreateTimer(*event_loop_);
  timer_->RequestPeriodic(&ControllerMessenger::OnTimeout, this, duration_ms, "controller_periodic_update");
}

void ControllerMessenger::OnTimeout() {
  if (controller_->IsConnected()) {
    axis_output_plc_adapter_->OnPlcCycleWrite();

    static uint32_t counter = 0;
    auto maybe_delta_time   = controller_->RetrieveInputs();
    management_client_->Update();

    if (auto result = controller_->WriteOutputs(); result.has_error()) {
      LOG_ERROR("Failed to write outputs: {}", result.error().to_string());
    }

    if (maybe_delta_time.has_value() && counter > 1000) {
      counter = 0;
      LOG_TRACE("Delta time: {}", maybe_delta_time.value());
    }

    counter++;
  }
}

void ControllerMessenger::OnAdaptioOutputUpdate(const AdaptioOutput& data) { controller_->SetAdaptio(data); }

void ControllerMessenger::OnKinematicAxisOutputUpdate(const AxisOutput& data) {
  switch (static_cast<AxisId>(data.get_axis_id())) {
    case AxisId::SLIDE_CROSS_HORIZONTAL:
      controller_->SetSlideCrossX(data);
      break;
    case AxisId::SLIDE_CROSS_VERTICAL:
      controller_->SetSlideCrossY(data);
      break;
    case AxisId::WELD_AXIS:
      controller_->SetWeldAxis(data);
      break;
    case AxisId::INVALID:
    default:
      LOG_ERROR("Axis ID {} is invalid, not setting outputs.", data.get_axis_id());
  }
}

void ControllerMessenger::OnTrackOutputUpdate(const TrackOutput& data) { controller_->SetTracking(data); }

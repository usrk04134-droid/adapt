
#include "management_client.h"

#include <cstdint>
#include <string>
#include <unordered_map>

#include "common/logging/application_log.h"
#include "common/messages/management.h"
#include "common/zevs/zevs_socket.h"
#include "controller/controller_data.h"

using controller::ManagementClient;

namespace {
const uint32_t SEQUENCE_NONE             = 0;
const uint32_t SEQUENCE_TRACKING         = 1;
const uint32_t SEQUENCE_AUTO_WELDING     = 2;
const uint32_t SEQUENCE_AUTO_CAL_MOVE    = 3;
const uint32_t SEQUENCE_AUTO_CAP_WELDING = 4;
}  // namespace

auto controller::TrackingControlData::operator==(const TrackingControlData& other) const -> bool {
  return horizontal_offset == other.horizontal_offset && vertical_offset == other.vertical_offset &&
         joint_tracking_mode == other.joint_tracking_mode;
}

auto controller::TrackingControlData::operator!=(const TrackingControlData& other) const -> bool {
  return !(*this == other);
}

ManagementClient::ManagementClient(zevs::Socket* socket, ManagementClientObserver* observer)
    : socket_(socket), observer_(observer) {
  socket_->Serve(&ManagementClient::OnReadyState, this);
  socket_->Serve(&ManagementClient::OnOnNotifyHandoverToManual, this);
  socket_->Serve(&ManagementClient::OnTrackingStoppedGrooveDataTimeout, this);
  socket_->Serve(&ManagementClient::OnScannerError, this);
  socket_->Serve(&ManagementClient::OnGracefulStop, this);
}

void ManagementClient::SendTrackingStart() {
  common::msg::management::TrackingStart data{};
  data.joint_tracking_mode = track_input_data_.joint_tracking_mode;
  data.horizontal_offset   = track_input_data_.horizontal_offset;
  data.vertical_offset     = track_input_data_.vertical_offset;
  socket_->Send(data);
}

void ManagementClient::SendTrackingUpdate() {
  common::msg::management::TrackingUpdate data{};
  data.joint_tracking_mode = track_input_data_.joint_tracking_mode;
  data.horizontal_offset   = track_input_data_.horizontal_offset;
  data.vertical_offset     = track_input_data_.vertical_offset;
  socket_->Send(data);
}

void ManagementClient::SendABPStart() {
  common::msg::management::ABPStart data{};
  socket_->Send(data);
}

void ManagementClient::SendABPStop() {
  common::msg::management::ABPStop data{};
  socket_->Send(data);
}

void ManagementClient::SendABPCapStart() {
  common::msg::management::ABPCapStart data{};
  socket_->Send(data);
}

void ManagementClient::SendABPCapStop() {
  common::msg::management::ABPCapStop data{};
  socket_->Send(data);
}

void ManagementClient::SendStop() {
  common::msg::management::Stop data{};
  socket_->Send(data);
}

void ManagementClient::OnReadyState(common::msg::management::ReadyState data) {
  switch (data.state) {
    case common::msg::management::ReadyState::State::NOT_READY:
      ready_state_ = ReadyState::NOT_READY;
      break;
    case common::msg::management::ReadyState::State::NOT_READY_AUTO_CAL_MOVE:
      ready_state_ = ReadyState::NOT_READY_AUTO_CAL_MOVE;
      break;
    case common::msg::management::ReadyState::State::TRACKING_READY:
      ready_state_ = ReadyState::TRACKING_READY;
      break;
    case common::msg::management::ReadyState::State::ABP_READY:
      ready_state_ = ReadyState::ABP_READY;
      break;
    case common::msg::management::ReadyState::State::ABP_CAP_READY:
      ready_state_ = ReadyState::ABP_CAP_READY;
      break;
    default:
      LOG_ERROR("Invalid ReadyState");
  }

  LOG_DEBUG("OnReadyState - ReadyState={}", ReadyStateToString(ready_state_));
}

void ManagementClient::OnOnNotifyHandoverToManual(common::msg::management::NotifyHandoverToManual /*data*/) {
  LOG_TRACE("Notification - handover to manual");
  handover_to_manual_ = true;
}

void ManagementClient::OnTrackingStoppedGrooveDataTimeout(
    common::msg::management::TrackingStoppedGrooveDataTimeout /*data*/) {
  LOG_TRACE("Joint tracking stopped - groove data timeout");
  state_ = InterfaceState::ERROR;
}

void ManagementClient::OnScannerError(common::msg::management::ScannerError /*data*/) {
  LOG_TRACE("Joint tracking stopped - scanner error");
  state_ = InterfaceState::ERROR;
}

void ManagementClient::OnGracefulStop(common::msg::management::GracefulStop /*data*/) {
  LOG_TRACE("Groove Finished");
  /* there is no graceful stop in the adaptio->PLC interface so we trigger an error to make PLC stop the weld
   * procedure*/
  state_ = InterfaceState::ERROR;
}

void ManagementClient::Init() { socket_->Send(common::msg::management::SubscribeReadyState{}); }

void ManagementClient::Update() {
  // Note that the heartbeat field in AdaptioOutput is set in ControllerMessenger
  AdaptioOutput output{};
  TrackOutput track_output{};

  switch (state_) {
    case InterfaceState::IDLE:
      output.set_status_ready(true);
      output.set_status_active(false);
      output.set_status_error(false);
      output.set_active_sequence_type(SEQUENCE_NONE);

      track_output.set_status_active(false);
      track_output.set_status_error(false);
      break;
    case InterfaceState::TRACKING:
      output.set_status_ready(false);
      output.set_status_active(true);
      output.set_status_error(false);
      output.set_active_sequence_type(SEQUENCE_TRACKING);

      track_output.set_status_active(true);
      track_output.set_status_error(false);
      break;
    case InterfaceState::ABP:
      output.set_status_ready(false);
      output.set_status_active(true);
      output.set_status_error(false);
      output.set_active_sequence_type(SEQUENCE_AUTO_WELDING);

      track_output.set_status_active(true);
      track_output.set_status_error(false);
      break;
    case InterfaceState::ABP_CAP:
      output.set_status_ready(false);
      output.set_status_active(true);
      output.set_status_error(false);
      output.set_active_sequence_type(SEQUENCE_AUTO_CAP_WELDING);

      track_output.set_status_active(true);
      track_output.set_status_error(false);
      break;
    case InterfaceState::ERROR:
      output.set_status_ready(false);
      output.set_status_active(true);
      output.set_status_error(true);
      output.set_active_sequence_type(SEQUENCE_NONE);

      track_output.set_status_active(true);
      track_output.set_status_error(true);
      break;
  }

  // ReadyState::NOT_READY_AUTO_CAL_MOVE is a late addition to handle a case where
  // a calibration sequence is started from the WebHMI and the PLC must be informed
  // to accept slide position requests from Adaptio.
  // In future versions the flow of control will be reversed and the logic redesigned.
  switch (ready_state_) {
    case ReadyState::NOT_READY:
      output.set_status_ready_for_tracking(false);
      output.set_status_ready_for_abp(false);
      output.set_status_ready_for_auto_cap(false);
      break;
    case ReadyState::NOT_READY_AUTO_CAL_MOVE:
      output.set_status_ready(false);  // Override ready and active
      output.set_status_active(true);
      output.set_status_ready_for_tracking(false);
      output.set_status_ready_for_abp(false);
      output.set_active_sequence_type(SEQUENCE_AUTO_CAL_MOVE);  // Override active_sequence_type
      break;
    case ReadyState::TRACKING_READY:
      output.set_status_ready_for_tracking(true);
      output.set_status_ready_for_abp(false);
      output.set_status_ready_for_auto_cap(false);
      break;
    case ReadyState::ABP_READY:
      output.set_status_ready_for_tracking(true);
      output.set_status_ready_for_abp(true);
      output.set_status_ready_for_auto_cap(false);
      break;
    case ReadyState::ABP_CAP_READY:
      output.set_status_ready_for_tracking(true);
      output.set_status_ready_for_abp(true);
      output.set_status_ready_for_auto_cap(true);
      break;
  }

  track_output.set_status_shallow(handover_to_manual_);

  observer_->AdaptioOutputUpdate(output);
  observer_->TrackOutputUpdate(track_output);
}

void ManagementClient::LogStateChange(const std::string& event, const InputData& data, InterfaceState new_state) const {
  if (new_state == InterfaceState::ERROR) {
    LOG_ERROR("{} - State: {}->{}, InputData: {}, ReadyState: {}", event, InterfaceStateToString(state_),
              InterfaceStateToString(new_state), data.ToString(), ReadyStateToString(ready_state_));
  } else {
    LOG_INFO("{} - State: {}->{}, InputData: {}, ReadyState: {}", event, InterfaceStateToString(state_),
             InterfaceStateToString(new_state), data.ToString(), ReadyStateToString(ready_state_));
  }
}

void ManagementClient::SetState(InterfaceState new_state, const std::string& event, const InputData& data) {
  LogStateChange(event, data, new_state);
  state_ = new_state;
  if (new_state == InterfaceState::IDLE) {
    handover_to_manual_ = false;
  }
}

// Note regarding OnAdaptioInput:
// Due to the calling order in PnDriver we know that OnAdaptioInput is called
// after OnTrackingInput in each update cycle.
void ManagementClient::OnAdaptioInput(const InputData& data) {
  if (data.shutdown) {
    SetState(InterfaceState::IDLE, "Shutdown", data);
    socket_->Send(common::msg::management::Shutdown{});
    return;
  }

  if (data.stop) {
    switch (state_) {
      case InterfaceState::TRACKING:
      case InterfaceState::ABP:
      case InterfaceState::ABP_CAP:
        SendStop();
        break;
      case InterfaceState::IDLE:
      case InterfaceState::ERROR:
        break;
    }
    SetState(InterfaceState::IDLE, "Stop", data);
    return;
  }

  if (!data.start) {
    LogStateChange("No state change", data, state_);
    return;
  }

  if (data.sequence_type == Sequence::ABP &&
      (ready_state_ != ReadyState::ABP_READY && ready_state_ != ReadyState::ABP_CAP_READY)) {
    SetState(InterfaceState::ERROR, "Cannot start ABP", data);
    return;
  }

  if (data.sequence_type == Sequence::ABP_CAP && ready_state_ != ReadyState::ABP_CAP_READY) {
    SetState(InterfaceState::ERROR, "Cannot start ABP CAP", data);
    return;
  }

  if (data.sequence_type == Sequence::TRACKING && ready_state_ == ReadyState::NOT_READY) {
    SetState(InterfaceState::ERROR, "Cannot start tracking", data);
    return;
  }

  switch (state_) {
    case InterfaceState::IDLE:
      if (data.sequence_type == Sequence::TRACKING) {
        SetState(InterfaceState::TRACKING, "Start tracking", data);
        SendTrackingStart();
      } else if (data.sequence_type == Sequence::ABP) {
        SetState(InterfaceState::ABP, "Start ABP", data);
        SendTrackingStart();
        SendABPStart();
      } else if (data.sequence_type == Sequence::ABP_CAP) {
        SetState(InterfaceState::ERROR, "IDLE -> ABP_CAP not allowed", data);
        return;
      }
      break;

    case InterfaceState::TRACKING:
      if (data.sequence_type == Sequence::ABP) {
        SetState(InterfaceState::ABP, "Start ABP", data);
        SendABPStart();
      } else if (data.sequence_type == Sequence::ABP_CAP) {
        SetState(InterfaceState::ABP_CAP, "Start ABP_CAP", data);
        SendABPCapStart();
      }
      break;

    case InterfaceState::ABP:
      if (data.sequence_type == Sequence::TRACKING) {
        SetState(InterfaceState::TRACKING, "Back to tracking", data);
        SendABPStop();
        SendTrackingUpdate();
      } else if (data.sequence_type == Sequence::ABP_CAP) {
        SetState(InterfaceState::ABP_CAP, "Start ABP_CAP", data);
        SendABPCapStart();
        SendTrackingUpdate();
      }
      break;

    case InterfaceState::ABP_CAP:
      if (data.sequence_type == Sequence::TRACKING) {
        SetState(InterfaceState::TRACKING, "Back to tracking", data);
        SendABPCapStop();
        SendTrackingUpdate();
      } else if (data.sequence_type == Sequence::ABP) {
        SetState(InterfaceState::ERROR, "ABP_CAP -> ABP not allowed", data);
        return;
      }
      break;

    case InterfaceState::ERROR:
      SetState(InterfaceState::ERROR, "In ERROR", data);
      break;
  }
}

void ManagementClient::OnTrackingInput(const TrackingControlData& data) {
  if (data.joint_tracking_mode != track_input_data_.joint_tracking_mode) {
    LOG_DEBUG("Controller requested tracking mode: {}", data.joint_tracking_mode);
  }

  track_input_data_ = data;

  if (state_ == InterfaceState::TRACKING) {
    SendTrackingUpdate();
  }
}

void ManagementClient::HeartbeatLost() {
  LOG_ERROR("ManagementClient::HeartbeatLost");
  if (state_ == InterfaceState::TRACKING || state_ == InterfaceState::ABP) {
    SendStop();
  }
  state_ = InterfaceState::ERROR;
}

void ManagementClient::Disconnected(uint32_t reason_code) {
  LOG_ERROR("ManagementClient::Disconnected, reason:{}", reason_code);
  if (state_ == InterfaceState::TRACKING || state_ == InterfaceState::ABP) {
    SendStop();
  }
  state_ = InterfaceState::ERROR;
}

auto ManagementClient::InputData::operator==(const InputData& other) const -> bool {
  return start == other.start && stop == other.stop && shutdown == other.shutdown &&
         sequence_type == other.sequence_type;
}

auto ManagementClient::InputData::ToString() const -> std::string {
  return "{ start: " + std::string(start ? "true" : "false") + ", stop: " + std::string(stop ? "true" : "false") +
         ", shutdown: " + std::string(shutdown ? "true" : "false") +
         ", sequence_type: " + ManagementClient::SequenceToString(sequence_type) + " }";
}

auto ManagementClient::SequenceToString(Sequence sequence) -> std::string {
  static const std::unordered_map<Sequence, std::string> SEQUENCE_TO_STRING_MAP = {
      {Sequence::NO_SEQUENCE, "no_sequence"},
      {Sequence::TRACKING,    "tracking"   },
      {Sequence::ABP,         "abp"        },
      {Sequence::ABP_CAP,     "abp-cap"    }
  };

  auto it = SEQUENCE_TO_STRING_MAP.find(sequence);
  if (it != SEQUENCE_TO_STRING_MAP.end()) {
    return it->second;
  }
  return "unknown";
}

auto ManagementClient::InterfaceStateToString(InterfaceState state) -> std::string {
  static const std::unordered_map<InterfaceState, std::string> INTERFACE_STATE_MAP = {
      {InterfaceState::IDLE,     "idle"    },
      {InterfaceState::TRACKING, "tracking"},
      {InterfaceState::ABP,      "abp"     },
      {InterfaceState::ABP_CAP,  "abp-cap" },
      {InterfaceState::ERROR,    "error"   }
  };

  auto it = INTERFACE_STATE_MAP.find(state);
  if (it != INTERFACE_STATE_MAP.end()) {
    return it->second;
  }
  return "unknown";
}

auto ManagementClient::ReadyStateToString(ReadyState state) -> std::string {
  static const std::unordered_map<ReadyState, std::string> READY_STATE_MAP = {
      {ReadyState::NOT_READY,               "not_ready"              },
      {ReadyState::NOT_READY_AUTO_CAL_MOVE, "not_ready_auto_cal_move"},
      {ReadyState::TRACKING_READY,          "tracking_ready"         },
      {ReadyState::ABP_READY,               "abp_ready"              },
      {ReadyState::ABP_CAP_READY,           "abp_cap_ready"          }
  };

  auto it = READY_STATE_MAP.find(state);
  if (it != READY_STATE_MAP.end()) {
    return it->second;
  }
  return "unknown";
}

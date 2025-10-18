#include "weld_system_server_impl.h"

#include <cstdint>

#include "common/logging/application_log.h"
#include "common/messages/weld_system.h"
#include "common/zevs/zevs_socket.h"
#include "controller/controller_data.h"
#include "controller/weld_system_server.h"

namespace {

auto ConvertEnum(common::msg::weld_system::SetWeldSystemSettings::Method data) -> uint32_t {
  switch (data) {
    case common::msg::weld_system::SetWeldSystemSettings::Method::DC:
    case common::msg::weld_system::SetWeldSystemSettings::Method::DC_NEG:
    case common::msg::weld_system::SetWeldSystemSettings::Method::AC:
      return static_cast<uint32_t>(data);
    default:
      LOG_ERROR("Invalid Method: {}", static_cast<uint32_t>(data));
  }
  return -1;
}

auto ConvertEnum(common::msg::weld_system::SetWeldSystemSettings::RegulationType data) -> uint32_t {
  switch (data) {
    case common::msg::weld_system::SetWeldSystemSettings::RegulationType::CW:
    case common::msg::weld_system::SetWeldSystemSettings::RegulationType::CA:
    case common::msg::weld_system::SetWeldSystemSettings::RegulationType::CC:
      return static_cast<uint32_t>(data);
    default:
      LOG_ERROR("Invalid RegulationType: {}", static_cast<uint32_t>(data));
  }
  return -1;
}

auto ConvertEnum(common::msg::weld_system::SetWeldSystemSettings::StartType data) -> uint32_t {
  switch (data) {
    case common::msg::weld_system::SetWeldSystemSettings::StartType::DIRECT:
    case common::msg::weld_system::SetWeldSystemSettings::StartType::SCRATCH:
      return static_cast<uint32_t>(data);
    default:
      LOG_ERROR("Invalid StartType: {}", static_cast<uint32_t>(data));
  }
  return -1;
}

}  // namespace

namespace controller {

WeldSystemServerImpl::WeldSystemServerImpl(zevs::Socket* socket, WeldSystemServerObserver* observer)
    : socket_(socket),
      observer_(observer),
      weld_systems_({
          {1, {}},
          {2, {}}
}) {
  socket_->Serve(&WeldSystemServerImpl::OnGetWeldSystemData, this);
  socket_->Serve(&WeldSystemServerImpl::OnSetWeldSystemSettings, this);
  socket_->Serve(&WeldSystemServerImpl::SubscribeWeldSystemStateChanges, this);
  socket_->Serve(&WeldSystemServerImpl::UnSubscribeWeldSystemStateChanges, this);
}

void WeldSystemServerImpl::OnGetWeldSystemData(common::msg::weld_system::GetWeldSystemData data) {
  common::msg::weld_system::GetWeldSystemDataRsp rsp{};

  auto iter = weld_systems_.find(data.index);
  if (iter != weld_systems_.end()) {
    auto const& weld_system_status = iter->second;

    rsp = common::msg::weld_system::GetWeldSystemDataRsp{
        .transaction_id    = data.transaction_id,
        .voltage           = weld_system_status.get_voltage(),
        .current           = weld_system_status.get_current(),
        .wire_lin_velocity = static_cast<float>(weld_system_status.get_wire_speed() * 10. / 60.),
        .deposition_rate   = weld_system_status.get_deposition_rate(),
        .heat_input        = weld_system_status.get_heat_input(),
        .twin_wire         = weld_system_status.get_status_twin_wire(),
        .wire_diameter     = weld_system_status.get_wire_diameter(),
    };
  }

  socket_->Send(rsp);
}

void WeldSystemServerImpl::OnSetWeldSystemSettings(common::msg::weld_system::SetWeldSystemSettings data) {
  auto output = PowerSourceOutput{};

  output.set_method(ConvertEnum(data.method));
  output.set_regulation_type(ConvertEnum(data.regulation_type));
  output.set_start_adjust(data.start_adjust);
  output.set_start_type(ConvertEnum(data.start_type));
  output.set_voltage(data.voltage);
  output.set_current(data.current);
  output.set_wire_speed(data.wire_speed);
  output.set_ice_wire_speed(data.ice_wire_speed);
  output.set_ac_frequency(data.ac_frequency);
  output.set_ac_offset(data.ac_offset);
  output.set_ac_phase_shift(data.ac_phase_shift);
  output.set_crater_fill_time(data.crater_fill_time);
  output.set_burn_back_time(data.burn_back_time);

  observer_->OnPowerSourceOutput(data.index, output);
}

void WeldSystemServerImpl::SendStateChange(uint32_t index, const PowerSourceInput& data) {
  if (!state_change_subscriber_) {
    return;
  }

  common::msg::weld_system::OnWeldSystemStateChange rsp{
      .index = index,
      .state = common::msg::weld_system::OnWeldSystemStateChange::State::INIT,
  };

  if (data.get_status_error() || data.get_status_start_failure()) {
    rsp.state = common::msg::weld_system::OnWeldSystemStateChange::State::ERROR;
  } else if (data.get_status_arcing()) {
    rsp.state = common::msg::weld_system::OnWeldSystemStateChange::State::ARCING;
  } else if (data.get_status_in_welding_sequence()) {
    rsp.state = common::msg::weld_system::OnWeldSystemStateChange::State::IN_WELDING_SEQUENCE;
  } else if (data.get_status_ready_to_start()) {
    rsp.state = common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START;
  }

  socket_->Send(rsp);
}

void WeldSystemServerImpl::SubscribeWeldSystemStateChanges(
    common::msg::weld_system::SubscribeWeldSystemStateChanges /*data*/) {
  state_change_subscriber_ = true;

  for (auto [id, data] : weld_systems_) {
    SendStateChange(id, data);
  }
}

void WeldSystemServerImpl::UnSubscribeWeldSystemStateChanges(
    common::msg::weld_system::UnSubscribeWeldSystemStateChanges /*data*/) {
  state_change_subscriber_ = false;
}

void WeldSystemServerImpl::OnPowerSourceInput(uint32_t index, PowerSourceInput const& data) {
  auto const iter   = weld_systems_.find(index);
  auto const update = iter == weld_systems_.end() || iter->second.get_status_error() != data.get_status_error() ||
                      iter->second.get_status_start_failure() != data.get_status_start_failure() ||
                      iter->second.get_status_arcing() != data.get_status_arcing() ||
                      iter->second.get_status_in_welding_sequence() != data.get_status_in_welding_sequence() ||
                      iter->second.get_status_ready_to_start() != data.get_status_ready_to_start();

  weld_systems_[index] = data;

  if (update) {
    SendStateChange(index, data);
  }
}

}  // namespace controller

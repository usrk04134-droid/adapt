#include "weld_system_client_impl.h"

#include <cstdint>

#include "../weld_system_client.h"
#include "../weld_system_types.h"
#include "common/logging/application_log.h"
#include "common/messages/weld_system.h"
#include "common/zevs/zevs_socket.h"

using weld_system::WeldSystemClientImpl;
using weld_system::WeldSystemId;

WeldSystemClientImpl::WeldSystemClientImpl(zevs::Socket* socket) : socket_(socket) {
  socket_->Serve(&WeldSystemClientImpl::OnGetWeldSystemDataRsp, this);
  socket_->Serve(&WeldSystemClientImpl::OnWeldSystemStateChange, this);
}

namespace {

auto WeldSystemIdIsValid(WeldSystemId id) -> bool {
  switch (id) {
    case WeldSystemId::ID1:
    case WeldSystemId::ID2:
      return true;
      break;
    case WeldSystemId::INVALID:
    default:
      LOG_ERROR("Invalid weld system id: {}", static_cast<uint32_t>(id));
  }
  return false;
}

auto WeldSystemIdToIndex(WeldSystemId id) -> uint32_t {
  switch (id) {
    case WeldSystemId::ID1:
    case WeldSystemId::ID2:
      return static_cast<uint32_t>(id);
    case WeldSystemId::INVALID:
    default:
      LOG_ERROR("Invalid weld system id: {}", static_cast<uint32_t>(id));
  }
  return -1;
}

auto WeldSystemIdFromIndex(uint32_t index) -> WeldSystemId {
  auto const id = static_cast<WeldSystemId>(index);

  return WeldSystemIdToIndex(id) != -1 ? id : WeldSystemId::INVALID;
}

auto ConvertEnum(weld_system::WeldSystemSettings::Method data)
    -> common::msg::weld_system::SetWeldSystemSettings::Method {
  switch (data) {
    case weld_system::WeldSystemSettings::Method::DC:
    case weld_system::WeldSystemSettings::Method::DC_NEG:
    case weld_system::WeldSystemSettings::Method::AC:
      return static_cast<common::msg::weld_system::SetWeldSystemSettings::Method>(data);
    default:
      LOG_ERROR("Invalid Method: {}", static_cast<uint32_t>(data));
  }
  return static_cast<common::msg::weld_system::SetWeldSystemSettings::Method>(-1);
}

auto ConvertEnum(weld_system::WeldSystemSettings::RegulationType data)
    -> common::msg::weld_system::SetWeldSystemSettings::RegulationType {
  switch (data) {
    case weld_system::WeldSystemSettings::RegulationType::CW:
    case weld_system::WeldSystemSettings::RegulationType::CA:
    case weld_system::WeldSystemSettings::RegulationType::CC:
      return static_cast<common::msg::weld_system::SetWeldSystemSettings::RegulationType>(data);
    default:
      LOG_ERROR("Invalid RegulationType: {}", static_cast<uint32_t>(data));
  }
  return static_cast<common::msg::weld_system::SetWeldSystemSettings::RegulationType>(-1);
}

auto ConvertEnum(weld_system::WeldSystemSettings::StartType data)
    -> common::msg::weld_system::SetWeldSystemSettings::StartType {
  switch (data) {
    case weld_system::WeldSystemSettings::StartType::DIRECT:
    case weld_system::WeldSystemSettings::StartType::SCRATCH:
      return static_cast<common::msg::weld_system::SetWeldSystemSettings::StartType>(data);
    default:
      LOG_ERROR("Invalid StartType: {}", static_cast<uint32_t>(data));
  }
  return static_cast<common::msg::weld_system::SetWeldSystemSettings::StartType>(-1);
}

}  // namespace

void WeldSystemClientImpl::OnGetWeldSystemDataRsp(common::msg::weld_system::GetWeldSystemDataRsp data) {
  auto const transaction_id = data.transaction_id;
  auto iter                 = transactions_.find(transaction_id);
  if (iter != transactions_.end()) {
    auto const transaction = iter->second;
    auto const status      = WeldSystemData{
             .voltage           = data.voltage,
             .current           = data.current,
             .wire_lin_velocity = data.wire_lin_velocity,
             .deposition_rate   = data.deposition_rate,
             .heat_input        = data.heat_input,
             .twin_wire         = data.twin_wire,
             .wire_diameter     = data.wire_diameter,
    };
    transactions_.erase(iter);

    transaction.on_get_status(transaction.id, status);
  }
}

void WeldSystemClientImpl::OnWeldSystemStateChange(common::msg::weld_system::OnWeldSystemStateChange data) {
  auto const id = WeldSystemIdFromIndex(data.index);
  if (id == WeldSystemId::INVALID) {
    LOG_ERROR("Invalid index: {}", data.index);
    return;
  }

  for (auto const& [index, on_state_change] : state_change_subscribers_) {
    auto state = WeldSystemState::ERROR;
    switch (data.state) {
      case common::msg::weld_system::OnWeldSystemStateChange::State::INIT:
        state = WeldSystemState::INIT;
        break;
      case common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START:
        state = WeldSystemState::READY_TO_START;
        break;
      case common::msg::weld_system::OnWeldSystemStateChange::State::IN_WELDING_SEQUENCE:
        state = WeldSystemState::IN_WELDING_SEQUENCE;
        break;
      case common::msg::weld_system::OnWeldSystemStateChange::State::ARCING:
        state = WeldSystemState::ARCING;
        break;
      case common::msg::weld_system::OnWeldSystemStateChange::State::ERROR:
      default:
        break;
    }

    on_state_change(id, state);
  }
}

void WeldSystemClientImpl::GetWeldSystemData(WeldSystemId id, OnGetWeldSystemData on_response) {
  if (!WeldSystemIdIsValid(id)) {
    return;
  }

  auto const transaction_id = ++transaction_id_;

  transactions_[transaction_id] = Transaction{
      .id             = id,
      .transaction_id = transaction_id,
      .on_get_status  = on_response,
  };

  socket_->Send(common::msg::weld_system::GetWeldSystemData{
      .transaction_id = transaction_id,
      .index          = WeldSystemIdToIndex(id),
  });
}

void WeldSystemClientImpl::SetWeldSystemData(WeldSystemId id, WeldSystemSettings data) {
  auto msg = common::msg::weld_system::SetWeldSystemSettings{
      .index            = WeldSystemIdToIndex(id),
      .method           = ConvertEnum(data.method),
      .regulation_type  = ConvertEnum(data.regulation_type),
      .start_adjust     = data.start_adjust,
      .start_type       = ConvertEnum(data.start_type),
      .voltage          = static_cast<float>(data.voltage),
      .current          = static_cast<float>(data.current),
      .wire_speed       = static_cast<float>(data.wire_speed),
      .ice_wire_speed   = static_cast<float>(data.ice_wire_speed),
      .ac_frequency     = static_cast<float>(data.ac_frequency),
      .ac_offset        = static_cast<float>(data.ac_offset),
      .ac_phase_shift   = static_cast<float>(data.ac_phase_shift),
      .crater_fill_time = static_cast<float>(data.crater_fill_time),
      .burn_back_time   = static_cast<float>(data.burn_back_time),
  };

  socket_->Send(msg);
}

auto WeldSystemClientImpl::SubscribeWeldSystemStateChanges(OnStateChange on_state_change) -> uint32_t {
  auto const handle                 = ++subscriber_index_;
  state_change_subscribers_[handle] = on_state_change;

  if (state_change_subscribers_.size() == 1) {
    socket_->Send(common::msg::weld_system::SubscribeWeldSystemStateChanges{});
  }

  return handle;
}

void WeldSystemClientImpl::UnSubscribeWeldSystemStateChanges(uint32_t handle) {
  state_change_subscribers_.erase(handle);

  if (state_change_subscribers_.empty()) {
    socket_->Send(common::msg::weld_system::UnSubscribeWeldSystemStateChanges{});
  }
}

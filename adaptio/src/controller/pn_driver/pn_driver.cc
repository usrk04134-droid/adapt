#include "pn_driver.h"

#include <bits/chrono.h>
#include <fmt/core.h>
#include <net/if.h>
#include <pnd/common/pniobase.h>
#include <pnd/common/pnioerrx.h>
#include <pnd/common/pniousrx.h>
#include <pnd/common/servusrx.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "common/clock_functions.h"
#include "common/logging/application_log.h"
#include "controller/controller.h"
#include "controller/controller_data.h"
#include "controller/pn_driver/pn_driver_data.h"
#include "pn_driver_callbacks.h"
#include "shared_src/pnd_pntrc.h"
#include "version.h"

using controller::pn_driver::PnDriver;
using controller::pn_driver::Subslot;

std::string io_type_as_string[] = {
    "PNIO_IO_IN",
    "PNIO_IO_OUT",
    "PNIO_IO_IN_OUT",
    "PNIO_IO_UNKNOWN",
};

PnDriver::PnDriver(Configuration configuration, clock_functions::SteadyClockNowFunc steady_clock_now_func)
    : configuration_(std::move(configuration)), steady_clock_now_func_(steady_clock_now_func) {
  if (configuration_.rema_path.empty() || !configuration_.rema_path.has_filename()) {
    auto now = std::chrono::system_clock::now();
    std::chrono::hh_mm_ss hms{
        std::chrono::round<std::chrono::seconds>(now - std::chrono::floor<std::chrono::days>(now))};
    auto now_seconds =
        std::chrono::duration_cast<std::chrono::seconds>(now - std::chrono::floor<std::chrono::days>(now)).count();

    LOG_WARNING("Rema file path invalid, setting temporary path: /tmp/rema_{}", now_seconds);

    configuration_.rema_path = fmt::format("/tmp/rema_{}", now_seconds);
  }

  if (!exists(configuration_.rema_path)) {
    LOG_WARNING("Rema file does not exist, setting defaults with IP: 192.168.100.151 and name: adaptio-controller");
    char rema_buffer[] =
        R"(<Object Name="REMA">
        <ClassRID>1</ClassRID>
        <Object Name="PN Driver Device_1">
          <ClassRID>6</ClassRID>
          <Object Name="Module_0">
            <ClassRID>7</ClassRID>
            <Key AID="1">0</Key>
            <Object Name="PROFINET Interface">
              <ClassRID>9</ClassRID>
              <Key AID="2">32768</Key>
              <Variable Name="DataRecordsRema">
                <AID>11</AID>
                <Value Datatype="SparseArray" Valuetype="BLOB">
                  <Field Key="23003" Length="32">A201001C01000000001200006164617074696F2D636F6E74726F6C6C65720000</Field>
                  <Field Key="F8000000" Length="64">0000807100000000000080000000000C000000000250000801000000000000000001000000000000000080000000000C00000000F00000080100002000020000</Field>
                </Value>
              </Variable>
            </Object>
          </Object>
        </Object>
      </Object>)";

    std::ofstream rema_file;
    rema_file.open(configuration_.rema_path);
    rema_file << rema_buffer;
    rema_file.close();
  }

  LOG_INFO("Using rema file: {}", configuration_.rema_path.string());
}

auto PnDriver::Connect() -> boost::outcome_v2::result<bool> {
  Init();

  // This callback must be registered before startup
  EthernetCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) { Ethernet(std::forward<decltype(arg)>(arg)); };
  RegisterInterfaceCallback(PNIO_CBE_IFC_APPL_READY, EthernetCallback<void(PNIO_CBE_PRM*)>::callback);

  PNIO_UINT32 result = InitDevice();

  if (result != PNIO_OK) {
    Cleanup();
    return ControllerErrorCode::FAILED_TO_CONNECT;
  }

  result = StartDevice();

  if (result != PNIO_OK) {
    UndoInit();
    Cleanup();
    return ControllerErrorCode::FAILED_TO_CONNECT;
  }

  result = OpenDevice();

  if (result != PNIO_OK) {
    Shutdown();
    UndoInit();
    Cleanup();
    return false;
  }

  result = OpenInterface();

  if (result != PNIO_OK) {
    CloseDevice();
    Shutdown();
    UndoInit();
    Cleanup();
    return ControllerErrorCode::FAILED_TO_CONNECT;
  }

  RegisterCallbacks();

  result = Activate();

  if (result != PNIO_OK) {
    CloseInterface();
    CloseDevice();
    Shutdown();
    UndoInit();
    Cleanup();
    return ControllerErrorCode::FAILED_TO_CONNECT;
  }

  // ListSubmodules();

  return true;
}

void PnDriver::Disconnect() {
  Deactivate();
  CloseInterface();
  CloseDevice();
  Shutdown();
  UndoInit();
  Cleanup();
}

auto PnDriver::IsConnected() -> bool { return application_relationship_established_; }

auto PnDriver::RetrieveInputs() -> boost::outcome_v2::result<bool> {
  auto update_result = Controller::RetrieveInputs();

  if (update_result.has_error()) {
    return update_result;
  }

  if (IsConnected()) {
    auto result = ReadInputs();

    if (result != PNIO_OK) {
      return ControllerErrorCode::INPUT_ERROR;
    }

  } else {
    return ControllerErrorCode::DISCONNECTED;
  }

  return true;
}

auto PnDriver::WriteOutputs() -> boost::outcome_v2::result<bool> {
  PNIO_UINT8 buffer[512];

  this->output_image_.to_bytes(buffer);

  auto result = WriteOutput(this->input, this->input.InDataLength, buffer);

  if (result != PNIO_OK) {
    LOG_ERROR("Failed to write to module {} @ {}:{}", this->input.ModIdent, this->input.SlotNum,
              this->input.SubslotNum);
    LOG_ERROR("Error: ({}) {}", result, GetPnDriverErrorAsString(result));
    return ControllerErrorCode::OUTPUT_ERROR;
  }

  return true;
}

void PnDriver::WriteAdaptioOutput(controller::AdaptioOutput data) { this->output_image_.adaptio = data; }

void PnDriver::WriteSlideCrossXOutput(controller::AxisOutput data) { this->output_image_.slide_cross_x = data; }

void PnDriver::WriteSlideCrossYOutput(controller::AxisOutput data) { this->output_image_.slide_cross_y = data; }

void PnDriver::WriteWeldAxisOutput(controller::AxisOutput data) { this->output_image_.weld_axis = data; }

void PnDriver::WritePowerSource1Output(controller::PowerSourceOutput data) {
  this->output_image_.power_source_1 = data;
}

void PnDriver::WritePowerSource2Output(controller::PowerSourceOutput data) {
  this->output_image_.power_source_2 = data;
}

void PnDriver::WriteTrackingOutput(controller::TrackOutput data) { this->output_image_.tracking = data; }

auto PnDriver::ReadInputs() -> PNIO_UINT32 {
  PNIO_UINT8 buffer[512];

  // Since AdaptioInput processing depends on data in other structs(currently TrackInput),
  // the call to HandleAdaptioInput is done last in this function.

  InputImage image;
  auto result = ReadInput(output, buffer);

  if (result == PNIO_OK) {
    image.from_bytes(buffer);

    HandlePowerSource1Input(image.power_source_1);
    HandlePowerSource2Input(image.power_source_2);
    HandleSlideCrossXInput(image.slide_cross_x);
    HandleSlideCrossYInput(image.slide_cross_y);
    HandleWeldAxisInput(image.weld_axis);
    HandleTrackingInput(image.tracking);

    HandleAdaptioInput(image.adaptio);
  } else {
    LOG_ERROR("Failed to read from module {} @ {}:{}", output.ModIdent, output.SlotNum, output.SubslotNum);
    LOG_ERROR("Error: ({}) {}", result, GetPnDriverErrorAsString(result));
    return result;
  }

  return PNIO_OK;
}

auto PnDriver::ReadInput(const Subslot& slot, PNIO_UINT8* buffer) -> PNIO_UINT32 {
  PNIO_ADDR addr{};

  addr.AddrType      = PNIO_ADDR_GEO;
  addr.IODataType    = slot.SubProperties;
  addr.u.Geo.API     = slot.ApiNum;
  addr.u.Geo.Station = 0;
  addr.u.Geo.Slot    = slot.SlotNum;
  addr.u.Geo.Subslot = slot.SubslotNum;

  PNIO_UINT32 data_length = -1;
  PNIO_IOXS local_state   = PNIO_S_GOOD;
  PNIO_IOXS remote_state  = PNIO_S_GOOD;

  PNIO_UINT32 result = PNIO_OK;

  {
    auto lock = std::scoped_lock<std::mutex>(mutex_);

    result = PNIO_data_read(application_handle_, &addr, slot.OutDataLength, &data_length, buffer, &local_state,
                            &remote_state);
  }

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_data_read() ({} @ {}:{}): ({}) {}", slot.ModIdent, slot.SlotNum, slot.SubslotNum, result,
              GetPnDriverErrorAsString(result));
  }

  return result;
}

auto PnDriver::WriteOutput(const Subslot& slot, PNIO_UINT32 buffer_length, PNIO_UINT8* buffer) -> PNIO_UINT32 {
  PNIO_ADDR addr{};

  addr.AddrType      = PNIO_ADDR_GEO;
  addr.IODataType    = slot.SubProperties;
  addr.u.Geo.API     = slot.ApiNum;
  addr.u.Geo.Station = 0;
  addr.u.Geo.Slot    = slot.SlotNum;
  addr.u.Geo.Subslot = slot.SubslotNum;

  PNIO_IOXS local_state  = PNIO_S_GOOD;
  PNIO_IOXS remote_state = PNIO_S_GOOD;

  PNIO_UINT32 result = PNIO_OK;

  {
    auto lock = std::scoped_lock<std::mutex>(mutex_);

    result = PNIO_data_write(application_handle_, &addr, buffer_length, buffer, &local_state, &remote_state);
  }

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_data_write() ({} @ {}:{}): ({}) {}", slot.ModIdent, slot.SlotNum, slot.SubslotNum, result,
              GetPnDriverErrorAsString(result));
  }

  return result;
}

void PnDriver::Init() {
  LOG_INFO("Initing PnDriver");
  InitTraceParams(&debug_settings_);

  debug_settings_.TraceLevels[PNIO_TRACE_COMP_ACP]   = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_CLRPC] = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_CM]    = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_EDDS]  = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_EDDI]  = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_EDDT]  = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_EPS]   = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_GSY]   = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_HIF]   = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_LLDP]  = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_MEM]   = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_IP2PN] = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_NME]   = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_PNTRC] = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_PSI]   = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_SNMPX] = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_SOCK]  = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_TCIP]  = PNIO_TRACE_LEVEL_OFF;
  debug_settings_.TraceLevels[PNIO_TRACE_COMP_PND]   = PNIO_TRACE_LEVEL_OFF;
}

auto PnDriver::InitDevice() -> PNIO_UINT32 {
  LOG_INFO("Init device");

  PNIO_UINT32 result = PNIO_OK;

  result = SERV_CP_init(nullptr);
  if (result != PNIO_OK) {
    LOG_ERROR("SERV_CP_init() returned error: ({}) {}", result, GetPnDriverErrorAsString(result));
    return result;
  }

  return result;
}

auto PnDriver::StartDevice() -> PNIO_UINT32 {
  LOG_INFO("Starting device");

  PNIO_UINT32 result = PNIO_OK;

  PNIO_CP_ID_TYPE network_interface{};

  // Set network interface
  network_interface.CpSelection  = PNIO_CP_SELECT_TYPE::PNIO_CP_SELECT_WITH_MAC_ADDRESS;
  *network_interface.Description = '\0';

  if (configuration_.mac_address.size() > 0) {
    memcpy(static_cast<PNIO_UINT8*>(network_interface.CpMacAddr),
           static_cast<PNIO_UINT8*>(configuration_.mac_address.data()), 6);
    LOG_INFO("Using mac_address configuration entry: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
             network_interface.CpMacAddr[0], network_interface.CpMacAddr[1], network_interface.CpMacAddr[2],
             network_interface.CpMacAddr[3], network_interface.CpMacAddr[4], network_interface.CpMacAddr[5]);
  } else if (configuration_.interface.length() > 0) {
    auto mac_address = GetMacAddress(configuration_.interface);

    if (mac_address.size() > 0) {
      memcpy(static_cast<PNIO_UINT8*>(network_interface.CpMacAddr), static_cast<PNIO_UINT8*>(mac_address.data()), 6);
      LOG_INFO("Using interface configuration entry: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
               network_interface.CpMacAddr[0], network_interface.CpMacAddr[1], network_interface.CpMacAddr[2],
               network_interface.CpMacAddr[3], network_interface.CpMacAddr[4], network_interface.CpMacAddr[5]);
    } else {
      LOG_ERROR("Failed to get MAC-address from interface name.");
      return PNIO_ERR_INVALID_CONFIG;
    }
  } else {
    LOG_ERROR("Neither mac-address nor interface defined.");
    return PNIO_ERR_INVALID_CONFIG;
  }

  PNIO_SYSTEM_DESCR product_info;
  /** Fill product info with default values **/
  strncpy((char*)(product_info.Vendor), "ESAB", sizeof("ESAB"));
  strncpy((char*)(product_info.ProductFamily), "Adaptio", sizeof("Adaptio"));
  strncpy((char*)(product_info.IM_DeviceType), "Adaptio controller", sizeof("Adaptio controller"));
  strncpy((char*)(product_info.IM_OrderId), "6ES7195-3AA00-0YA1", sizeof("6ES7195-3AA00-0YA1"));

  product_info.IM_HwRevision                       = 0;
  product_info.IM_SwVersion.revision_prefix        = 'V';
  product_info.IM_SwVersion.functional_enhancement = ADAPTIO_VERSION_MAJOR;
  product_info.IM_SwVersion.bug_fix                = ADAPTIO_VERSION_MINOR;
  product_info.IM_SwVersion.internal_change        = ADAPTIO_VERSION_PATCH;

  strncpy((char*)(product_info.ProductSerialNr), "0", sizeof(product_info.ProductSerialNr));

  LOG_DEBUG("PND IOD:\n{}", pnd_iod_buffer);

  LOG_INFO("IM_DeviceType: {}", (char*)product_info.IM_DeviceType);
  LOG_INFO("IM_HwRevision: {}", product_info.IM_HwRevision);
  LOG_INFO("IM_OrderId: {}", (char*)product_info.IM_OrderId);
  LOG_INFO("ProductFamily: {}", (char*)product_info.ProductFamily);
  LOG_INFO("ProductSerialNr: {}", (char*)product_info.ProductSerialNr);

  std::ifstream rema_file(configuration_.rema_path);
  std::stringstream rema_buffer;
  if (rema_file.is_open()) {
    rema_buffer << rema_file.rdbuf();
    LOG_INFO("Read rema file:\n{}", rema_buffer.str());
  } else {
    LOG_ERROR("Failed to open rema file from {}", configuration_.rema_path.string());
  }
  auto rema = rema_buffer.str();

  result = SERV_CP_startup(&network_interface, 1, (PNIO_UINT8*)pnd_iod_buffer, strlen(pnd_iod_buffer),
                           (PNIO_UINT8*)rema.c_str(), strlen(rema.c_str()), &product_info, PNIO_DEVICE_ROLE);

  if (result != PNIO_OK) {
    LOG_ERROR("SERV_CP_startup(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("SERV_CP_startup(): ({}) {}", result, GetPnDriverErrorAsString(result));
  }

  if (result == PNIO_WARN_VERSION_MISMATCH) {
    result = PNIO_OK;
  } else if (result == PNIO_WARN_REALTIME_COULD_NOT_BE_SET) {
    result = PNIO_OK;
  }

  return result;
}

auto PnDriver::OpenDevice() -> PNIO_UINT32 {
  LOG_INFO("Opening device");

  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_device_open(1, &application_handle_);
  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_device_open(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Device opened");
  }

  return result;
}

auto PnDriver::OpenInterface() -> PNIO_UINT32 {
  LOG_INFO("Opening interface");

  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_interface_open(1, EthernetCallback<void(PNIO_CBE_PRM*)>::callback,
                               EthernetCallback<void(PNIO_CBE_PRM*)>::callback,
                               EthernetCallback<void(PNIO_CBE_PRM*)>::callback, &interface_handle_);
  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_interface_open(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Interface opened");
  }

  return result;
}

void PnDriver::RegisterCallbacks() {
  LOG_INFO("Registering callbacks");

  using controller::pn_driver::ConnectedCallback;
  using controller::pn_driver::DiagnosticsCallback;
  using controller::pn_driver::DisconnectedCallback;
  using controller::pn_driver::EthernetCallback;
  using controller::pn_driver::InDataCallback;
  using controller::pn_driver::OwnershipCallback;
  using controller::pn_driver::PrmEndCallback;
  using controller::pn_driver::ReadCallback;
  using controller::pn_driver::WriteCallback;

  ConnectedCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) { Connected(std::forward<decltype(arg)>(arg)); };
  RegisterCallback(PNIO_CBE_DEV_CONNECT_IND, ConnectedCallback<void(PNIO_CBE_PRM*)>::callback);

  DisconnectedCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) {
    Disconnected(std::forward<decltype(arg)>(arg));
  };
  RegisterCallback(PNIO_CBE_DEV_DISCONNECT_IND, DisconnectedCallback<void(PNIO_CBE_PRM*)>::callback);

  OwnershipCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) { Ownership(std::forward<decltype(arg)>(arg)); };
  RegisterCallback(PNIO_CBE_DEV_OWNERSHIP_IND, OwnershipCallback<void(PNIO_CBE_PRM*)>::callback);

  PrmEndCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) { PrmEnd(std::forward<decltype(arg)>(arg)); };
  RegisterCallback(PNIO_CBE_DEV_PRMEND_IND, PrmEndCallback<void(PNIO_CBE_PRM*)>::callback);

  InDataCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) { InData(std::forward<decltype(arg)>(arg)); };
  RegisterCallback(PNIO_CBE_DEV_INDATA_IND, InDataCallback<void(PNIO_CBE_PRM*)>::callback);

  DiagnosticsCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) {
    Diagnostics(std::forward<decltype(arg)>(arg));
  };
  RegisterCallback(PNIO_CBE_DEV_DIAG_CONF, DiagnosticsCallback<void(PNIO_CBE_PRM*)>::callback);

  ReadCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) { ReadRecord(std::forward<decltype(arg)>(arg)); };
  RegisterCallback(PNIO_CBE_REC_READ_CONF, ReadCallback<void(PNIO_CBE_PRM*)>::callback);

  WriteCallback<void(PNIO_CBE_PRM*)>::func = [this](auto&& arg) { WriteRecord(std::forward<decltype(arg)>(arg)); };
  RegisterCallback(PNIO_CBE_REC_WRITE_CONF, WriteCallback<void(PNIO_CBE_PRM*)>::callback);

  RegisterInterfaceCallback(PNIO_CBE_IFC_SET_ADDR_CONF, EthernetCallback<void(PNIO_CBE_PRM*)>::callback);
  RegisterInterfaceCallback(PNIO_CBE_REMA_READ_CONF, EthernetCallback<void(PNIO_CBE_PRM*)>::callback);
}

auto PnDriver::RegisterCallback(PNIO_CBE_TYPE type, PNIO_CBF callback) const -> PNIO_UINT32 {
  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_register_cbf(application_handle_, type, callback);

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_register_cbf(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Registered callback of type {}", (uint32_t)type);
  }

  return result;
}

auto PnDriver::RegisterInterfaceCallback(PNIO_CBE_TYPE type, PNIO_CBF callback) const -> PNIO_UINT32 {
  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_interface_register_cbf(application_handle_, type, callback);

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_interface_register_cbf(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Registered interface callback of type {}", (uint32_t)type);
  }

  return result;
}

auto PnDriver::Activate() -> PNIO_UINT32 {
  LOG_INFO("Activating device");

  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_device_activate(application_handle_, nullptr, PNIO_DA_TRUE);

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_device_activate(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Device activated");
  }

  return result;
}

auto PnDriver::Deactivate() -> PNIO_UINT32 {
  LOG_INFO("Dectivating device");

  application_relationship_established_ = false;

  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_device_activate(application_handle_, nullptr, PNIO_DA_FALSE);

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_device_activate(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Device deactivated");
  }

  return result;
}

auto PnDriver::CloseDevice() -> PNIO_UINT32 {
  LOG_INFO("Closing device");

  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_device_close(application_handle_);

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_device_close(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Device closed");
  }

  return result;
}

auto PnDriver::CloseInterface() -> PNIO_UINT32 {
  LOG_INFO("Closing interface");

  PNIO_UINT32 result = PNIO_OK;

  result = PNIO_interface_close(interface_handle_);

  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_interface_close(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Interface closed");
  }

  return result;
}

auto PnDriver::Shutdown() -> PNIO_UINT32 {
  LOG_INFO("Shutting down device");

  PNIO_UINT32 result = PNIO_OK;

  result = SERV_CP_shutdown();

  if (result != PNIO_OK) {
    LOG_ERROR("SERV_CP_shutdown(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Shutdown complete");
  }

  return result;
}

auto PnDriver::UndoInit() -> PNIO_UINT32 {
  LOG_INFO("Undo device init");

  PNIO_UINT32 result = PNIO_OK;

  result = SERV_CP_undo_init();

  if (result != PNIO_OK) {
    LOG_ERROR("SERV_CP_undo_init(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    LOG_INFO("Undid init");
  }

  return result;
}

void PnDriver::Cleanup() {
  LOG_INFO("Cleaning up resources");

  ReleaseTraceResources();

  LOG_INFO("Trace resources cleaned up");
}

auto PnDriver::GetMacAddress(const std::string& interface_name) -> std::vector<uint8_t> {
  std::vector<uint8_t> mac{};
  mac.resize(6);

  int file_descriptor = -1;

  ifreq interface_request{};

  file_descriptor = socket(AF_INET, SOCK_DGRAM, 0);

  std::array<char, 255> error_message{};

  if (file_descriptor == -1) {
    auto err = errno;
    (void)strerror_r(err, error_message.data(), error_message.size());
    LOG_ERROR("Failed to open socket: {}", error_message.data());
    return {};
  }

  interface_request.ifr_addr.sa_family = AF_INET;
  strncpy(static_cast<char*>(interface_request.ifr_name), static_cast<const char*>(interface_name.c_str()),
          IFNAMSIZ - 1);

  if (ioctl(file_descriptor, SIOCGIFHWADDR, &interface_request) == -1) {
    auto err = errno;
    (void)strerror_r(err, error_message.data(), error_message.size());
    LOG_ERROR("Interface request failed: {}", error_message.data());
    close(file_descriptor);
    return {};
  }

  close(file_descriptor);

  mac[0] = interface_request.ifr_ifru.ifru_addr.sa_data[0];
  mac[1] = interface_request.ifr_ifru.ifru_addr.sa_data[1];
  mac[2] = interface_request.ifr_ifru.ifru_addr.sa_data[2];
  mac[3] = interface_request.ifr_ifru.ifru_addr.sa_data[3];
  mac[4] = interface_request.ifr_ifru.ifru_addr.sa_data[4];
  mac[5] = interface_request.ifr_ifru.ifru_addr.sa_data[5];

  LOG_DEBUG("Interface {} resolved to MAC-address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", interface_name,
            mac.data()[0], mac.data()[1], mac.data()[2], mac.data()[3], mac.data()[4], mac.data()[5]);

  return mac;
}

void PnDriver::ListSubmodules() {
  PND_IOD_LIST_MODULES_TYPE modules;

  PNIO_UINT32 result = PNIO_list_submodules(application_handle_, &modules);
  if (result != PNIO_OK) {
    LOG_ERROR("PNIO_list_submodules(): ({}) {}", result, GetPnDriverErrorAsString(result));
  } else {
    if (modules.SubmoduleCount != 0) {
      LOG_INFO("{} submodules configured for PROFINET device", modules.SubmoduleCount);

      auto* p_sub = (PNIO_CTRL_DIAG_CONFIG_SUBMODULE*)modules.pListSubmoduleBuffer;

      PNIO_CTRL_DIAG_CONFIG_SUBMODULE submodule{};
      for (size_t i = 3; i < modules.SubmoduleCount; i++) {
        std::memcpy(&submodule, &p_sub[i], sizeof(PNIO_CTRL_DIAG_CONFIG_SUBMODULE));
        // const char* submodule_type = (pSub[i].Address.IODataType == PNIO_IO_UNKNOWN)
        //                                 ? "DATALESS"
        //                                 : ((pSub[i].Address.IODataType == PNIO_IO_IN)
        //                                        ? "IN"
        //                                        : ((pSub[i].Address.IODataType == PNIO_IO_IN_OUT) ? "INOUT" : "OUT"));
        LOG_INFO("API:={} Slot:={} SubSlot:={} Module Ident:={} Submodule Ident:={} InLen={} OutLen={}", submodule.API,
                 submodule.Slot, submodule.Subslot, submodule.ModIdent, submodule.SubModIdent, submodule.InDataLength,
                 submodule.OutDataLength);
      }
    } else {
      LOG_WARNING("No submodules configured for PROFINET device");
    }
  }
}

// Callbacks
void PnDriver::Connected(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  auto data = p->u.ConnectInd;
  LOG_INFO("Callback: AR is connected");
  LOG_INFO("AR: {}", data.ArNum);
  LOG_INFO("Session key: {}", data.ArSessionKey);
  PNIO_UINT8* ip = ((PNIO_UINT8*)(&data.HostIp));
  LOG_INFO("Host: {}.{}.{}.{}", ip[0], ip[1], ip[2], ip[3]);
  LOG_INFO("Send clock: {}", data.SendClock);
  LOG_INFO("Reduction ratio (in): {}", data.ReductionRatioIn);
  LOG_INFO("Reduction ratio (out): {}", data.ReductionRatioOut);
}

void PnDriver::Disconnected(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  application_relationship_established_ = false;

  auto data = p->u.DisconnectInd;

  LOG_INFO("Callback: AR got disconnected: AR = {}, Session key = {}, Reason: ({}) {}", data.ArNum, data.SessionKey,
           data.ReasonCode, GetPnDriverReasonCodeAsString(data.ReasonCode));
}

void PnDriver::Ownership(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);
  LOG_INFO("Callback: Result of the ownership change indication arrived");

  auto data = p->u.OwnershipInd;

  for (int i = 0; i < data.NrOfExpSubmodule; i++) {
    auto submod = data.pExpSubmodules[i];

    if (!submod.IsWrongSubmod) {
      if (submod.SlotNum == 1 && submod.SubSlotNum == 1) {
        this->output = {
            .ApiNum        = submod.ApiNum,
            .SlotNum       = submod.SlotNum,
            .SubslotNum    = submod.SubSlotNum,
            .ModIdent      = submod.ModIdent,
            .SubIdent      = submod.SubIdent,
            .OwnSessionKey = submod.OwnSessionKey,
            .IsWrongSubmod = submod.IsWrongSubmod,
            .SubProperties = submod.SubProperties,
            .InDataLength  = submod.InDataLength,
            .OutDataLength = submod.OutDataLength,
        };
      } else if (submod.SlotNum == 2 && submod.SubSlotNum == 1) {
        this->input = {
            .ApiNum        = submod.ApiNum,
            .SlotNum       = submod.SlotNum,
            .SubslotNum    = submod.SubSlotNum,
            .ModIdent      = submod.ModIdent,
            .SubIdent      = submod.SubIdent,
            .OwnSessionKey = submod.OwnSessionKey,
            .IsWrongSubmod = submod.IsWrongSubmod,
            .SubProperties = submod.SubProperties,
            .InDataLength  = submod.InDataLength,
            .OutDataLength = submod.OutDataLength,
        };
      }
    }

    LOG_DEBUG("Submodule: [");
    LOG_DEBUG("  ApiNum        = {}", submod.ApiNum);
    LOG_DEBUG("  SlotNum       = {}", submod.SlotNum);
    LOG_DEBUG("  SubSlotNum    = {}", submod.SubSlotNum);
    LOG_DEBUG("  ModIdent      = {:#08x}", submod.ModIdent);
    LOG_DEBUG("  SubIdent      = {:#08x}", submod.SubIdent);
    LOG_DEBUG("  OwnSessionKey = {}", submod.OwnSessionKey);
    LOG_DEBUG("  IsWrongSubmod = {}", submod.IsWrongSubmod);
    LOG_DEBUG("  SubProperties = {}", io_type_as_string[submod.SubProperties]);
    LOG_DEBUG("  InDataLength  = {}", submod.InDataLength);
    LOG_DEBUG("  OutDataLength = {}", submod.OutDataLength);
    LOG_DEBUG("]\n");
  }
}

void PnDriver::PrmEnd(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  LOG_INFO("Callback: Parameterization of the device is complete");

  PNIO_ADDR addr{};
  PNIO_UINT8 input_data;
  PNIO_IOXS local_state  = PNIO_S_GOOD;
  PNIO_IOXS remote_state = PNIO_S_GOOD;
  PNIO_UINT32 result     = PNIO_OK;

  auto data = p->u.PrmendInd;

  for (PNIO_UINT16 i = 0; i < data.NrOfElem; i++) {
    LOG_INFO("\tApi: {}, Slot: {}, Subslot: {}, IO Type: {}", data.pAddr[i].u.Geo.API, data.pAddr[i].u.Geo.Slot,
             data.pAddr[i].u.Geo.Subslot, (PNIO_UINT32)data.pAddr[i].IODataType);
  }

  addr.AddrType = PNIO_ADDR_GEO;

  {
    const auto& slot   = this->output;
    addr.u.Geo.API     = slot.ApiNum;
    addr.u.Geo.Slot    = slot.SlotNum;
    addr.u.Geo.Subslot = slot.SubslotNum;
    addr.IODataType    = slot.SubProperties;

    // This updates the local state of the submodule
    if (slot.SubProperties == PNIO_IO_IN_OUT) {
      if (slot.IsWrongSubmod == PNIO_FALSE) {
        // Update Input buffer local state (IOPS)
        addr.IODataType = PNIO_IO_IN;
        result          = PNIO_data_write(application_handle_, &addr, 0, &input_data, &local_state, &remote_state);

        // Update Output buffer local state (IOCS)
        addr.IODataType = PNIO_IO_OUT;
        result          = PNIO_data_write(application_handle_, &addr, 0, &input_data, &local_state, &remote_state);
      }
    } else if ((slot.SubProperties == PNIO_IO_IN) || (slot.SubProperties == PNIO_IO_OUT)) {
      if (slot.IsWrongSubmod == PNIO_FALSE) {
        result = PNIO_data_write(application_handle_, &addr, 0, &input_data, &local_state, &remote_state);
      }
    } else {
      // Data less submodules, no update
    }

    if (result != PNIO_OK) {
      LOG_ERROR("PNIO_data_write(): ({}) {}", result, GetPnDriverErrorAsString(result));
    }
  }

  {
    const auto& slot   = this->input;
    addr.u.Geo.API     = slot.ApiNum;
    addr.u.Geo.Slot    = slot.SlotNum;
    addr.u.Geo.Subslot = slot.SubslotNum;
    addr.IODataType    = slot.SubProperties;

    // This updates the local state of the submodule
    if (slot.SubProperties == PNIO_IO_IN_OUT) {
      if (slot.IsWrongSubmod == PNIO_FALSE) {
        // Update Input buffer local state (IOPS)
        addr.IODataType = PNIO_IO_IN;
        result          = PNIO_data_write(application_handle_, &addr, 0, &input_data, &local_state, &remote_state);

        // Update Output buffer local state (IOCS)
        addr.IODataType = PNIO_IO_OUT;
        result          = PNIO_data_write(application_handle_, &addr, 0, &input_data, &local_state, &remote_state);
      }
    } else if ((slot.SubProperties == PNIO_IO_IN) || (slot.SubProperties == PNIO_IO_OUT)) {
      if (slot.IsWrongSubmod == PNIO_FALSE) {
        result = PNIO_data_write(application_handle_, &addr, 0, &input_data, &local_state, &remote_state);
      }
    } else {
      // Data less submodules, no update
    }

    if (result != PNIO_OK) {
      LOG_ERROR("PNIO_data_write(): ({}) {}", result, GetPnDriverErrorAsString(result));
    }
  }

  LOG_INFO("Parameterization complete");
}

void PnDriver::InData(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  application_relationship_established_ = true;
  LOG_INFO("Callback: PROFINET device is in data exchange");
  LOG_INFO("AR: {}", p->u.IndataInd.ArNum);
  LOG_INFO("Session key: {}", p->u.IndataInd.SessionKey);
}

void PnDriver::Diagnostics(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  LOG_INFO("Callback: Result of diagnostics request");

  auto data = p->u.DevDiagConf;

  if (data.pDiagData->DiagMode == DIAGNOSIS_APPEAR) {
    if (data.ErrorCode == PNIO_RET_OK) {
      LOG_DEBUG("Diagnosis Appear request completed successfully");
    } else {
      LOG_WARNING("Diagnosis Appear request failed with error code: ({}) {}", data.ErrorCode,
                  GetPnDriverErrorAsString(data.ErrorCode));
    }
  } else {
    if (data.ErrorCode == PNIO_RET_OK) {
      LOG_DEBUG("Diagnosis Disappear request completed successfully");
    } else {
      LOG_WARNING("Diagnosis Disappear request failed with error code: ({}) {}", data.ErrorCode,
                  GetPnDriverErrorAsString(data.ErrorCode));
    }
  }
}

void PnDriver::ReadRecord(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  LOG_DEBUG("Callback: A data record read request from PROFINET controller has arrived");

  auto* record_read = &p->u.RecReadConf;

  LOG_DEBUG("Record read index: {:#x}", record_read->RecordIndex);

  PNIO_UINT32 status = PNIO_OK;

  switch (record_read->RecordIndex) {
    case 0xF880: {  // AMR
    } break;
    case 0xAFF0: {  // IM 0
    } break;
    case 0xAFF1: {  // IM 1
    } break;
    case 0xAFF2: {  // IM 2
    } break;
    case 0xAFF3: {  // IM 3
    } break;
    case 0xAFF4: {  // IM 4
    } break;
    default: {
    }
  }

  record_read->Response = status;
}

void PnDriver::WriteRecord(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  // This part handles the remanent data, just send OK for now.
  p->u.RecWriteConf.Response = PNIO_OK;

  LOG_DEBUG("Callback: A data record write request from PROFINET controller has arrived");
}

void PnDriver::Ethernet(PNIO_CBE_PRM* p) {
  auto lock = std::scoped_lock<std::mutex>(mutex_);

  auto data = p->u.SetAddrConf;

  switch (p->CbeType) {
    case PNIO_CBE_IFC_SET_ADDR_CONF: {
      if (data.Err == PNIO_OK) {
        if (data.Mode == PNIO_SET_NOS_MODE) {
          if (data.pStationName != nullptr && data.StationNameLen > 0) {
            LOG_INFO("Interface: Station name set to {}", (char*)data.pStationName);
            std::stringstream stream;
            stream << data.pStationName;
            station_name_ = stream.str();
          }
        } else if (data.Mode == PNIO_SET_IP_MODE) {
          if (data.LocalIPAddress[0] != 0) {
            LOG_INFO("Interface: IP configuration os interface is set to:");
            LOG_INFO("  IP: {}.{}.{}.{}", data.LocalIPAddress[0], data.LocalIPAddress[1], data.LocalIPAddress[2],
                     data.LocalIPAddress[3]);

            LOG_INFO("  Net mask: {}.{}.{}.{}", data.LocalSubnetMask[0], data.LocalSubnetMask[1],
                     data.LocalSubnetMask[2], data.LocalSubnetMask[3]);

            LOG_INFO("  Default router: {}.{}.{}.{}", data.DefaultRouterAddr[0], data.DefaultRouterAddr[1],
                     data.DefaultRouterAddr[2], data.DefaultRouterAddr[3]);

            auto ip = new PNIO_UINT8[4];
            ip[0]   = data.LocalIPAddress[0];
            ip[1]   = data.LocalIPAddress[1];
            ip[2]   = data.LocalIPAddress[2];
            ip[3]   = data.LocalIPAddress[3];

            ip_address_ = ip;
          } else {
            LOG_INFO("Interface: Address settings have been changed without mode");
          }
        }
      } else {
        LOG_ERROR("Failed to set IP suite and/or name of station: ({}) {}", data.Err,
                  GetPnDriverErrorAsString(data.Err));
      }
    } break;
    case PNIO_CBE_REMA_READ_CONF: {
      LOG_INFO("Interface: Change in remanent data occurred");
      auto buffer = (char*)p->u.RemaReadConf.RemaXMLBuffer;
      LOG_INFO("{}", buffer);

      std::ofstream rema_file(configuration_.rema_path, std::ios::trunc);
      rema_file << buffer;
      rema_file.close();
    } break;
    case PNIO_CBE_IFC_REC_READ_CONF:
      LOG_INFO("Interface: Result of the read data record job arrived");
      break;
    case PNIO_CBE_IFC_REC_WRITE_CONF:
      LOG_INFO("Interface: Result of the write data record job arrived");
      break;
    case PNIO_CBE_IFC_ALARM_IND: {
      auto* alarm_data = p->u.AlarmInd.pAlarmData;
      LOG_WARNING("Interface: Interface alarm arrived: {}:{} - ", alarm_data->SlotNum, alarm_data->SubslotNum,
                  (PNIO_UINT32)alarm_data->AlarmType);
    } break;
    case PNIO_CBE_IFC_APPL_READY: {
      LOG_INFO("Interface: Parametrization of the local ethernet interface is complete");
    } break;
    default:
      LOG_WARNING("Unknown interface callback type: {}", (uint32_t)p->CbeType);
  }
}

const std::string PnDriver::GetPnDriverReasonCodeAsString(const PNIO_UINT32 reason_code) {
  switch (reason_code) {
    case CM_AR_REASON_NONE:
      return "no error";
    case CM_AR_REASON_1:
      return "sequence numbers do not match (no longer used in versions >= V3.9";
    case CM_AR_REASON_2:
      return "alarm instance closed (no longer used in versions >= V3.6)";
    case CM_AR_REASON_MEM:
      return "out of mem";
    case CM_AR_REASON_FRAME:
      return "add provider or consumer failed";
    case CM_AR_REASON_MISS:
      return "miss (consumer)";
    case CM_AR_REASON_TIMER:
      return "cmi timeout";
    case CM_AR_REASON_ALARM:
      return "alarm-open failed";
    case CM_AR_REASON_ALSND:
      return "alarm-send.cnf(-)";
    case CM_AR_REASON_ALACK:
      return "alarm-ack-send.cnf(-)";
    case CM_AR_REASON_ALLEN:
      return "alarm-data too long";
    case CM_AR_REASON_ASRT:
      return "alarm.ind(err)";
    case CM_AR_REASON_RPC:
      return "rpc-client call.cnf(-)";
    case CM_AR_REASON_ABORT:
      return "ar-abort.req";
    case CM_AR_REASON_RERUN:
      return "rerun aborts existing";
    case CM_AR_REASON_REL:
      return "got release.ind";
    case CM_AR_REASON_PAS:
      return "device passivated";
    case CM_AR_REASON_RMV:
      return "device / AR removed";
    case CM_AR_REASON_PROT:
      return "protocol violation";
    case CM_AR_REASON_NARE:
      return "NARE error";
    case CM_AR_REASON_BIND:
      return "RPC-Bind error";
    case CM_AR_REASON_CONNECT:
      return "RPC-Connect error";
    case CM_AR_REASON_READ:
      return "RPC-Read error";
    case CM_AR_REASON_WRITE:
      return "RPC-Write error";
    case CM_AR_REASON_CONTROL:
      return "RPC-Control error";
    case CM_AR_REASON_25:
      return "reserved (formerly: pull or plug in forbidden window)";
    case CM_AR_REASON_26:
      return "reserved (formerly: AP removed)";
    case CM_AR_REASON_LNK_DOWN:
      return "link down";
    case CM_AR_REASON_28:
      return "reserved (formerly: could not register multicast-mac)";
    case CM_AR_REASON_SYNC:
      return "not synchronized (cannot start companion-ar)";
    case CM_AR_REASON_30:
      return "reserved (formerly: wrong topology (cannot start companion-ar))";
    case CM_AR_REASON_DCP_NAME:
      return "dcp, station-name changed";
    case CM_AR_REASON_DCP_RESET:
      return "dcp, reset to factory or factory reset";
    case CM_AR_REASON_33:
      return "reserved (formerly: cannot start companion-ar)";
    case CM_AR_REASON_IRDATA:
      return "no irdata record yet";
    case CM_AR_REASON_PDEV:
      return "ownership of physical device";
    case CM_AR_REASON_LNK_MODE:
      return "link mode not full duplex";
    case CM_AR_REASON_IPSUITE:
      return "IP-Suite [of the IOC] changed by means of DCP_set(IPParameter) or local engineering";
    case CM_AR_REASON_RDHT:
      return "IOCARSR RDHT expired";
    case CM_AR_REASON_PDEV_PRM:
      return "IOCARSR PDev, parametrization impossible (AP01238541)";
    case CM_AR_REASON_ARDY:
      return "Remote application timeout expired";
    case CM_AR_REASON_41_UNUSED:
      return "IOCARSR Redundant interface lost or access to the peripherals impossible";
    case CM_AR_REASON_MTOT:
      return "IOCARSR MTOT expired";
    case CM_AR_REASON_43_UNUSED:
      return "IOCARSR AR protocol violation";
    case CM_AR_REASON_COC:
      return "Pdev, plug port without CombinedObjectContainer";
    case CM_AR_REASON_NME:
      return "NME, no or wrong configuration";
    default:
      return "Unknown reason";
  }
}

const std::string PnDriver::GetPnDriverErrorAsString(PNIO_UINT32 error_code) {
  switch (error_code) {
    case PNIO_OK: {
      return "PNIO SUCCESS";
    }
    case PNIO_WARN_IRT_INCONSISTENT: {
      return "IRT Data may be inconsistent";
    }
    case PNIO_WARN_NO_SUBMODULES: {
      return "no submodules to be updated";
    }
    case PNIO_WARN_LOCAL_STATE_BAD: {
      return "data was written with local state PNIO_S_BAD, because not all components of splitted module have local "
             "state PNIO_S_GOOD";
    }
    case PNIO_WARN_NO_DECENTRALIOSYSTEM: {
      return "no decentral io system is found in the configuration";
    }
    case PNIO_WARN_VERSION_MISMATCH: {
      return "the version in hardware configuration is different from the version that is running";
    }
    case PNIO_WARN_REALTIME_COULD_NOT_BE_SET: {
      return "realtime priority could not be set";
    }
    case PNIO_ERR_PRM_HND: {
      return "parameter Handle is illegal";
    }
    case PNIO_ERR_PRM_BUF: {
      return "parameter buffer is NULL-Ptr";
    }
    case PNIO_ERR_PRM_LEN: {
      return "parameter length is wrong";
    }
    case PNIO_ERR_PRM_ADD: {
      return "parameter address is wrong";
    }
    case PNIO_ERR_PRM_RSTATE: {
      return "parameter remote state is NULL-Ptr";
    }
    case PNIO_ERR_PRM_CALLBACK: {
      return "parameter cbf is illegal";
    }
    case PNIO_ERR_PRM_TYPE: {
      return "parameter type has no valid value";
    }
    case PNIO_ERR_PRM_EXT_PAR: {
      return "parameter ExtPar has no valid value";
    }
    case PNIO_ERR_PRM_IO_TYPE: {
      return "parameter PNIO_ADDR::IODataType is wrong";
    }
    case PNIO_ERR_PRM_CP_ID: {
      return "parameter CpIndex is wrong, propably driver is not loaded";
    }
    case PNIO_ERR_PRM_LOC_STATE: {
      return "parameter IOlocState has no valid value";
    }
    case PNIO_ERR_PRM_REC_INDEX: {
      return "parameter RecordIndex has no valid value";
    }
    case PNIO_ERR_PRM_TIMEOUT: {
      return "parameter timeout has no valid value";
    }
    case PNIO_ERR_PRM_DEV_ANNOTATION: {
      return "parameter annotation has no valid value";
    }
    case PNIO_ERR_PRM_DEV_STATE: {
      return "parameter state has no valid value";
    }
    case PNIO_ERR_PRM_PCBF: {
      return "parameter pCbf has no valid value";
    }
    case PNIO_ERR_PRM_MAX_AR_VALUE: {
      return "parameter MaxAR has no valid value";
    }
    case PNIO_ERR_PRM_ACCESS_TYPE: {
      return "parameter AccessType has no valid value";
    }
    case PNIO_ERR_PRM_POINTER: {
      return "an invalid pointer was passed";
    }
    case PNIO_ERR_PRM_INVALIDARG: {
      return "an invalid argument was passed";
    }
    case PNIO_ERR_PRM_MEASURE_NUMBER: {
      return "wrong Measure No in cycle statistics, must be -1 (actual measure) up to 49";
    }
    case PNIO_ERR_PRM_CYCLE_OFFSET: {
      return "wrong Offset for cycle info buffer (must be 0 to 19)";
    }
    case PNIO_ERR_PRM_ROUTER_ADD: {
      return "address used by io router";
    }
    case PNIO_ERR_PRM_IP: {
      return "parameter IP has no valid value";
    }
    case PNIO_ERR_PRM_NOS: {
      return "parameter NoS has no valid value";
    }
    case PNIO_ERR_PRM_NOS_LEN: {
      return "parameter length is wrong";
    }
    case PNIO_ERR_PRM_CONSISTENCY: {
      return "the parameterization is not consistent";
    }
    case PNIO_ERR_PRM_INDEX: {
      return "unknown index (PrmWrite and PrmRead)";
    }
    case PNIO_ERR_PRM_PORTID: {
      return "port-id does not match with index (PrmWrite and PrmRead)";
    }
    case PNIO_ERR_PRM_DATA: {
      return "data-length too short (PrmRead) or data-length not consistent with block-structure (PrmWrite)";
    }
    case PNIO_ERR_PRM_BLOCK: {
      return "wrong block-type/version or something wrong with the block-data (PrmWrite)";
    }
    case PNIO_ERR_PRM_VERSION:  // check is there any internally use scneraio for us, otherwise remove it
    {
      return "internally used";
    }
    case PNIO_ERR_WRONG_HND: {
      return "unknown handle";
    }
    case PNIO_ERR_MAX_REACHED: {
      return "maximal number of opens reached; close unused applications";
    }
    case PNIO_ERR_CREATE_INSTANCE: {
      return "fatal error, reboot your system";
    }
    case PNIO_ERR_MODE_VALUE: {
      return "parameter mode has no valid value";
    }
    case PNIO_ERR_OPFAULT_NOT_REG: {
      return "register OPFAULT callback before register STARTOP callback";
    }
    case PNIO_ERR_NEWCYCLE_SEQUENCE_REG: {
      return "register NEWCYCLE callback before register STARTOP callback";
    }
    case PNIO_ERR_NETWORK_PROT_NOT_AVAILABLE: {
      return "network protocol not available, check card configuration";
    }
    case PNIO_ERR_RETRY: {
      return "pnio stack not available, try again later";
    }
    case PNIO_ERR_NO_CONNECTION: {
      return "device data not available, because device is not connected to controller";
    }
    case PNIO_ERR_OS_RES: {
      return "fatal error, no more operation system resources available";
    }
    case PNIO_ERR_ALREADY_DONE: {
      return "action was already performed";
    }
    case PNIO_ERR_NO_CONFIG: {
      return "no configuration for this index available";
    }
    case PNIO_ERR_SET_MODE_NOT_ALLOWED: {
      return "PNIO_set_mode not allowed, use PNIO_CEP_MODE_CTRL by PNIO_controller_open";
    }
    case PNIO_ERR_DEV_ACT_NOT_ALLOWED: {
      return "PNIO_device_activate not allowed, use PNIO_CEP_MODE_CTRL by PNIO_controller_open";
    }
    case PNIO_ERR_NO_LIC_SERVER: {
      return "licence server not running, check your installation";
    }
    case PNIO_ERR_VALUE_LEN: {
      return "wrong length value";
    }
    case PNIO_ERR_SEQUENCE: {
      return "wrong calling sequence";
    }
    case PNIO_ERR_INVALID_CONFIG: {
      return "invalid configuration, check your configuration";
    }
    case PNIO_ERR_UNKNOWN_ADDR: {
      return "address unknown in configuration, check your configuration";
    }
    case PNIO_ERR_NO_RESOURCE: {
      return "no resouce too many requests been processed";
    }
    case PNIO_ERR_CONFIG_IN_UPDATE: {
      return "configuration update is in progress or CP is in STOP state, try again later";
    }
    case PNIO_ERR_NO_FW_COMMUNICATION: {
      return "no communication with firmware, reset cp or try again later";
    }
    case PNIO_ERR_STARTOP_NOT_REGISTERED: {
      return "no synchonous function allowed, use PNIO_CEP_SYNC_MODE by PNIO_controller_open or PNIO_device_open";
    }
    case PNIO_ERR_OWNED: {
      return "interface-submodule cannot be removed because it is owned by an AR";
    }
    case PNIO_ERR_START_THREAD_FAILED: {
      return "failed to start thread, propably by lack of pthread resources";
    }
    case PNIO_ERR_START_RT_THREAD_FAILED: {
      return "failed to start realtime thread, propably you need root capability to do it";
    }
    case PNIO_ERR_DRIVER_IOCTL_FAILED: {
      return "failed in ioctl driver, propably API version mismatch";
    }
    case PNIO_ERR_AFTER_EXCEPTION: {
      return "exception ocurred, save exception info (see manual) and reset cp";
    }
    case PNIO_ERR_NO_CYCLE_INFO_DATA: {
      return "no cycle data available";
    }
    case PNIO_ERR_SESSION: {
      return "request belongs to an old session";
    }
    case PNIO_ERR_ALARM_DATA_FORMAT: {
      return "wrong format of alarm data";
    }
    case PNIO_ERR_ABORT: {
      return "operation was aborted";
    }
    case PNIO_ERR_CORRUPTED_DATA: {
      return "datas are corrupted or have wrong format";
    }
    case PNIO_ERR_FLASH_ACCESS: {
      return "error by flash operations";
    }
    case PNIO_ERR_WRONG_RQB_LEN: {
      return "wrong length of request block at firmware interface, firmware not compatible to host sw";
    }
    case PNIO_ERR_NO_RESET_VERIFICATION: {
      return "reset request was sent to firmware, but firmware rut up can't be verified";
    }
    case PNIO_ERR_SET_IP_NOS_NOT_ALLOWED: {
      return "setting IP and/or NoS is not allowed";
    }
    case PNIO_ERR_READ_IP_NOS_NOT_ALLOWED: {
      return "reading IP and/or NoS is not allowed";
    }
    case PNIO_ERR_INVALID_REMA: {
      return "rema data is not valid";
    }
    case PNIO_ERR_NOT_REENTERABLE: {
      return "the function is not reenterable";
    }
    case PNIO_ERR_INVALID_STATION: {
      return "station not exists or not configured as optional";
    }
    case PNIO_ERR_INVALID_PORT: {
      return "port not configured as programmable peer";
    }
    case PNIO_ERR_NO_ADAPTER_FOUND: {
      return "no ethernet adapter found";
    }
    case PNIO_ERR_ACCESS_DENIED: {
      return "access denied";
    }
    case PNIO_ERR_VARIANT_MISMATCH: {
      return "the variant in hardware configuration is different from the variant that is running";
    }
    case PNIO_ERR_MEMORY_OPERATION: {
      return "memory allocation or deallocation error";
    }
    case PNIO_ERR_UNEXPECTED_CM_ERROR: {
      return "Cannot map cm error code to pnio error code";
    }
    case PNIO_ERR_EDD_STATUS: {
      return "ISO application sync error caused by EDD status";
    }
    case PNIO_ERR_NO_SUDO_PRIV_RIGHTS: {
      return "Application is not started with sudo priveleges";
    }
    case PNIO_ERR_STARTUP_ARG_ERROR: {
      return "Startup Argument(s) is not recognized";
    }
    case PNIO_ERR_INTERNAL: {
      return "fatal error, contact devkits support at profinet.devkits.industry@siemens.com";
    }
    default: {
      break;
    }
  }

  return "Unknown error code";
}

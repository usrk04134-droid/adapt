#pragma once

#include <pnd/common/pniobase.h>
#include <pnd/common/pniousrx.h>
#include <pnd/common/servusrx.h>

#include <boost/outcome.hpp>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "common/clock_functions.h"
#include "controller/controller.h"
#include "controller/controller_data.h"
#include "controller/pn_driver/pn_driver_data.h"

namespace controller::pn_driver {

struct Configuration {
  std::filesystem::path rema_path;
  std::vector<uint8_t> mac_address;
  std::string interface;
};

class PnDriver final : public controller::Controller {
 public:
  /**
   * Constructs a new PnDriver instance with provided configuration.
   * @param configuration
   */
  explicit PnDriver(Configuration configuration, clock_functions::SteadyClockNowFunc steady_clock_now_func);

  PnDriver(PnDriver&)                     = delete;
  auto operator=(PnDriver&) -> PnDriver&  = delete;
  PnDriver(PnDriver&&)                    = delete;
  auto operator=(PnDriver&&) -> PnDriver& = delete;

  ~PnDriver() override = default;
  auto Connect() -> boost::outcome_v2::result<bool> override;
  void Disconnect() override;
  auto IsConnected() -> bool override;

  void WriteAdaptioOutput(controller::AdaptioOutput data) override;
  void WriteSlideCrossXOutput(controller::AxisOutput data) override;
  void WriteSlideCrossYOutput(controller::AxisOutput data) override;
  void WriteWeldAxisOutput(controller::AxisOutput data) override;
  void WritePowerSource1Output(controller::PowerSourceOutput data) override;
  void WritePowerSource2Output(controller::PowerSourceOutput data) override;
  void WriteTrackingOutput(controller::TrackOutput data) override;

 private:
  Configuration configuration_               = {};
  PNIO_UINT32 application_handle_            = 0xFFFF;
  PNIO_UINT32 interface_handle_              = 0xFFFF;
  std::optional<std::string> station_name_   = std::nullopt;
  std::optional<PNIO_UINT8*> ip_address_     = std::nullopt;
  InputImage input_image_                    = {};
  OutputImage output_image_                  = {};
  Subslot input                              = {};
  Subslot output                             = {};
  std::mutex mutex_                          = {};
  PNIO_DEBUG_SETTINGS_TYPE debug_settings_   = {};
  bool application_relationship_established_ = false;
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;

  auto RetrieveInputs() -> boost::outcome_v2::result<bool> override;
  auto WriteOutputs() -> boost::outcome_v2::result<bool> override;
  auto ReadInputs() -> PNIO_UINT32;
  auto ReadInput(const Subslot& slot, PNIO_UINT8* buffer) -> PNIO_UINT32;
  auto WriteOutput(const Subslot& slot, PNIO_UINT32 buffer_length, PNIO_UINT8* buffer) -> PNIO_UINT32;

  void Init();
  auto InitDevice() -> PNIO_UINT32;
  auto StartDevice() -> PNIO_UINT32;
  auto OpenDevice() -> PNIO_UINT32;
  auto OpenInterface() -> PNIO_UINT32;
  auto RegisterCallback(PNIO_CBE_TYPE type, PNIO_CBF callback) const -> PNIO_UINT32;
  auto RegisterInterfaceCallback(PNIO_CBE_TYPE type, PNIO_CBF callback) const -> PNIO_UINT32;
  void RegisterCallbacks();
  auto Activate() -> PNIO_UINT32;
  auto Deactivate() -> PNIO_UINT32;
  auto CloseDevice() -> PNIO_UINT32;
  auto CloseInterface() -> PNIO_UINT32;
  auto Shutdown() -> PNIO_UINT32;
  auto UndoInit() -> PNIO_UINT32;
  void Cleanup();

  auto GetMacAddress(const std::string& interface_name) -> std::vector<uint8_t>;
  void ListSubmodules();

  void Connected(PNIO_CBE_PRM*);
  void Disconnected(PNIO_CBE_PRM*);
  void Ownership(PNIO_CBE_PRM*);
  void PrmEnd(PNIO_CBE_PRM*);
  void InData(PNIO_CBE_PRM*);
  void Diagnostics(PNIO_CBE_PRM*);
  void ReadRecord(PNIO_CBE_PRM*);
  void WriteRecord(PNIO_CBE_PRM*);
  void Ethernet(PNIO_CBE_PRM*);

  const static std::string GetPnDriverErrorAsString(PNIO_UINT32 error_code);
  const static std::string GetPnDriverReasonCodeAsString(PNIO_UINT32 reason_code);
};

/**
 * Yoinked from cm_usr.h in pndriver
 */
enum CmArReasonEnum {

  CM_AR_REASON_NONE = 0, /**< 0x00: no error */
  /***/
  CM_AR_REASON_1         = 1,  /**< 0x01: sequence numbers do not match (no longer used in versions >= V3.9)*/
  CM_AR_REASON_2         = 2,  /**< 0x02: alarm instance closed (no longer used in versions >= V3.6) */
  CM_AR_REASON_MEM       = 3,  /**< 0x03: out of mem */
  CM_AR_REASON_FRAME     = 4,  /**< 0x04: add provider or consumer failed */
  CM_AR_REASON_MISS      = 5,  /**< 0x05: miss (consumer) */
  CM_AR_REASON_TIMER     = 6,  /**< 0x06: cmi timeout */
  CM_AR_REASON_ALARM     = 7,  /**< 0x07: alarm-open failed */
  CM_AR_REASON_ALSND     = 8,  /**< 0x08: alarm-send.cnf(-) */
  CM_AR_REASON_ALACK     = 9,  /**< 0x09: alarm-ack-send.cnf(-) */
  CM_AR_REASON_ALLEN     = 10, /**< 0x0A: alarm-data too long */
  CM_AR_REASON_ASRT      = 11, /**< 0x0B: alarm.ind(err) */
  CM_AR_REASON_RPC       = 12, /**< 0x0C: rpc-client call.cnf(-) */
  CM_AR_REASON_ABORT     = 13, /**< 0x0D: ar-abort.req */
  CM_AR_REASON_RERUN     = 14, /**< 0x0E: rerun aborts existing */
  CM_AR_REASON_REL       = 15, /**< 0x0F: got release.ind */
  CM_AR_REASON_PAS       = 16, /**< 0x10: device passivated */
  CM_AR_REASON_RMV       = 17, /**< 0x11: device / AR removed */
  CM_AR_REASON_PROT      = 18, /**< 0x12: protocol violation */
  CM_AR_REASON_NARE      = 19, /**< 0x13: NARE error */
  CM_AR_REASON_BIND      = 20, /**< 0x14: RPC-Bind error */
  CM_AR_REASON_CONNECT   = 21, /**< 0x15: RPC-Connect error */
  CM_AR_REASON_READ      = 22, /**< 0x16: RPC-Read error */
  CM_AR_REASON_WRITE     = 23, /**< 0x17: RPC-Write error */
  CM_AR_REASON_CONTROL   = 24, /**< 0x18: RPC-Control error */
  CM_AR_REASON_25        = 25, /**< 0x19: reserved (formerly: pull or plug in forbidden window) */
  CM_AR_REASON_26        = 26, /**< 0x1A: reserved (formerly: AP removed) */
  CM_AR_REASON_LNK_DOWN  = 27, /**< 0x1B: link "down", for local purpose only */
  CM_AR_REASON_28        = 28, /**< 0x1C: reserved (formerly: could not register multicast-mac) */
  CM_AR_REASON_SYNC      = 29, /**< 0x1D: not synchronized (cannot start companion-ar) */
  CM_AR_REASON_30        = 30, /**< 0x1E: reserved (formerly: wrong topology (cannot start companion-ar)) */
  CM_AR_REASON_DCP_NAME  = 31, /**< 0x1F: dcp, station-name changed */
  CM_AR_REASON_DCP_RESET = 32, /**< 0x20: dcp, reset to factory or factory reset */
  CM_AR_REASON_33        = 33, /**< 0x21: reserved (formerly: cannot start companion-ar) */
  CM_AR_REASON_IRDATA    = 34, /**< 0x22: no irdata record yet */
  CM_AR_REASON_PDEV      = 35, /**< 0x23: ownership of physical device */
  CM_AR_REASON_LNK_MODE  = 36, /**< 0x24: link mode not full duplex */
  CM_AR_REASON_IPSUITE =
      37, /**< 0x25: IP-Suite [of the IOC] changed by means of DCP_set(IPParameter) or local engineering */
  CM_AR_REASON_RDHT      = 38, /**< 0x26: IOCARSR RDHT expired */
  CM_AR_REASON_PDEV_PRM  = 39, /**< 0x27: IOCARSR PDev, parametrization impossible (AP01238541) */
  CM_AR_REASON_ARDY      = 40, /**< 0x28: Remote application timeout expired */
  CM_AR_REASON_41_UNUSED = 41, /**< 0x29: IOCARSR Redundant interface lost or access to the peripherals impossible */
  CM_AR_REASON_MTOT      = 42, /**< 0x2A: IOCARSR MTOT expired */
  CM_AR_REASON_43_UNUSED = 43, /**< 0x2B: IOCARSR AR protocol violation */
  CM_AR_REASON_COC       = 44, /**< 0x2C: Pdev, plug port without CombinedObjectContainer */
  CM_AR_REASON_NME       = 45, /**< 0x2D: NME, no or wrong configuration */

  /** @cond INTERNAL */
  CM_AR_REASON_R1_BRIDGE = 252, /**< internally used only, not visible in LogBookData */
  CM_AR_REASON_R1_CONN   = 253, /**< internally used only, not visible in LogBookData */
  CM_AR_REASON_R1_DISC   = 254, /**< internally used only, not visible in LogBookData */
  /** @endcond */

  CM_AR_REASON_MAX = 255 /**< !*/
};

}  // namespace controller::pn_driver

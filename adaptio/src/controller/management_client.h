#pragma once

#include <cstdint>

#include "common/messages/management.h"
#include "common/zevs/zevs_socket.h"
#include "controller/controller_data.h"

namespace controller {

struct TrackingControlData {
  float horizontal_offset{};
  float vertical_offset{};
  uint32_t joint_tracking_mode{};

  auto operator==(const TrackingControlData& other) const -> bool;
  auto operator!=(const TrackingControlData& other) const -> bool;
};

class ManagementClientObserver {
 public:
  virtual ~ManagementClientObserver() = default;

  virtual void AdaptioOutputUpdate(const AdaptioOutput&) = 0;
  virtual void TrackOutputUpdate(const TrackOutput&)     = 0;
};

class ManagementClient {
 public:
  ManagementClient(zevs::Socket*, ManagementClientObserver*);

  enum class Sequence : uint32_t {
    NO_SEQUENCE = 0,
    TRACKING    = 1,
    ABP         = 2,
    ABP_CAP     = 4,
  };
  static auto SequenceToString(Sequence) -> std::string;

  struct InputData {
    bool start    = false;
    bool stop     = false;
    bool shutdown = false;
    Sequence sequence_type{};
    auto ToString() const -> std::string;
    auto operator==(const InputData&) const -> bool;
  };

  void Init();
  void Update();
  void OnAdaptioInput(const InputData&);
  void OnTrackingInput(const TrackingControlData&);
  void HeartbeatLost();
  void Disconnected(uint32_t);

 private:
  zevs::Socket* socket_;
  ManagementClientObserver* observer_;

  enum class InterfaceState {
    IDLE,
    TRACKING,
    ABP,
    ABP_CAP,
    ERROR,
  } state_{};
  static auto InterfaceStateToString(InterfaceState) -> std::string;

  enum class ReadyState {
    NOT_READY,
    NOT_READY_AUTO_CAL_MOVE,
    TRACKING_READY,
    ABP_READY,
    ABP_CAP_READY,
  } ready_state_{};
  static auto ReadyStateToString(ReadyState) -> std::string;

  bool handover_to_manual_ = false;
  bool ready_for_auto_cap_ = false;
  TrackingControlData track_input_data_;

  void LogStateChange(const std::string& event, const InputData& data, InterfaceState new_state) const;
  void SetState(InterfaceState new_state, const std::string& event, const InputData& data);
  void OnReadyState(common::msg::management::ReadyState);
  void OnOnNotifyHandoverToManual(common::msg::management::NotifyHandoverToManual);
  void OnTrackingStoppedGrooveDataTimeout(common::msg::management::TrackingStoppedGrooveDataTimeout);
  void OnScannerError(common::msg::management::ScannerError);
  void OnGracefulStop(common::msg::management::GracefulStop);

  void SendTrackingStart();
  void SendTrackingUpdate();
  void SendABPStart();
  void SendABPStop();
  void SendABPCapStart();
  void SendABPCapStop();
  void SendStop();
};

}  // namespace controller

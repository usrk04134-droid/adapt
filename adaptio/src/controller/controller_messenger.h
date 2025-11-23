#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "axis_output_plc_adapter.h"
#include "common/clock_functions.h"
#include "common/zevs/zevs_core.h"
#include "common/zevs/zevs_socket.h"
#include "controller/controller.h"
#include "controller/controller_data.h"
#include "controller/weld_system_server_impl.h"
#include "kinematics_server.h"
#include "management_client.h"
#include "slide_position_buffer.h"
#include "weld_axis_buffer.h"
#include "weld_axis_filter.h"

namespace controller {

enum class AxisId : uint32_t {
  INVALID,
  SLIDE_CROSS_HORIZONTAL,
  SLIDE_CROSS_VERTICAL,
  WELD_AXIS,
  SIZE,
};

namespace input {

struct ExternalControllerInput {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000010 };
  bool set_external_command_mode;
};

}  // namespace input

class ControllerMessenger : public ControllerDataCallbacks,
                            public ManagementClientObserver,
                            public WeldSystemServerObserver {
 public:
  explicit ControllerMessenger(ControllerPtr controller, uint32_t cycle_time,
                               clock_functions::SystemClockNowFunc system_clock_now_func,
                               clock_functions::SteadyClockNowFunc steady_clock_now_func,
                               std::string endpoint_base_url = "controller_messages");

  ControllerMessenger(ControllerMessenger&)                     = delete;
  auto operator=(ControllerMessenger&) -> ControllerMessenger&  = delete;
  ControllerMessenger(ControllerMessenger&&)                    = delete;
  auto operator=(ControllerMessenger&&) -> ControllerMessenger& = delete;

  ~ControllerMessenger() override;

  void OnAdaptioInputUpdate(AdaptioInput data) override;
  void OnSlideCrossXInputUpdate(AxisInput data) override;
  void OnSlideCrossYInputUpdate(AxisInput data) override;
  void OnWeldAxisInputUpdate(AxisInput data) override;
  void OnPowerSource1InputUpdate(PowerSourceInput data) override;
  void OnPowerSource2InputUpdate(PowerSourceInput data) override;
  void OnTrackingInputUpdate(TrackInput data) override;
  void OnHeartbeatLost() override;
  void OnDisconnected(uint32_t reason_code) override;
  auto ValidateHeartbeat() -> bool override;

  void SuperviseHeartbeat();
  void ThreadEntry(const std::string& name);
  void StartThread(const std::string& event_loop_name);
  void JoinThread();

  // ManagementClientObserver
  void AdaptioOutputUpdate(const AdaptioOutput& data) override;
  void TrackOutputUpdate(const TrackOutput& data) override;
  void OnPowerSourceOutput(uint32_t index, PowerSourceOutput const& data) override;

 protected:
  void CheckAdaptioHeartbeat();

  std::chrono::time_point<std::chrono::steady_clock> heartbeat_last_changed_ = {};
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;

 private:
  ControllerPtr controller_;
  uint32_t cycle_time_               = 100;
  uint8_t filter_median_window_size_ = 5;
  std::thread thread_;
  zevs::EventLoopPtr event_loop_;
  std::string management_endpoint_url_;
  std::string kinematics_endpoint_url_;
  std::string weld_system_endpoint_url_;
  zevs::SocketPtr management_socket_;
  zevs::SocketPtr kinematics_socket_;
  zevs::SocketPtr weld_system_socket_;
  zevs::TimerPtr timer_;
  std::unique_ptr<ManagementClient> management_client_;
  std::unique_ptr<KinematicsServer> kinematics_server_;
  std::unique_ptr<SlidePositionBuffer> horizontal_position_buffer_;
  std::unique_ptr<SlidePositionBuffer> vertical_position_buffer_;
  std::unique_ptr<WeldAxisBuffer> weld_axis_buffer_;
  std::unique_ptr<WeldSystemServerImpl> weld_system_server_;
  std::unique_ptr<WeldAxisFilter> weld_axis_median_filter_;
  clock_functions::SystemClockNowFunc system_clock_now_func_;

  bool supervise_heartbeat_ = false;

  auto Connect() -> boost::outcome_v2::result<bool>;
  void StartTimer();
  void OnTimeout();

  void OnAdaptioOutputUpdate(const AdaptioOutput& data);
  void OnKinematicAxisOutputUpdate(const AxisOutput& data);
  void OnTrackOutputUpdate(const TrackOutput& data);

  std::unique_ptr<AxisOutputPlcAdapter> axis_output_plc_adapter_;
  ManagementClient::InputData input_data_cache_;
  uint32_t heartbeat_{};  // should be different from previous_heartbeat_
  uint32_t previous_heartbeat_{std::numeric_limits<uint32_t>::max()};
  AxisInput slide_cross_x_input_data_cache_;
  AxisInput slide_cross_y_input_data_cache_;
  AxisInput weld_axis_input_data_cache_;
  TrackingControlData tracking_control_cache_;
};

using ControllerMessengerPtr = std::unique_ptr<ControllerMessenger>;

}  // namespace controller

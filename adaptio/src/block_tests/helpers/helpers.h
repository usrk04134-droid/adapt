#pragma once

#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>
#include <sys/types.h>

#include <chrono>
#include <memory>

#include "application.h"
#include "block_tests/mocks/config_manager_mock.h"
#include "block_tests/scanner/on_demand_file_image_provider.h"
#include "common/clock_functions.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_test_support.h"
#include "controller/controller_data.h"
#include "controller/controller_messenger.h"
#include "scanner/scanner_application.h"
#include "scanner/test/mock/stub_core_factory.h"

class ApplicationWrapper {
 public:
  explicit ApplicationWrapper(SQLite::Database *database, configuration::ConfigManagerMock *config_manager,
                              clock_functions::SystemClockNowFunc system_clock_now_func,
                              clock_functions::SteadyClockNowFunc steady_clock_now_func);
  void Start();
  void Exit();
  auto InShutdown() const -> bool;
  auto Registry() -> prometheus::Registry *;
  auto GetWeldControlConfig() -> weld_control::Configuration;
  auto GetConfigManagerMock() -> configuration::ConfigManagerMock *;

 private:
  configuration::ConfigManagerMock *configuration_;
  std::unique_ptr<Application> application_;
  clock_functions::SystemClockNowFunc system_clock_now_func_;
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;
  std::shared_ptr<prometheus::Registry> registry_;

  // Parameters for Application construction
  SQLite::Database *database_;
  std::filesystem::path events_path_;
  std::filesystem::path logs_path_;
};

class ClockNowFuncWrapper {
 public:
  ClockNowFuncWrapper();
  auto GetSystemClock() -> std::chrono::time_point<std::chrono::system_clock>;
  auto GetSteadyClock() -> std::chrono::time_point<std::chrono::steady_clock>;
  auto StepSystemClock(std::chrono::milliseconds ms) -> void { system_clock_latest_ += ms; }
  auto StepSteadyClock(std::chrono::milliseconds ms) -> void { steady_clock_latest_ += ms; }

 private:
  std::chrono::system_clock::duration system_clock_latest_;
  std::chrono::steady_clock::duration steady_clock_latest_;
};

class ScannerDataWrapper {
 public:
  ScannerDataWrapper();
  auto Get() const -> common::msg::scanner::SliceData;
  auto GetWithConfidence(common::msg::scanner::SliceConfidence confidence) const -> common::msg::scanner::SliceData;
  auto ShiftHorizontal(double value) -> ScannerDataWrapper &;
  auto FillUp(double value) -> ScannerDataWrapper &;

 private:
  common::msg::scanner::SliceData data_;
};

class TimerWrapper {
 public:
  explicit TimerWrapper(clock_functions::SteadyClockNowFunc steady_clock_now_func);
  void RequestTimer(uint32_t duration_ms, bool periodic, const std::string &task_name);
  void CancelTimer(const std::string &task_name);
  // Dispatch expired timers
  void DispatchAllExpired();

  using DispatchHandler = std::function<void(const std::string &task_name)>;
  void SetDispatchHandler(DispatchHandler dispatch_handler);

 private:
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;
  struct TimerTask {
    std::chrono::milliseconds duration_ms;
    bool periodic;
    std::string name;
    std::chrono::steady_clock::duration request_time;
    auto operator<(const TimerTask &other) const -> bool;
  };
  std::set<TimerTask> timer_tasks_;
  DispatchHandler dispatch_handler_{nullptr};
};

class TestFixture {
 public:
  explicit TestFixture();
  auto DescribeQueue() const -> std::string;

  auto Factory() -> zevs::MocketFactory *;
  auto WebHmiIn() -> zevs::Mocket *;
  auto WebHmiOut() -> zevs::Mocket *;
  auto Management() -> zevs::Mocket *;
  auto Kinematics() -> zevs::Mocket *;
  auto Scanner() -> zevs::Mocket *;
  auto WeldSystem() -> zevs::Mocket *;
  auto Timer() -> zevs::MocketTimer *;

  auto ScannerData() -> ScannerDataWrapper *;
  auto Sut() -> ApplicationWrapper *;
  auto GetDatabase() -> SQLite::Database *;
  auto GetConfigManagerMock() -> configuration::ConfigManagerMock *;

  void SetupTimerWrapper();
  void SetupDefaultConfiguration();
  auto GetClockNowFuncWrapper() -> ClockNowFuncWrapper *;
  auto GetTimerWrapper() -> TimerWrapper *;
  void StartApplication();
  void StopApplication();
  void SetupMockets();

 private:
  auto StartedOK() const -> bool;
  auto MocketsFound() const -> bool;
  void SetDefaultCalibration();

  std::unique_ptr<ApplicationWrapper> application_sut_;
  ScannerDataWrapper scanner_data_;

  zevs::MocketFactory factory_;
  zevs::MocketPtr web_hmi_in_mocket_;
  zevs::MocketPtr web_hmi_out_mocket_;
  zevs::MocketPtr management_mocket_;
  zevs::MocketPtr kinematics_mocket_;
  zevs::MocketPtr scanner_mocket_;
  zevs::MocketPtr weld_system_mocket_;
  zevs::MocketTimerPtr timer_mocket_;

  SQLite::Database database_;
  std::shared_ptr<ClockNowFuncWrapper> clock_now_func_wrapper_;
  std::shared_ptr<TimerWrapper> timer_wrapper_;
  std::unique_ptr<configuration::ConfigManagerMock> config_manager_mock_;
};

struct MockPlc : public controller::Controller {
  auto Connect() -> boost::outcome_v2::result<bool> override {
    is_connected = true;
    return true;
  }

  void Disconnect() override { is_connected = false; }
  auto IsConnected() -> bool override { return is_connected; }
  void WriteAdaptioOutput(controller::AdaptioOutput data) override { adaptio_output = data; }
  void WriteTrackingOutput(controller::TrackOutput data) override { track_output = data; }
  void WriteSlideCrossXOutput(controller::AxisOutput data) override { axis_x_output = data; }
  void WriteSlideCrossYOutput(controller::AxisOutput data) override { axis_y_output = data; }

  controller::AdaptioOutput adaptio_output;
  controller::TrackOutput track_output;
  controller::AxisOutput axis_x_output;
  controller::AxisOutput axis_y_output;
  bool is_connected = false;
};

class ControllerFixture {
 public:
  explicit ControllerFixture(clock_functions::SystemClockNowFunc system_clock_now_func);

  void Start();
  void Stop();

  auto Management() -> zevs::Mocket *;
  auto Kinematics() -> zevs::Mocket *;
  auto WeldSystem() -> zevs::Mocket *;
  auto Timer() -> zevs::MocketTimer *;
  auto Mock() -> MockPlc *;
  auto Sut() -> controller::ControllerMessenger *;

 private:
  clock_functions::SystemClockNowFunc system_clock_now_func_;
  MockPlc *mock_plc_;
  controller::ControllerMessengerPtr controller_messenger_;
  std::string base_url_;

  zevs::MocketPtr management_mocket_;
  zevs::MocketPtr kinematics_mocket_;
  zevs::MocketPtr weld_system_mocket_;
  zevs::MocketTimerPtr timer_mocket_;
};

class ScannerFixture {
 public:
  explicit ScannerFixture(const scanner::ScannerCalibrationData &cal_data,
                          clock_functions::SystemClockNowFunc system_clock_now_func,
                          const std::filesystem::path &base_dir, const std::string &serial_number,
                          std::string endpoint_base_url                  = "adaptio",
                          std::optional<std::filesystem::path> logs_path = std::nullopt);

  void Start();
  void Stop();

  void ProcessImage(const std::filesystem::path &file);
  void RepeatToFillMedianBuffer(const std::filesystem::path &file);
  void TriggerScannerData();

  auto Scanner() -> zevs::Mocket *;
  auto Timer() -> zevs::MocketTimer *;
  auto Sut() -> scanner::ScannerApplication *;

  using EventObserverT = std::function<void()>;
  void SetEventObserver(EventObserverT observer);

 private:
  scanner::ScannerCalibrationData cal_data_;
  clock_functions::SystemClockNowFunc sys_now_;
  std::unique_ptr<OnDemandFileImageProvider> provider_;
  std::string base_url_;
  std::optional<std::filesystem::path> logs_path_;

  std::shared_ptr<prometheus::Registry> registry_;

  scanner::image_logger::ImageLoggerPtr image_logger_;
  scanner::ScannerApplicationPtr app_;

  zevs::MocketPtr scanner_mocket_;
  zevs::MocketTimerPtr timer_mocket_;
  EventObserverT event_observer_;
};

class MultiFixture {
 public:
  MultiFixture();
  void PlcDataUpdate();
  auto Main() -> TestFixture &;
  auto Ctrl() -> ControllerFixture &;
  void SetupScanner(const std::string &serial_number);
  auto Scan() -> ScannerFixture &;  // valid only after SetupScanner()
  void SetIdleCallback(std::function<void()> cb);

 private:
  void ForwardAllPending();

  TestFixture app_;
  ControllerFixture ctrl_;
  std::unique_ptr<ScannerFixture> scan_;
  std::function<void()> idle_cb_;
};

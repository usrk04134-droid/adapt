#include "helpers.h"

#include <doctest/doctest.h>
#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <array>
#include <algorithm>
#include <boost/outcome.hpp>
#include <boost/outcome/result.hpp>
#include <boost/outcome/success_failure.hpp>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "application.h"
#include "calibration/calibration_configuration.h"
#include "calibration/calibration_types.h"
#include "common/clock_functions.h"
#include "common/logging/application_log.h"
#include "common/messages/scanner.h"
#include "common/tolerances/tolerances_configuration.h"
#include "common/zevs/zevs_test_support.h"
#include "controller/controller_configuration.h"
#include "image_logging/image_logging_configuration.h"
#include "joint_geometry/joint_geometry.h"
#include "mocks/config_manager_mock.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/scanner_configuration.h"
#include "test_utils/testlog.h"
#include "weld_control/weld_control_types.h"

const uint32_t TIMER_INSTANCE       = 1;
const uint32_t TIMER_INSTANCE_2     = 2;
const std::string ENDPOINT_BASE_URL = "adaptio";
const uint32_t PLC_CYCLE_TIME_MS    = 100;

namespace {
auto MakeEndpoint(const std::string& suffix) -> std::string {
  return fmt::format("inproc://{}/{}", ENDPOINT_BASE_URL, suffix);
}
}  // namespace

ApplicationWrapper::ApplicationWrapper(SQLite::Database* database, configuration::ConfigManagerMock* config_manager,
                                       clock_functions::SystemClockNowFunc system_clock_now_func,
                                       clock_functions::SteadyClockNowFunc steady_clock_now_func)

    : configuration_(config_manager),
      system_clock_now_func_(system_clock_now_func),
      steady_clock_now_func_(steady_clock_now_func),
      database_(database),
      events_path_("assets/events/events.yaml"),
      logs_path_("/var/log/adaptio/") {
  // ConfigManagerMock is pre-configured by TestFixture, no need to Init here

  registry_ = std::make_shared<prometheus::Registry>();
}

void ApplicationWrapper::Start() {
  // Create the Application instance
  application_ = std::make_unique<Application>(configuration_, events_path_, database_, logs_path_,
                                               system_clock_now_func_, steady_clock_now_func_, registry_.get(), -1);
  // Run the application
  application_->Run("Application", ENDPOINT_BASE_URL);
}

void ApplicationWrapper::Exit() {
  if (application_) {
    application_->Exit();
    // Destroy the Application instance (calls destructor)
    application_.reset();
  }
}

auto ApplicationWrapper::InShutdown() const -> bool { return application_ ? application_->InShutdown() : true; }

auto ApplicationWrapper::Registry() -> prometheus::Registry* { return registry_.get(); }

auto ApplicationWrapper::GetWeldControlConfig() -> weld_control::Configuration {
  return configuration_->GetWeldControlConfiguration();
}

auto ApplicationWrapper::GetConfigManagerMock() -> configuration::ConfigManagerMock* { return configuration_; }

ClockNowFuncWrapper::ClockNowFuncWrapper()
    : system_clock_latest_(std::chrono::system_clock::now().time_since_epoch()),
      steady_clock_latest_(std::chrono::steady_clock::now().time_since_epoch()) {};
auto ClockNowFuncWrapper::GetSystemClock() -> std::chrono::time_point<std::chrono::system_clock> {
  return std::chrono::time_point<std::chrono::system_clock>(system_clock_latest_);
};
auto ClockNowFuncWrapper::GetSteadyClock() -> std::chrono::time_point<std::chrono::steady_clock> {
  return std::chrono::time_point<std::chrono::steady_clock>(steady_clock_latest_);
};

TimerWrapper::TimerWrapper(clock_functions::SteadyClockNowFunc steady_clock_now_func)
    : steady_clock_now_func_(steady_clock_now_func) {};

void TimerWrapper::RequestTimer(uint32_t duration_ms, bool periodic, const std::string& task_name) {
  timer_tasks_.insert(
      {std::chrono::milliseconds(duration_ms), periodic, task_name, steady_clock_now_func_().time_since_epoch()});
}

void TimerWrapper::CancelTimer(const std::string& task_name) {
  std::erase_if(timer_tasks_, [task_name](const auto& task) { return task.name == task_name; });
}

void TimerWrapper::DispatchAllExpired() {
  std::set<std::string> expired_tasks;
  if (!dispatch_handler_) {
    return;  // Not yet configured
  }
  // Finds expired timeouts and indicate to be removed if non periodic
  auto remove = [this, &expired_tasks](TimerTask task) {
    if (steady_clock_now_func_ != nullptr &&
        (steady_clock_now_func_().time_since_epoch() > (task.request_time + task.duration_ms))) {
      expired_tasks.insert({task.name});
      if (task.periodic) {
        task.request_time = steady_clock_now_func_().time_since_epoch();
      }
      return !task.periodic;
    }
    return false;
  };
  std::erase_if(timer_tasks_, remove);
  // Dispatch expired tasks
  for (const auto& task : expired_tasks) {
    dispatch_handler_(task);
  }
}

void TimerWrapper::SetDispatchHandler(DispatchHandler dispatch_handler) {
  dispatch_handler_ = std::move(dispatch_handler);
}

inline auto TimerWrapper::TimerTask::operator<(const TimerTask& other) const -> bool {
  return (name < other.name) || ((name == other.name));
}

void TestFixture::SetupTimerWrapper() {
  timer_mocket_->SetRequestObserver([this](uint32_t duration_ms, bool periodic, const std::string& task_name) {
    GetTimerWrapper()->RequestTimer(duration_ms, periodic, task_name);
  });
  timer_mocket_->SetCancelObserver([this](const std::string& task_name) { GetTimerWrapper()->CancelTimer(task_name); });
  timer_wrapper_->SetDispatchHandler([this](const std::string& task_name) { Timer()->Dispatch(task_name); });
}

void TestFixture::SetupDefaultConfiguration() {
  auto* mock = config_manager_mock_.get();

  // Initialize the mock
  if (mock->Init({}, {}, {}) != boost::outcome_v2::success()) {
    LOG_ERROR("Init of configuration mock failed");
    return;
  }

  //  NOLINTBEGIN(*-magic-numbers)

  weld_control::Configuration weld_config{};
  weld_config.scanner_groove_geometry_update.tolerance.upper_width = 2.0;
  weld_config.scanner_groove_geometry_update.tolerance.wall_angle  = 0.13962634;
  weld_config.supervision.arcing_lost_grace                        = std::chrono::milliseconds{3000};
  weld_config.scanner_input_interval                               = std::chrono::milliseconds{50};
  weld_config.adaptivity.gaussian_filter.kernel_size               = 301;
  weld_config.adaptivity.gaussian_filter.sigma                     = 50.0;
  weld_config.handover_grace                                       = std::chrono::seconds{25};
  weld_config.scanner_low_confidence_grace                         = std::chrono::seconds{5};
  weld_config.scanner_no_confidence_grace                          = std::chrono::seconds{5};
  mock->SetWeldControlConfiguration(weld_config);

  controller::ControllerConfigurationData controller_config{};
  controller_config.type = controller::ControllerType::SIMULATION;
  mock->SetControllerConfig(controller_config);

  scanner::ScannerConfigurationData scanner_config{};
  scanner_config.gray_minimum_top    = 32;
  scanner_config.gray_minimum_wall   = 16;
  scanner_config.gray_minimum_bottom = 32;
  mock->SetScannerConfig(scanner_config);

  scanner::image_provider::ImageProviderConfigData image_provider_config{};
  image_provider_config.image_provider      = scanner::image_provider::ImageProviderType::SIMULATION;
  image_provider_config.sim_config.realtime = false;
  mock->SetImageProviderConfig(image_provider_config);

  tolerances::Configuration tolerances_config{};
  tolerances_config.joint_geometry.upper_width   = 10.0;
  tolerances_config.joint_geometry.surface_angle = 0.174532925;
  tolerances_config.joint_geometry.wall_angle    = 0.13962634;
  mock->SetTolerancesConfiguration(tolerances_config);

  calibration::Configuration calibration_config{};
  calibration_config.grid_config.margin_top                 = 10.0;
  calibration_config.grid_config.margin_x                   = 0.0;
  calibration_config.grid_config.margin_z                   = 30.0;
  calibration_config.grid_config.margin_c                   = 5.0;
  calibration_config.grid_config.target_nr_gridpoints       = 20;
  calibration_config.runner_config.slide_velocity           = 5.0;
  calibration_config.runner_config.stabilization_time       = 2.0;
  calibration_config.runner_config.near_target_delta        = 1.0;
  calibration_config.runner_config.max_time_per_observation = 30.0;
  mock->SetCalibrationConfiguration(calibration_config);

  image_logging::Configuration image_logging_config{};
  image_logging_config.mode = image_logging::Mode::OFF;
  mock->SetImageLoggingConfiguration(image_logging_config);

  joint_geometry::JointGeometry joint_geometry{};
  joint_geometry.upper_joint_width_mm        = 50.0;
  joint_geometry.groove_depth_mm             = 28.0;
  joint_geometry.left_joint_angle_rad        = 0.1396;
  joint_geometry.right_joint_angle_rad       = 0.1396;
  joint_geometry.left_max_surface_angle_rad  = 0.34906585;
  joint_geometry.right_max_surface_angle_rad = 0.34906585;
  mock->SetCalibrationFixtureJointGeometry(joint_geometry);

  calibration::WeldObjectCalibration woc_calibration{};
  woc_calibration.y            = -1.727200000000000e+03;
  woc_calibration.z            = -2.445100000000000e+03;
  woc_calibration.x_adjustment = 1.600000000000000e+00;
  mock->SetCircWeldObjectCalib(woc_calibration);

  calibration::LaserTorchCalibration laser_torch_calibration{};
  laser_torch_calibration.x     = 0.000000000000000e+00;
  laser_torch_calibration.y     = 3.500000000000000e+01;
  laser_torch_calibration.z     = -2.500000000000000e+01;
  laser_torch_calibration.angle = 2.000000000000000e-01;

  mock->SetLaserTorchCalib(laser_torch_calibration);

  //  NOLINTEND(*-magic-numbers)
}

void TestFixture::SetupMockets() {
  // LOG_DEBUG("factory contains: {}", factory.Describe());
  // bind_endpoints: {inproc://adaptio/WebHmiIn, inproc://adaptio/WebHmiOut}
  // connect_endpoints: {inproc://adaptio/control, inproc://adaptio/kinematics,inproc://adaptio/scanner}

  web_hmi_in_mocket_  = factory_.GetMocket(zevs::Endpoint::BIND, "tcp://0.0.0.0:5555");
  web_hmi_out_mocket_ = factory_.GetMocket(zevs::Endpoint::BIND, "tcp://0.0.0.0:5556");
  management_mocket_  = factory_.GetMocket(zevs::Endpoint::CONNECT, MakeEndpoint("management"));
  kinematics_mocket_  = factory_.GetMocket(zevs::Endpoint::CONNECT, MakeEndpoint("kinematics"));
  scanner_mocket_     = factory_.GetMocket(zevs::Endpoint::CONNECT, MakeEndpoint("scanner"));
  weld_system_mocket_ = factory_.GetMocket(zevs::Endpoint::CONNECT, MakeEndpoint("weld-system"));
  timer_mocket_       = factory_.GetMocketTimer(TIMER_INSTANCE);
}

auto TestFixture::MocketsFound() const -> bool {
  return web_hmi_in_mocket_ && web_hmi_out_mocket_ && management_mocket_ && kinematics_mocket_ && scanner_mocket_ &&
         weld_system_mocket_;
}

auto TestFixture::StartedOK() const -> bool { return MocketsFound(); }

auto TestFixture::GetClockNowFuncWrapper() -> ClockNowFuncWrapper* { return clock_now_func_wrapper_.get(); }
auto TestFixture::GetTimerWrapper() -> TimerWrapper* { return timer_wrapper_.get(); }

void TestFixture::StartApplication() {
  TESTLOG_NOHDR("  --== fixture StartApplication ==--")
  application_sut_->Start();
  SetupMockets();
  assert(StartedOK());
  TESTLOG_NOHDR("  --== fixture StartApplication done ==--")
}

void TestFixture::StopApplication() { application_sut_->Exit(); }

TestFixture::TestFixture()
    :  // NOLINTNEXTLINE(hicpp-signed-bitwise)
      database_(SQLite::Database(":memory:", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE)) {
  database_.exec("PRAGMA foreign_keys=on");

  clock_now_func_wrapper_            = std::make_shared<ClockNowFuncWrapper>();
  auto clock_system_now_func_wrapper = [this]() -> std::chrono::time_point<std::chrono::system_clock> {
    return clock_now_func_wrapper_->GetSystemClock();
  };
  auto clock_steady_now_func_wrapper = [this]() -> std::chrono::time_point<std::chrono::steady_clock> {
    return clock_now_func_wrapper_->GetSteadyClock();
  };
  timer_wrapper_       = std::make_shared<TimerWrapper>(clock_steady_now_func_wrapper);
  config_manager_mock_ = std::make_unique<configuration::ConfigManagerMock>();

  // Set up default configuration values
  SetupDefaultConfiguration();

  application_sut_ = std::make_unique<ApplicationWrapper>(&database_, config_manager_mock_.get(),
                                                          clock_system_now_func_wrapper, clock_steady_now_func_wrapper);

  TESTLOG_NOHDR("  --== fixture setup done ==--")
}

auto TestFixture::Factory() -> zevs::MocketFactory* { return &factory_; }
auto TestFixture::WebHmiIn() -> zevs::Mocket* { return web_hmi_in_mocket_.get(); }
auto TestFixture::WebHmiOut() -> zevs::Mocket* { return web_hmi_out_mocket_.get(); }
auto TestFixture::Management() -> zevs::Mocket* { return management_mocket_.get(); }
auto TestFixture::Kinematics() -> zevs::Mocket* { return kinematics_mocket_.get(); }
auto TestFixture::Scanner() -> zevs::Mocket* { return scanner_mocket_.get(); }
auto TestFixture::WeldSystem() -> zevs::Mocket* { return weld_system_mocket_.get(); }
auto TestFixture::Timer() -> zevs::MocketTimer* { return timer_mocket_.get(); }

auto TestFixture::DescribeQueue() const -> std::string {
  std::string description = "{WebHmiIn:" + std::to_string(web_hmi_in_mocket_->Queued()) +
                            ", WebHmiOut:" + std::to_string(web_hmi_out_mocket_->Queued()) +
                            ", Management:" + std::to_string(management_mocket_->Queued()) +
                            ", Kinematics:" + std::to_string(kinematics_mocket_->Queued()) +
                            ", Scanner:" + std::to_string(scanner_mocket_->Queued()) +
                            ", WeldSystem:" + std::to_string(weld_system_mocket_->Queued()) + "}";

  return description;
}

auto TestFixture::ScannerData() -> ScannerDataWrapper* { return &scanner_data_; }

auto TestFixture::Sut() -> ApplicationWrapper* { return application_sut_.get(); }

auto TestFixture::GetDatabase() -> SQLite::Database* { return &database_; }

auto TestFixture::GetConfigManagerMock() -> configuration::ConfigManagerMock* { return config_manager_mock_.get(); }

const double TOP_LEVEL    = 0.0;
const double BOTTOM_LEVEL = -30.0;

ScannerDataWrapper::ScannerDataWrapper() {
  data_.groove[0] = {-22, TOP_LEVEL};
  data_.groove[1] = {-15, BOTTOM_LEVEL};
  data_.groove[2] = {-10, BOTTOM_LEVEL};
  data_.groove[3] = {0, BOTTOM_LEVEL};
  data_.groove[4] = {10, BOTTOM_LEVEL};
  data_.groove[5] = {15, BOTTOM_LEVEL};
  data_.groove[6] = {22, TOP_LEVEL};

  data_.confidence = common::msg::scanner::SliceConfidence::HIGH;

  constexpr std::array<common::msg::scanner::Coordinate, 7> anchor_line = {
      common::msg::scanner::Coordinate{-22.0, TOP_LEVEL},      common::msg::scanner::Coordinate{-15.0, BOTTOM_LEVEL},
      common::msg::scanner::Coordinate{-10.0, BOTTOM_LEVEL},   common::msg::scanner::Coordinate{0.0, BOTTOM_LEVEL},
      common::msg::scanner::Coordinate{10.0, BOTTOM_LEVEL},    common::msg::scanner::Coordinate{15.0, BOTTOM_LEVEL},
      common::msg::scanner::Coordinate{22.0, TOP_LEVEL}};

  const double segment_count = static_cast<double>(anchor_line.size() - 1);

  for (std::size_t i = 0; i < common::msg::scanner::LINE_ARRAY_SIZE; ++i) {
    if (common::msg::scanner::LINE_ARRAY_SIZE == 1) {
      data_.line[i] = anchor_line.front();
      continue;
    }

    const double t          = static_cast<double>(i) / (common::msg::scanner::LINE_ARRAY_SIZE - 1);
    const double scaled_pos = t * segment_count;
    const std::size_t idx   = std::min<std::size_t>(static_cast<std::size_t>(scaled_pos), anchor_line.size() - 2);
    const double local_t    = scaled_pos - static_cast<double>(idx);

    const auto& start = anchor_line[idx];
    const auto& end   = anchor_line[idx + 1];

    data_.line[i].x = std::lerp(start.x, end.x, local_t);
    data_.line[i].y = std::lerp(start.y, end.y, local_t);
  }
}

auto ScannerDataWrapper::Get() const -> common::msg::scanner::SliceData { return data_; }
auto ScannerDataWrapper::GetWithConfidence(common::msg::scanner::SliceConfidence confidence) const
    -> common::msg::scanner::SliceData {
  auto data       = data_;
  data.confidence = confidence;

  return data;
}

auto ScannerDataWrapper::ShiftHorizontal(double value) -> ScannerDataWrapper& {
  for (auto& coord : data_.groove) {
    coord.x += value;
  }

  for (auto& coord : data_.line) {
    coord.x += value;
  }

  return *this;
}

auto ScannerDataWrapper::FillUp(double value) -> ScannerDataWrapper& {
  for (auto& coord : data_.groove) {
    if (coord.y < TOP_LEVEL) coord.y += value;
  }

  for (auto& coord : data_.line) {
    if (coord.y < TOP_LEVEL) coord.y += value;
  }

  return *this;
}

ControllerFixture::ControllerFixture(clock_functions::SystemClockNowFunc system_clock_now_func)
    : system_clock_now_func_(std::move(system_clock_now_func)) {
  auto mock_plc_ptr     = std::make_unique<MockPlc>();
  mock_plc_             = mock_plc_ptr.get();
  controller_messenger_ = std::make_unique<controller::ControllerMessenger>(std::move(mock_plc_ptr), PLC_CYCLE_TIME_MS,
                                                                            system_clock_now_func_, ENDPOINT_BASE_URL);
}

void ControllerFixture::Start() {
  controller_messenger_->ThreadEntry("ControllerMessenger");

  auto* factory       = zevs::GetMocketFactory();
  management_mocket_  = factory->GetMocket(zevs::Endpoint::BIND, MakeEndpoint("management"));
  kinematics_mocket_  = factory->GetMocket(zevs::Endpoint::BIND, MakeEndpoint("kinematics"));
  weld_system_mocket_ = factory->GetMocket(zevs::Endpoint::BIND, MakeEndpoint("weld-system"));
  timer_mocket_       = factory->GetMocketTimer(TIMER_INSTANCE_2);
}

void ControllerFixture::Stop() { controller_messenger_.reset(); }

auto ControllerFixture::Management() -> zevs::Mocket* { return management_mocket_.get(); }
auto ControllerFixture::Kinematics() -> zevs::Mocket* { return kinematics_mocket_.get(); }
auto ControllerFixture::WeldSystem() -> zevs::Mocket* { return weld_system_mocket_.get(); }
auto ControllerFixture::Timer() -> zevs::MocketTimer* { return timer_mocket_.get(); }
auto ControllerFixture::Mock() -> MockPlc* { return mock_plc_; }
auto ControllerFixture::Sut() -> controller::ControllerMessenger* { return controller_messenger_.get(); }

MultiFixture::MultiFixture()
    : ctrl_([wrapper = app_.GetClockNowFuncWrapper()]() { return wrapper->GetSystemClock(); }) {
  app_.StartApplication();
  ctrl_.Start();

  app_.WebHmiIn()->SetDispatchObserver([this](const zevs::Mocket&, zevs::Mocket::Location) { ForwardAllPending(); });

  app_.Scanner()->SetDispatchObserver([this](const zevs::Mocket&, zevs::Mocket::Location) { ForwardAllPending(); });

  app_.Timer()->SetDispatchObserver(
      [this](const zevs::MocketTimer&, zevs::MocketTimer::Location) { ForwardAllPending(); });

  ctrl_.Timer()->SetDispatchObserver(
      [this](const zevs::MocketTimer&, zevs::MocketTimer::Location) { ForwardAllPending(); });
}

void MultiFixture::PlcDataUpdate() {
  // A call to ForwardAllPending is done before timer dispatch to make sure
  // events and state changes are reflected in output data to plc
  ForwardAllPending();
  ctrl_.Timer()->Dispatch("controller_periodic_update");
}

auto MultiFixture::Main() -> TestFixture& { return app_; }
auto MultiFixture::Ctrl() -> ControllerFixture& { return ctrl_; }

void MultiFixture::ForwardAllPending() {
  enum class Side { APP, CTRL };

  auto forward_one_side = [this](Side current_side) -> bool {
    bool moved_any_message = false;

    struct MocketPair {
      zevs::Mocket* source;
      zevs::Mocket* destination;
    };

    const std::array<MocketPair, 3> bridge_pairs =
        (current_side == Side::APP)
            ? std::array<MocketPair, 3>{{
                  {.source = app_.Management(),  .destination = ctrl_.Management()},
                  {.source = app_.Kinematics(),  .destination = ctrl_.Kinematics()},
                  {.source = app_.WeldSystem(),  .destination = ctrl_.WeldSystem()},
              }}
            : std::array<MocketPair, 3>{{
                  {.source = ctrl_.Management(), .destination = app_.Management()},
                  {.source = ctrl_.Kinematics(), .destination = app_.Kinematics()},
                  {.source = ctrl_.WeldSystem(), .destination = app_.WeldSystem()},
              }};

    for (const auto& mocket_pair : bridge_pairs) {
      while (mocket_pair.source && mocket_pair.destination && mocket_pair.source->Queued() > 0) {
        auto message = mocket_pair.source->ReceiveMessage();
        if (!message) {
          break;
        }
        mocket_pair.destination->DispatchMessage(std::move(message));
        moved_any_message = true;
      }
    }
    return moved_any_message;
  };

  // APP first, then CTRL; repeat until neither side moved
  while (true) {
    bool made_progress  = false;
    made_progress      |= forward_one_side(Side::APP);
    made_progress      |= forward_one_side(Side::CTRL);
    if (!made_progress) {
      break;
    }
  }
}

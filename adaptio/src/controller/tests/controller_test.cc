#include "controller/controller.h"

#include <doctest/doctest.h>

#include <boost/outcome/result.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>

#include "../controller_messenger.h"
#include "common/messages/management.h"
#include "common/zevs/zevs_test_support.h"
#include "controller/controller_data.h"

//  NOLINTBEGIN(*-magic-numbers, *-optional-access)
//
using controller::Controller;
using controller::ControllerMessenger;

namespace {
const uint32_t TIMER_INSTANCE       = 1;
const uint32_t TRACKING_MODE_CENTER = 1;
const uint32_t TRACKING_MODE_RIGHT  = 2;
const uint32_t HEARTBEAT_VAL_1      = 55;
const uint32_t HEARTBEAT_VAL_2      = 56;

const uint32_t SEQUENCE_NONE          = 0;
const uint32_t SEQUENCE_TRACKING      = 1;
const uint32_t SEQUENCE_AUTO_WELDING  = 2;
const uint32_t SEQUENCE_AUTO_CAL_MOVE = 3;
const uint32_t SEQUENCE_ABP_CAP       = 4;
}  // namespace

struct MockController : public Controller {
  auto Connect() -> boost::outcome_v2::result<bool> override {
    is_connected = true;
    return true;
  }

  void Disconnect() override { is_connected = false; }
  auto IsConnected() -> bool override { return is_connected; }
  void WriteAdaptioOutput(controller::AdaptioOutput data) override { adaptio_output = data; }
  void WriteTrackingOutput(controller::TrackOutput data) override { track_output = data; }

  controller::AdaptioOutput adaptio_output;
  controller::TrackOutput track_output;
  bool is_connected = false;
};

struct Fixture {
  Fixture() {
    auto controller_ptr = std::make_unique<MockController>();
    controller          = controller_ptr.get();
    sut                 = std::make_unique<ControllerMessenger>(
        std::move(controller_ptr), 100, []() { return std::chrono::system_clock::now(); },
        []() { return std::chrono::steady_clock::now(); }, "mock");

    sut->ThreadEntry("Controller messenger");

    management_mocket  = factory.GetMocket(zevs::Endpoint::BIND, "inproc://mock/management");
    kinematics_mocket  = factory.GetMocket(zevs::Endpoint::BIND, "inproc://mock/kinematics");
    weld_system_mocket = factory.GetMocket(zevs::Endpoint::BIND, "inproc://mock/weld-system");
    CHECK(management_mocket);
    CHECK(kinematics_mocket);
    CHECK(weld_system_mocket);

    CHECK(controller->IsConnected());
    CHECK(management_mocket->Receive<common::msg::management::SubscribeReadyState>());
  }
  zevs::MocketFactory factory;
  MockController* controller;
  std::unique_ptr<ControllerMessenger> sut;
  zevs::MocketPtr management_mocket;
  zevs::MocketPtr kinematics_mocket;
  zevs::MocketPtr weld_system_mocket;
};

TEST_SUITE("Controller Messenger") {
  TEST_CASE("Start joint tracking") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    // Dispatch timeout on ControllerMessenger
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());

    controller::TrackInput track_data{};
    track_data.set_joint_tracking_mode(TRACKING_MODE_RIGHT);
    fixture.sut->OnTrackingInputUpdate(track_data);

    controller::AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(1);
    fixture.sut->OnAdaptioInputUpdate(data);

    auto msg = fixture.management_mocket->Receive<common::msg::management::TrackingStart>();
    CHECK_EQ(msg.value().horizontal_offset, 0.0);
    CHECK_EQ(msg.value().joint_tracking_mode, TRACKING_MODE_RIGHT);
    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    // Dispatch timeout on ControllerMessenger
    timer_mocket->Dispatch("controller_periodic_update");

    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());
    CHECK(fixture.controller->track_output.get_status_active());
    CHECK_EQ(SEQUENCE_TRACKING, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("NotifyHandoverToManual") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    controller::AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(1);
    fixture.sut->OnAdaptioInputUpdate(data);

    fixture.management_mocket->Dispatch(common::msg::management::NotifyHandoverToManual{});

    // Dispatch timeout on ControllerMessenger
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());
    CHECK(fixture.controller->track_output.get_status_active());
    CHECK(fixture.controller->track_output.get_status_shallow());
  }

  TEST_CASE("Scanner error") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    controller::AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(1);
    fixture.sut->OnAdaptioInputUpdate(data);

    fixture.management_mocket->Dispatch(common::msg::management::ScannerError{});

    // Dispatch timeout
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK(fixture.controller->adaptio_output.get_status_error());
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());

    // Clear the error by sending stop
    controller::AdaptioInput input_stop{};
    input_stop.set_commands_stop(true);
    fixture.sut->OnAdaptioInputUpdate(input_stop);

    // Dispatch timeout
    timer_mocket->Dispatch("controller_periodic_update");

    CHECK(fixture.controller->adaptio_output.get_status_ready());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());
  }

  TEST_CASE("Start abp failure") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    // Dispatch timeout
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    // Check ready bits
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_tracking());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_ready_for_abp());

    // Check active and error bits
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());

    controller::AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(2);
    fixture.sut->OnAdaptioInputUpdate(data);

    // Check that no start msg was sent
    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    // Dispatch timeout
    timer_mocket->Dispatch("controller_periodic_update");

    // Check active and error bit
    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK(fixture.controller->adaptio_output.get_status_error());
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("Start abp success") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_READY});

    // Dispatch timeout
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    // Check ready bits
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_tracking());
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_abp());

    controller::AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(2);
    fixture.sut->OnAdaptioInputUpdate(data);

    CHECK(fixture.management_mocket->Receive<common::msg::management::TrackingStart>());
    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPStart>());

    // Dispatch timeout
    timer_mocket->Dispatch("controller_periodic_update");

    // Check active and error bit
    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK_EQ(SEQUENCE_AUTO_WELDING, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("JT to ABP to JT") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});
    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_READY});

    {
      // Start Joint Tracking
      controller::TrackInput track_data{};
      track_data.set_joint_tracking_mode(TRACKING_MODE_RIGHT);
      fixture.sut->OnTrackingInputUpdate(track_data);

      controller::AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(1);
      fixture.sut->OnAdaptioInputUpdate(data);
    }

    auto msg = fixture.management_mocket->Receive<common::msg::management::TrackingStart>();
    CHECK_EQ(msg.value().horizontal_offset, 0.0);
    CHECK_EQ(msg.value().joint_tracking_mode, TRACKING_MODE_RIGHT);
    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    // Dispatch timeout on ControllerMessenger
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_TRACKING, fixture.controller->adaptio_output.get_active_sequence_type());

    // Start ABP
    controller::AdaptioInput start_abp{};
    start_abp.set_commands_start(true);
    start_abp.set_sequence_type(2);
    fixture.sut->OnAdaptioInputUpdate(start_abp);

    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPStart>());
    CHECK_EQ(fixture.management_mocket->Queued(), 0);
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_AUTO_WELDING, fixture.controller->adaptio_output.get_active_sequence_type());

    {
      // Start Joint Tracking
      // Note that PLC sets tracking CENTER when going back
      controller::TrackInput track_data{};
      track_data.set_joint_tracking_mode(TRACKING_MODE_CENTER);
      fixture.sut->OnTrackingInputUpdate(track_data);

      controller::AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(1);
      fixture.sut->OnAdaptioInputUpdate(data);
    }

    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPStop>());
    auto update_msg = fixture.management_mocket->Receive<common::msg::management::TrackingUpdate>();
    CHECK_EQ(update_msg.value().joint_tracking_mode, TRACKING_MODE_CENTER);
    CHECK_EQ(fixture.management_mocket->Queued(), 0);
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_TRACKING, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("Active sequence auto cal move") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_READY});

    // Dispatch timeout
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    // Check ready bits
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_tracking());
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_abp());

    // Check active sequence none
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());

    fixture.management_mocket->Dispatch(common::msg::management::ReadyState{
        .state = common::msg::management::ReadyState::State::NOT_READY_AUTO_CAL_MOVE});

    timer_mocket->Dispatch("controller_periodic_update");

    // Check active sequence auto cal move
    CHECK_EQ(SEQUENCE_AUTO_CAL_MOVE, fixture.controller->adaptio_output.get_active_sequence_type());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_ready());
    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_READY});

    timer_mocket->Dispatch("controller_periodic_update");

    // Check active sequence none
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());
    CHECK(fixture.controller->adaptio_output.get_status_ready());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());
  }

  TEST_CASE("Heartbeat") {
    Fixture fixture;
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);

    CHECK(fixture.sut->ValidateHeartbeat());
    {
      controller::AdaptioInput data{};
      data.set_heartbeat(HEARTBEAT_VAL_1);
      fixture.sut->OnAdaptioInputUpdate(data);
    }

    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(fixture.controller->adaptio_output.get_heartbeat(), HEARTBEAT_VAL_1);
    CHECK(fixture.sut->ValidateHeartbeat());

    {
      controller::AdaptioInput data{};
      data.set_heartbeat(HEARTBEAT_VAL_2);
      fixture.sut->OnAdaptioInputUpdate(data);
    }

    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(fixture.controller->adaptio_output.get_heartbeat(), HEARTBEAT_VAL_2);
    CHECK(fixture.sut->ValidateHeartbeat());
  }

  TEST_CASE("No Tracking without start") {
    // Check that only setting the sequence (without start) does not start tracking
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    controller::TrackInput track_data{};
    track_data.set_joint_tracking_mode(TRACKING_MODE_RIGHT);
    fixture.sut->OnTrackingInputUpdate(track_data);

    controller::AdaptioInput data{};
    data.set_commands_start(false);
    data.set_sequence_type(1);
    fixture.sut->OnAdaptioInputUpdate(data);

    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    // Dispatch timeout on ControllerMessenger
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    CHECK_FALSE(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());
    CHECK_FALSE(fixture.controller->track_output.get_status_active());
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("No Error without start") {
    // Check that only setting the sequence does not cause ERROR state
    // when start is false even though ReadyState is NOT_READY
    Fixture fixture;

    controller::AdaptioInput data{};
    data.set_commands_start(false);
    data.set_sequence_type(1);
    fixture.sut->OnAdaptioInputUpdate(data);

    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    // Dispatch timeout on ControllerMessenger
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    CHECK_FALSE(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());
    CHECK_FALSE(fixture.controller->track_output.get_status_active());
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("ABP CAP success") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    // Dispatch timeout
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    // Check ready bits
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_tracking());

    {
      // Start Joint Tracking
      controller::TrackInput track_data{};
      track_data.set_joint_tracking_mode(TRACKING_MODE_RIGHT);
      fixture.sut->OnTrackingInputUpdate(track_data);

      controller::AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(SEQUENCE_TRACKING);
      fixture.sut->OnAdaptioInputUpdate(data);
    }

    auto msg = fixture.management_mocket->Receive<common::msg::management::TrackingStart>();
    CHECK_EQ(msg.value().horizontal_offset, 0.0);
    CHECK_EQ(msg.value().joint_tracking_mode, TRACKING_MODE_RIGHT);
    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_CAP_READY});

    // Dispatch timeout on ControllerMessenger
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_TRACKING, fixture.controller->adaptio_output.get_active_sequence_type());

    CHECK(fixture.controller->adaptio_output.get_status_ready_for_auto_cap());

    controller::AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(SEQUENCE_ABP_CAP);
    fixture.sut->OnAdaptioInputUpdate(data);

    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPCapStart>());

    // Dispatch timeout
    timer_mocket->Dispatch("controller_periodic_update");

    // Check active and error bit
    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_error());
    CHECK_EQ(SEQUENCE_ABP_CAP, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("ABP CAP failure not ready") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    // Dispatch timeout
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    // Check ready bits
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_tracking());

    {
      // Start Joint Tracking
      controller::TrackInput track_data{};
      track_data.set_joint_tracking_mode(TRACKING_MODE_RIGHT);
      fixture.sut->OnTrackingInputUpdate(track_data);

      controller::AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(SEQUENCE_TRACKING);
      fixture.sut->OnAdaptioInputUpdate(data);
    }

    auto msg = fixture.management_mocket->Receive<common::msg::management::TrackingStart>();
    CHECK_EQ(msg.value().horizontal_offset, 0.0);
    CHECK_EQ(msg.value().joint_tracking_mode, TRACKING_MODE_RIGHT);
    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    // Dispatch timeout to establish tracking
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_TRACKING, fixture.controller->adaptio_output.get_active_sequence_type());

    // Stay in ABP_READY (not CAP ready) to test failure
    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_READY});

    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_FALSE(fixture.controller->adaptio_output.get_status_ready_for_auto_cap());

    controller::AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(SEQUENCE_ABP_CAP);
    fixture.sut->OnAdaptioInputUpdate(data);

    // Check that no CAP start msg was sent
    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    // Dispatch timeout
    timer_mocket->Dispatch("controller_periodic_update");

    // Check active and error bit
    CHECK(fixture.controller->adaptio_output.get_status_active());
    CHECK(fixture.controller->adaptio_output.get_status_error());
    CHECK_EQ(SEQUENCE_NONE, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("ABP to CAP transition") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_READY});

    // Start ABP first
    controller::AdaptioInput start_abp{};
    start_abp.set_commands_start(true);
    start_abp.set_sequence_type(SEQUENCE_AUTO_WELDING);
    fixture.sut->OnAdaptioInputUpdate(start_abp);

    CHECK(fixture.management_mocket->Receive<common::msg::management::TrackingStart>());
    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPStart>());

    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_AUTO_WELDING, fixture.controller->adaptio_output.get_active_sequence_type());

    // Now transition to CAP ready state
    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_CAP_READY});

    timer_mocket->Dispatch("controller_periodic_update");
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_auto_cap());

    // Transition to CAP
    controller::AdaptioInput start_cap{};
    start_cap.set_commands_start(true);
    start_cap.set_sequence_type(SEQUENCE_ABP_CAP);
    fixture.sut->OnAdaptioInputUpdate(start_cap);

    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPCapStart>());

    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_ABP_CAP, fixture.controller->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("CAP to tracking transition") {
    Fixture fixture;

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::TRACKING_READY});

    // Dispatch timeout
    auto timer_mocket = fixture.factory.GetMocketTimer(TIMER_INSTANCE);
    timer_mocket->Dispatch("controller_periodic_update");

    // Check ready bits
    CHECK(fixture.controller->adaptio_output.get_status_ready_for_tracking());

    {
      // Start Joint Tracking
      controller::TrackInput track_data{};
      track_data.set_joint_tracking_mode(TRACKING_MODE_RIGHT);
      fixture.sut->OnTrackingInputUpdate(track_data);

      controller::AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(SEQUENCE_TRACKING);
      fixture.sut->OnAdaptioInputUpdate(data);
    }

    auto msg = fixture.management_mocket->Receive<common::msg::management::TrackingStart>();
    CHECK_EQ(msg.value().horizontal_offset, 0.0);
    CHECK_EQ(msg.value().joint_tracking_mode, TRACKING_MODE_RIGHT);
    CHECK_EQ(fixture.management_mocket->Queued(), 0);

    fixture.management_mocket->Dispatch(
        common::msg::management::ReadyState{.state = common::msg::management::ReadyState::State::ABP_CAP_READY});

    // Dispatch timeout on ControllerMessenger
    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_TRACKING, fixture.controller->adaptio_output.get_active_sequence_type());

    CHECK(fixture.controller->adaptio_output.get_status_ready_for_auto_cap());

    // Start CAP
    controller::AdaptioInput start_cap{};
    start_cap.set_commands_start(true);
    start_cap.set_sequence_type(SEQUENCE_ABP_CAP);
    fixture.sut->OnAdaptioInputUpdate(start_cap);

    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPCapStart>());

    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_ABP_CAP, fixture.controller->adaptio_output.get_active_sequence_type());

    // Transition to tracking
    controller::TrackInput track_data{};
    track_data.set_joint_tracking_mode(TRACKING_MODE_CENTER);
    fixture.sut->OnTrackingInputUpdate(track_data);

    controller::AdaptioInput start_tracking{};
    start_tracking.set_commands_start(true);
    start_tracking.set_sequence_type(SEQUENCE_TRACKING);
    fixture.sut->OnAdaptioInputUpdate(start_tracking);

    CHECK(fixture.management_mocket->Receive<common::msg::management::ABPCapStop>());
    auto update_msg = fixture.management_mocket->Receive<common::msg::management::TrackingUpdate>();
    CHECK_EQ(update_msg.value().joint_tracking_mode, TRACKING_MODE_CENTER);

    timer_mocket->Dispatch("controller_periodic_update");
    CHECK_EQ(SEQUENCE_TRACKING, fixture.controller->adaptio_output.get_active_sequence_type());
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)

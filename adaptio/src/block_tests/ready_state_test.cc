
#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>
#include <utility>

#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "helpers/helpers.h"
#include "helpers/helpers_abp_parameters.h"
#include "helpers/helpers_calibration_v2.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_kinematics.h"
#include "helpers/helpers_settings.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

TEST_SUITE("Ready state") {
  TEST_CASE("Basic") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);

    fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
    auto msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::NOT_READY);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::TRACKING_READY);

    // Start calibration (V2) should transition to NOT_READY immediately
    WeldObjectCalStart(fixture, 4.0, 25.0, 1000.0);
    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::NOT_READY);
  }

  TEST_CASE("ABP ready 1") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);

    fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
    auto msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::NOT_READY);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::TRACKING_READY);

    StoreDefaultABPParams(fixture);

    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::ABP_READY);
  }

  TEST_CASE("ABP ready 2") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);

    fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
    auto msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::NOT_READY);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::TRACKING_READY);

    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();

    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::ABP_READY);
  }

  TEST_CASE("ABP / JT ready no edge-sensor") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);
    StoreDefaultABPParams(fixture);
    StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);

    fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
    auto msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::TRACKING_READY);

    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::ABP_READY);
  }

  TEST_CASE("ABP / JT ready edge-sensor") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);
    StoreDefaultABPParams(fixture);
    StoreSettings(fixture, TestSettings{.use_edge_sensor = true}, true); /* same as default value */

    fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
    auto msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::NOT_READY);

    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);

    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::NOT_READY);

    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);
    msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK_EQ(msg->state, common::msg::management::ReadyState::State::ABP_READY);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)

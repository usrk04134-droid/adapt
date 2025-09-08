#pragma once

#include <doctest/doctest.h>

#include "block_tests/helpers.h"
#include "common/messages/kinematics.h"

inline auto CheckAndDispatchGetWeldAxis(TestFixture& fixture, double position, double velocity, double radius) {
  auto const req = fixture.Kinematics()->Receive<common::msg::kinematics::GetWeldAxisData>();
  CHECK(req);

  fixture.Kinematics()->Dispatch(common::msg::kinematics::GetWeldAxisDataRsp{
      .client_id = req->client_id,
      .position  = position,
      .velocity  = velocity,
      .radius    = radius,
  });
}

inline auto DispatchKinematicsStateChange(TestFixture& fixture,
                                          common::msg::kinematics::StateChange::State weld_axis_state) {
  fixture.Kinematics()->Dispatch(common::msg::kinematics::StateChange{
      .weld_axis_state = weld_axis_state,
  });
}

inline auto DispatchKinematicsEdgeStateChange(TestFixture& fixture,
                                              common::msg::kinematics::EdgeStateChange::State edge_state) {
  fixture.Kinematics()->Dispatch(common::msg::kinematics::EdgeStateChange{
      .edge_state = edge_state,
  });
}

inline auto CheckAndDispatchEdgePosition(TestFixture& fixture, double position) {
  auto const req = fixture.Kinematics()->Receive<common::msg::kinematics::GetEdgePosition>();
  CHECK(req);

  fixture.Kinematics()->Dispatch(common::msg::kinematics::GetEdgePositionRsp{
      .client_id = req->client_id,
      .position  = position,
  });
}

#pragma once

#include <doctest/doctest.h>

#include "common/messages/weld_system.h"
#include "helpers.h"
#include "weld_system_client/weld_system_types.h"

inline auto CheckAndDispatchWeldSystemDataRsp(TestFixture& fixture, weld_system::WeldSystemId id,
                                              common::msg::weld_system::GetWeldSystemDataRsp status_rsp) {
  auto const req = fixture.WeldSystem()->Receive<common::msg::weld_system::GetWeldSystemData>();
  CHECK(req);
  CHECK_EQ(req->index, static_cast<uint32_t>(id));

  status_rsp.transaction_id = req->transaction_id;

  fixture.WeldSystem()->Dispatch(status_rsp);
}

inline auto DispatchWeldSystemStateChange(TestFixture& fixture, weld_system::WeldSystemId id,
                                          common::msg::weld_system::OnWeldSystemStateChange::State state) {
  fixture.WeldSystem()->Dispatch(common::msg::weld_system::OnWeldSystemStateChange{
      .index = static_cast<uint32_t>(id),
      .state = state,
  });
}

#pragma once
#include <doctest/doctest.h>

#include "block_tests/helpers.h"
#include "common/zevs/zevs_test_support.h"
#include "web_hmi/web_hmi_json_helpers.h"

inline auto ReceiveJsonByName(TestFixture& fixture, const std::string& name) -> nlohmann::json {
  const auto fn_match = [name](const zevs::MessagePtr& message) {
    auto const name0 = web_hmi::UnpackMessageName(message);

    return name0 == name;
  };

  auto rec_msg = fixture.WebHmiOut()->Receive(fn_match);
  DOCTEST_CHECK_MESSAGE(rec_msg, "Failed to receive message with name: " << name);
  return rec_msg ? web_hmi::UnpackMessagePayload(rec_msg) : nlohmann::json{};
}

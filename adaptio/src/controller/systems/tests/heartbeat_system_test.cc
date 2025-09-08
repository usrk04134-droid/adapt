#include "../heartbeat/heartbeat_system.h"

#include <doctest/doctest.h>

#include <chrono>

using controller::HeartbeatSystem;

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

TEST_SUITE("Heartbeat System") {
  TEST_CASE("Apply") {
    bool heartbeat_lost_callback_called = false;

    HeartbeatSystem heartbeat_system(
        []() -> bool { return true; },
        [&heartbeat_lost_callback_called]() -> void { heartbeat_lost_callback_called = true; },
        []() { return std::chrono::steady_clock::now(); });

    heartbeat_system.Apply();
    CHECK_FALSE(heartbeat_lost_callback_called);
  }

  TEST_CASE("ApplyWhenHeartbeatLost") {
    bool validate_heartbeat             = true;
    bool heartbeat_lost_callback_called = false;
    std::chrono::steady_clock::duration steady_clock_now{std::chrono::steady_clock::now().time_since_epoch()};

    HeartbeatSystem heartbeat_system(
        [&validate_heartbeat]() -> bool { return validate_heartbeat; },
        [&heartbeat_lost_callback_called]() -> void { heartbeat_lost_callback_called = true; },
        [&steady_clock_now]() { return std::chrono::time_point<std::chrono::steady_clock>(steady_clock_now); });

    heartbeat_system.Apply();
    validate_heartbeat = false;
    CHECK_FALSE(heartbeat_lost_callback_called);

    // Step steady clock 499ms and recheck
    steady_clock_now += static_cast<std::chrono::milliseconds>(499);
    heartbeat_system.Apply();
    CHECK_FALSE(heartbeat_lost_callback_called);

    // Step steady clock 2ms more and recheck
    steady_clock_now += static_cast<std::chrono::milliseconds>(2);
    heartbeat_system.Apply();
    CHECK(heartbeat_lost_callback_called);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)


// NOLINTBEGIN(*-magic-numbers, *-optional-access, *-use-nodiscard)
#include <doctest/doctest.h>

#include <cstdint>

#include "../zevs_socket.h"
#include "../zevs_test_support.h"

const uint32_t TIMER_INSTANCE = 1;
const uint32_t DURATION_MS    = 1000;

class TimerUser {
 public:
  bool timeout1 = false;
  bool timeout2 = false;
  bool timeout3 = false;
  void OnTimeout1() { timeout1 = true; }
  void OnTimeout2() { timeout2 = true; }
  void OnTimeout3() { timeout3 = true; }
};

TEST_SUITE("zevs_test_support_timer") {
  TEST_CASE("dispatch_order") {
    zevs::MocketFactory factory;
    zevs::MocketEventLoop event_loop("MOCK_EVENT_LOOP");
    TimerUser obj;

    auto timer = zevs::GetFactory()->CreateTimer(event_loop);
    timer->Request(&TimerUser::OnTimeout1, &obj, DURATION_MS, "test_task");
    timer->Request(&TimerUser::OnTimeout2, &obj, DURATION_MS, "test_task");
    timer->Request(&TimerUser::OnTimeout3, &obj, DURATION_MS, "test_task");

    auto mocket_timer = factory.GetMocketTimer(TIMER_INSTANCE);

    {
      mocket_timer->Dispatch("test_task");
      bool timer_state = obj.timeout1 && !obj.timeout2 && !obj.timeout3;
      CHECK(timer_state);
    }

    {
      mocket_timer->Dispatch("test_task");
      bool timer_state = obj.timeout1 && obj.timeout2 && !obj.timeout3;
      CHECK(timer_state);
    }

    {
      mocket_timer->Dispatch("test_task");
      bool timer_state = obj.timeout1 && obj.timeout2 && obj.timeout3;
      CHECK(timer_state);
    }
  }

  TEST_CASE("dispatch_periodic") {
    zevs::MocketFactory factory;
    zevs::MocketEventLoop event_loop("MOCK_EVENT_LOOP");
    TimerUser obj;

    auto timer = zevs::GetFactory()->CreateTimer(event_loop);
    timer->RequestPeriodic(&TimerUser::OnTimeout1, &obj, DURATION_MS, "test_task");

    auto mocket_timer = factory.GetMocketTimer(TIMER_INSTANCE);

    {
      mocket_timer->Dispatch("test_task");
      CHECK(obj.timeout1);
    }

    obj.timeout1 = false;
    // Check that period timer can be dispatched again
    {
      mocket_timer->Dispatch("test_task");
      CHECK(obj.timeout1);
    }
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access, *-use-nodiscard)

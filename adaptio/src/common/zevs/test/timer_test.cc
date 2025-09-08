#include "../zevs_core.h"
#include "../zevs_socket.h"
// the tests in this file is only on the API level, thus the above headers

#include <doctest/doctest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <vector>

// NOLINTBEGIN(*-magic-numbers)

const auto LONG_WAIT       = std::chrono::milliseconds(1000);
const auto EXTRA_LONG_WAIT = std::chrono::milliseconds(10000);

const uint32_t DURATION_10_MS = 10;
const uint32_t DURATION_20_MS = 20;
const uint32_t DURATION_50_MS = 50;

class TimerClient {
 public:
  void OnTimeout1() { handler1(); }
  void OnTimeout2() { handler2(); }
  void OnTimeout3() { handler3(); }

  zevs::EventLoopPtr event_loop;
  zevs::TimerPtr timer;
  std::function<void()> handler1;
  std::function<void()> handler2;
  std::function<void()> handler3;
  uint32_t task_id1 = 0;
  uint32_t task_id2 = 0;
  uint32_t task_id3 = 0;
  uint32_t joker_id = 0;
  std::vector<uint32_t> timeouts;
};

TEST_SUITE("zevs_timer_tests") {
  TEST_CASE("zevs_timer_periodic") {
    auto context = zevs::GetCoreFactory()->CreateContext();
    TimerClient obj;

    obj.handler1 = [&]() {
      obj.timeouts.push_back(obj.task_id1);
      // cancel after 3 10ms timeouts
      if (obj.timeouts.size() == 3) {
        obj.timer->Cancel(obj.task_id1);
      }
    };

    obj.handler2 = [&]() {
      obj.timeouts.push_back(obj.task_id2);
      // exit after 1 50ms timeout
      obj.event_loop->Exit();
    };

    auto run = [&]() {
      obj.event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      obj.timer      = zevs::GetFactory()->CreateTimer(*obj.event_loop);
      obj.task_id1   = obj.timer->RequestPeriodic(&TimerClient::OnTimeout1, &obj, DURATION_10_MS, "10ms task");
      obj.task_id2   = obj.timer->RequestPeriodic(&TimerClient::OnTimeout2, &obj, DURATION_50_MS, "50ms task");
      obj.event_loop->Run();
    };

    auto finished = std::async(run);

    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);

    // Check that only 3 10ms timeouts and 1 50ms timeout were received
    auto exp_timeouts = std::vector<uint32_t>{
        obj.task_id1,
        obj.task_id1,
        obj.task_id1,
        obj.task_id2,
    };
    CHECK_EQ(obj.timeouts, exp_timeouts);
  }

  TEST_CASE("zevs_timer_single") {
    auto context = zevs::GetCoreFactory()->CreateContext();
    TimerClient obj;

    obj.handler1 = [&]() {
      obj.timeouts.push_back(obj.task_id1);

      // cancel the extra 20ms timer
      obj.timer->Cancel(obj.joker_id);
    };

    obj.handler2 = [&]() {
      obj.timeouts.push_back(obj.task_id2);
      // exit after the 50ms timeout
      obj.event_loop->Exit();
    };

    obj.handler3 = [&]() { obj.timeouts.push_back(obj.task_id3); };

    auto run = [&]() {
      obj.event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      obj.timer      = zevs::GetFactory()->CreateTimer(*obj.event_loop);
      obj.task_id1   = obj.timer->Request(&TimerClient::OnTimeout1, &obj, DURATION_10_MS, "10ms task");

      // requesting an extra 20ms timer, which will be cancelled
      obj.joker_id = obj.timer->Request(&TimerClient::OnTimeout2, &obj, DURATION_20_MS, "20ms task");

      obj.task_id2 = obj.timer->Request(&TimerClient::OnTimeout2, &obj, DURATION_50_MS, "50ms task");
      obj.task_id3 = obj.timer->Request(&TimerClient::OnTimeout3, &obj, DURATION_20_MS, "20ms task 2");
      obj.event_loop->Run();
    };

    auto finished = std::async(run);

    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);

    // Check that 3 timeouts were received and in the right order
    auto exp_timeouts = std::vector<uint32_t>{
        obj.task_id1,
        obj.task_id3,
        obj.task_id2,
    };
    CHECK_EQ(obj.timeouts, exp_timeouts);
  }

  TEST_CASE("zevs_timer_remove") {
    auto context = zevs::GetCoreFactory()->CreateContext();
    std::vector<uint32_t> timeouts;
    auto obj_to_remove = std::make_unique<TimerClient>();

    obj_to_remove->handler1 = [&]() { timeouts.push_back(obj_to_remove->task_id1); };

    // obj will own the eventloop
    TimerClient obj;
    obj.handler1 = [&]() {
      timeouts.push_back(obj.task_id1);

      // delete the other TimerClient
      obj_to_remove.reset();
    };

    obj.handler2 = [&]() {
      timeouts.push_back(obj.task_id2);
      // exit after the 50ms timeout
      obj.event_loop->Exit();
    };

    auto run = [&]() {
      obj.event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      obj.timer      = zevs::GetFactory()->CreateTimer(*obj.event_loop);
      obj.task_id1   = obj.timer->Request(&TimerClient::OnTimeout1, &obj, DURATION_10_MS, "10ms task");
      obj.task_id2   = obj.timer->Request(&TimerClient::OnTimeout2, &obj, DURATION_50_MS, "50ms task");

      // start timer which will be removed with the deleted TimerClient
      obj_to_remove->timer = zevs::GetFactory()->CreateTimer(*obj.event_loop);
      obj_to_remove->task_id1 =
          obj_to_remove->timer->Request(&TimerClient::OnTimeout1, obj_to_remove.get(), DURATION_20_MS, "20ms task");

      obj.event_loop->Run();
    };

    auto finished = std::async(run);

    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);

    // Check that only two timeouts were received
    auto exp_timeouts = std::vector<uint32_t>{
        obj.task_id1,
        obj.task_id2,
    };
    CHECK_EQ(timeouts, exp_timeouts);
  }

  TEST_CASE("zevs_timer_plenty") {
    // This testcase is an example of how tricky things can become when threads
    // interacts. With the given number of timeouts, the default buffer limits of ZeroMQ
    // (default high water marks for sockets ~1000) are sufficient for the test to pass.

    // Requesting many timers takes significant time and involves synchronization between
    // this and the timer thread. Some of the first timers can start to expire before the
    // event loop is started which is done afterwards. If timers start to expire, the
    // timeout messages will be buffered by ZeroMQ until the eventloop is started and
    // receive polling starts.

    const uint32_t num_timeouts    = 500;
    const uint32_t duration_100_ms = 100;
    const uint32_t duration_200_ms = 200;

    auto context = zevs::GetCoreFactory()->CreateContext();
    TimerClient obj;

    obj.handler1 = [&]() { obj.timeouts.push_back(obj.task_id1); };

    obj.handler2 = [&]() {
      // exit after the 50ms timeout
      obj.event_loop->Exit();
    };

    auto run = [&]() {
      obj.event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      obj.timer      = zevs::GetFactory()->CreateTimer(*obj.event_loop);

      // request NUM_TIMEOUTS timeouts
      for (int i = 0; i < num_timeouts; ++i) {
        obj.task_id1 = obj.timer->Request(&TimerClient::OnTimeout1, &obj, duration_100_ms, "plenty_timers_task");
      }

      obj.task_id2 = obj.timer->Request(&TimerClient::OnTimeout2, &obj, duration_200_ms, "exit_task");
      obj.event_loop->Run();
    };

    auto finished = std::async(run);

    CHECK_EQ(finished.wait_for(EXTRA_LONG_WAIT), std::future_status::ready);

    // Check that NUM_TIMEOUTS timeouts were received (not counting the handler2 timeout)
    CHECK_EQ(obj.timeouts.size(), num_timeouts);
  }
}

// NOLINTEND(*-magic-numbers)

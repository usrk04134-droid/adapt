#include "../zevs_core.h"
#include "../zevs_socket.h"
// the tests in this file is only on the API level, thus the above headers

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <string>
#include <thread>

// NOLINTBEGIN(*-magic-numbers)
#include <doctest/doctest.h>

const auto LONG_WAIT   = std::chrono::milliseconds(1000);
const auto SHORT_WAIT  = std::chrono::milliseconds(100);
const uint32_t TESTVAL = 42;

struct A {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000100 };
  uint32_t i1 = 0;
};

struct B {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000101 };
  uint32_t i1 = 0;
  std::string s1;  // to make B not trivially copyable
};

struct C {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000102 };
  // No fields in this struct
};

template <typename Data>
class Peer {
 public:
  void OnEvent(Data data) { handler(data); }
  void PromiseRenew() {
    promise = {};
    future  = promise.get_future();
  }
  zevs::EventLoopPtr event_loop;
  zevs::SocketPtr socket;
  std::function<void(Data)> handler;
  std::promise<void> promise;
  std::future<void> future;
};

TEST_SUITE("zevs_socket_tests") {
  TEST_CASE("zevs_socket_getFactory") {
    auto* factory1 = zevs::GetFactory();
    auto* factory2 = zevs::GetFactory();
    CHECK_NE(factory1, nullptr);
    CHECK_EQ(factory1, factory2);
  }

  TEST_CASE("zevs_socket_pair_receive") {
    auto context = zevs::GetCoreFactory()->CreateContext();

    Peer<A> obj;
    A received_a;
    obj.handler = [&](A a) {
      received_a = a;
      obj.event_loop->Exit();
    };

    auto obj_run = [&]() {
      obj.event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      obj.socket     = zevs::GetFactory()->CreatePairSocket(*obj.event_loop);
      obj.socket->Bind("inproc://#1");
      obj.socket->Serve(&Peer<A>::OnEvent, &obj);
      obj.event_loop->Run();
    };

    auto finished = std::async(obj_run);

    auto socket = zevs::GetFactory()->CreatePairSocket();
    socket->Connect("inproc://#1");
    socket->Send(A{TESTVAL});

    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);
    CHECK_EQ(received_a.i1, TESTVAL);
  }

  // Test that if a socket is deleted there is no message
  // received and no crash
  TEST_CASE("zevs_socket_remove_socket") {
    auto context = zevs::GetCoreFactory()->CreateContext();

    Peer<A> obj;
    obj.PromiseRenew();
    obj.handler = [&](A a) {
      obj.socket.reset();
      obj.promise.set_value();
    };

    auto obj_run = [&]() {
      obj.event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      obj.socket     = zevs::GetFactory()->CreatePairSocket(*obj.event_loop);
      obj.socket->Bind("inproc://#1");
      obj.socket->Serve(&Peer<A>::OnEvent, &obj);
      obj.event_loop->Run();
    };

    auto finished = std::async(obj_run);

    auto socket = zevs::GetFactory()->CreatePairSocket();
    socket->Connect("inproc://#1");
    socket->Send(A{TESTVAL});
    CHECK_EQ(obj.future.wait_for(LONG_WAIT), std::future_status::ready);

    obj.PromiseRenew();
    obj.handler = [&](A a) { obj.promise.set_value(); };
    socket->Send(A{TESTVAL});
    // The following checks that the future is not available, thus
    // nothing was received.
    CHECK_NE(obj.future.wait_for(SHORT_WAIT), std::future_status::ready);

    obj.event_loop->Exit();
    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);
  }

  // Test pub-sub and struct with no fields (C)
  TEST_CASE("zevs_socket_pubsub_receive") {
    auto context = zevs::GetCoreFactory()->CreateContext();

    Peer<C> obj;
    obj.handler = [&](C c) { obj.event_loop->Exit(); };

    auto obj_run = [&]() {
      obj.event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      obj.socket     = zevs::GetFactory()->CreateSubSocket(*obj.event_loop);
      obj.socket->Connect("inproc://#1");
      obj.socket->SetFilter("");
      obj.socket->Serve(&Peer<C>::OnEvent, &obj);
      obj.event_loop->Run();
    };

    auto finished = std::async(obj_run);

    auto socket = zevs::GetFactory()->CreatePubSocket();
    socket->Bind("inproc://#1");

    std::this_thread::sleep_for(SHORT_WAIT);  // pub-sub connect
    socket->SendWithEnvelope("test_topic", C{});

    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);
  }
}

// NOLINTEND(*-magic-numbers)

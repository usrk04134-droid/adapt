#include <cstdint>
#include <string>
#include <thread>
#include <utility>

#include "../zevs_core.h"
#include "../zevs_socket.h"
#include "../zevs_test_support.h"

// The tests in this file represents an example of how an application would
// do unittest for a class which uses zevs.
// The intention is to avoid sending actual zmq messages in the unit test
// which makes the test much easier to understand. Since the test can
// be run in a single thread it is also deterministic.

// NOLINTBEGIN(*-magic-numbers, *-optional-access, *-use-nodiscard)
#include <doctest/doctest.h>

const uint32_t TESTVAL1 = 42;
const int32_t TESTVAL2  = -5;

struct A {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000100 };
  uint32_t u1 = 0;
};

struct B {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000200 };
  int32_t i1 = 0.0;
};

class Messenger {
 public:
  explicit Messenger(std::string endpoint_base) : endpoint_base_(std::move(endpoint_base)) {}
  void OnA(A data) { a_ = data; }

  void TriggerSend() {
    client_socket_->Send(B{TESTVAL2});
    client_socket_->Send(A{TESTVAL1});
    pub_socket_->Send(A{TESTVAL1});
  }

  // ThreadEntry (public) is called directly in unit test
  void ThreadEntry(const std::string& event_loop_name) {
    // create server, client and timer sockets
    event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(event_loop_name);

    server_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
    server_socket_->Bind(endpoint_base_ + "_server");
    server_socket_->Serve(&Messenger::OnA, this);

    client_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
    client_socket_->Connect(endpoint_base_ + "_client");

    pub_socket_ = zevs::GetFactory()->CreatePubSocket();
    pub_socket_->Bind(endpoint_base_ + "_pub");

    event_loop_->Run();
  }

  // These methods are called in production code but are not
  // tested in unit tests (represented by the tests in this file)
  void StartThread(const std::string& event_loop_name) {
    thread_ = std::thread(&Messenger::ThreadEntry, this, event_loop_name);
  }
  void ExitThread() { thread_.join(); }

  auto GetA() const -> A { return a_; }

 private:
  A a_;
  std::string endpoint_base_;
  zevs::EventLoopPtr event_loop_;
  zevs::SocketPtr server_socket_;
  zevs::SocketPtr client_socket_;
  zevs::SocketPtr pub_socket_;
  std::thread thread_;
};

const std::string ENDPOINT_BASE   = "inproc://test";
const std::string EVENT_LOOP_NAME = "Messenger";

TEST_SUITE("zevs_test_support") {
  TEST_CASE("zevs_test_support_get_factory") {
    const zevs::MocketFactory factory;

    auto* core_factory = zevs::GetCoreFactory();
    CHECK_EQ(core_factory, &factory);
  }

  TEST_CASE("zevs_test_support_restore_factory") {
    zevs::CoreFactory* test_p = nullptr;
    {
      const zevs::MocketFactory factory;
      test_p = zevs::GetCoreFactory();
      // ~MocketFactory restores CoreFactory
    }
    CHECK_NE(zevs::GetCoreFactory(), test_p);
  }

  TEST_CASE("zevs_test_support_dispatch") {
    zevs::MocketFactory factory;
    Messenger obj{ENDPOINT_BASE};
    obj.ThreadEntry(EVENT_LOOP_NAME);

    auto server_mocket = factory.GetMocket(zevs::Endpoint::BIND, ENDPOINT_BASE + "_server");

    server_mocket->Dispatch(A{TESTVAL1});
    CHECK_EQ(obj.GetA().u1, TESTVAL1);

    server_mocket->Dispatch(A{77});
    CHECK_EQ(obj.GetA().u1, 77);
  }

  TEST_CASE("zevs_test_support_receive") {
    zevs::MocketFactory factory;
    Messenger obj{ENDPOINT_BASE};
    obj.ThreadEntry(EVENT_LOOP_NAME);

    obj.TriggerSend();

    auto client_mocket = factory.GetMocket(zevs::Endpoint::CONNECT, ENDPOINT_BASE + "_client");
    auto pub_mocket    = factory.GetMocket(zevs::Endpoint::BIND, ENDPOINT_BASE + "_pub");

    auto b_msg = client_mocket->Receive<B>();
    CHECK_EQ(b_msg.value().i1, TESTVAL2);

    auto a_msg = client_mocket->Receive<A>();
    CHECK_EQ(a_msg.value().u1, TESTVAL1);

    auto a_msg2 = pub_mocket->Receive<A>();
    CHECK_EQ(a_msg2.value().u1, TESTVAL1);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access, *-use-nodiscard)

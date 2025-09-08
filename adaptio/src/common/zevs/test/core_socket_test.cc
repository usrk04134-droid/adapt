#include "../zevs_core.h"

// the tests in this file is only on the API level, thus the above headers

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <future>
#include <string>
#include <utility>

// NOLINTBEGIN(*-magic-numbers)
#include <doctest/doctest.h>

const auto LONG_WAIT           = std::chrono::milliseconds(1000);
const uint32_t MESSAGE_ID      = 42;
const std::string TEST_CONTENT = "Some content";

inline auto CreatePackagedMessage(uint32_t message_id, const std::string& content) -> zevs::MessagePtr {
  auto msg = zevs::GetCoreFactory()->CreatePackagedMessage(message_id, content.size());
  std::memcpy(msg->Data(), content.c_str(), msg->Size());
  return msg;
}

inline auto CreateRawMessage(const std::string& content) -> zevs::MessagePtr {
  auto msg = zevs::GetCoreFactory()->CreateRawMessage(content.size());
  std::memcpy(msg->Data(), content.c_str(), msg->Size());
  return msg;
}

TEST_SUITE("zevs_core_socket_tests") {
  TEST_CASE("packaged_message") {
    auto context = zevs::GetCoreFactory()->CreateContext();
    zevs::EventLoopPtr event_loop;
    zevs::CoreSocketPtr bind_socket;

    // The lambdas will be called in the std::async thread.
    zevs::MessagePtr received_msg;
    auto handler = [&](zevs::MessagePtr msg) {
      received_msg = std::move(msg);
      event_loop->Exit();
    };

    auto obj_run = [&]() {
      event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      bind_socket =
          zevs::GetCoreFactory()->CreateCoreSocket(*event_loop, zevs::SocketType::PAIR, zevs::MessageType::PACKAGED);
      bind_socket->Bind("inproc://#1");
      bind_socket->SetHandler(handler);
      event_loop->Run();
    };

    auto finished = std::async(obj_run);

    // Set up the connecting side, send/receive a message and check the result
    auto connect_socket = zevs::GetCoreFactory()->CreateCoreSocket(zevs::SocketType::PAIR, zevs::MessageType::PACKAGED);
    connect_socket->Connect("inproc://#1");

    auto test_msg = CreatePackagedMessage(MESSAGE_ID, TEST_CONTENT);
    connect_socket->Send(std::move(test_msg));

    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);
    CHECK_EQ(received_msg->Id(), MESSAGE_ID);
    CHECK_EQ(received_msg->Size(), TEST_CONTENT.size());
    CHECK_EQ(received_msg->Type(), zevs::MessageType::PACKAGED);
    CHECK_EQ(std::string{static_cast<char*>(received_msg->Data()), received_msg->Size()}, TEST_CONTENT);
  }

  TEST_CASE("raw_message") {
    auto context = zevs::GetCoreFactory()->CreateContext();
    zevs::EventLoopPtr event_loop;
    zevs::CoreSocketPtr bind_socket;

    zevs::MessagePtr received_msg;
    auto handler = [&](zevs::MessagePtr msg) {
      received_msg = std::move(msg);
      event_loop->Exit();
    };

    auto obj_run = [&]() {
      event_loop = zevs::GetCoreFactory()->CreateEventLoop("Test");
      bind_socket =
          zevs::GetCoreFactory()->CreateCoreSocket(*event_loop, zevs::SocketType::PAIR, zevs::MessageType::RAW);
      bind_socket->Bind("inproc://#1");
      bind_socket->SetHandler(handler);
      event_loop->Run();
    };

    auto finished = std::async(obj_run);

    // Set up the connecting side, send/receive a message and check the result
    auto connect_socket = zevs::GetCoreFactory()->CreateCoreSocket(zevs::SocketType::PAIR, zevs::MessageType::RAW);
    connect_socket->Connect("inproc://#1");

    auto test_msg = CreateRawMessage(TEST_CONTENT);
    connect_socket->Send(std::move(test_msg));

    CHECK_EQ(finished.wait_for(LONG_WAIT), std::future_status::ready);
    CHECK_EQ(received_msg->Size(), TEST_CONTENT.size());
    CHECK_EQ(received_msg->Type(), zevs::MessageType::RAW);
    CHECK_EQ(std::string{static_cast<char*>(received_msg->Data()), received_msg->Size()}, TEST_CONTENT);
  }
}

// NOLINTEND(*-magic-numbers)

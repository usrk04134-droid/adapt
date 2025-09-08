#include <doctest/doctest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

#include "../zevs_core.h"
#include "../zevs_test_support.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, *-use-nodiscard)

const std::string ENDPOINT_NAME = "inproc://test";
const uint32_t MESSAGE_ID       = 42;
const std::string TEST_CONTENT  = "Some content";

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

TEST_SUITE("zevs_test_support_message_tests") {
  TEST_CASE("packaged_dispatch") {
    zevs::MocketFactory factory;

    // "Application side"
    zevs::MessagePtr received_msg;
    auto handler = [&](zevs::MessagePtr msg) { received_msg = std::move(msg); };

    zevs::EventLoopPtr empty_event_loop;
    auto bind_socket = zevs::GetCoreFactory()->CreateCoreSocket(*empty_event_loop, zevs::SocketType::PAIR,
                                                                zevs::MessageType::PACKAGED);
    bind_socket->Bind(ENDPOINT_NAME);
    bind_socket->SetHandler(handler);

    // "Test support side"
    auto mocket = factory.GetMocket(zevs::Endpoint::BIND, ENDPOINT_NAME);

    auto test_msg = CreatePackagedMessage(MESSAGE_ID, TEST_CONTENT);
    mocket->DispatchMessage(std::move(test_msg));

    // Asserts
    CHECK_EQ(received_msg->Id(), MESSAGE_ID);
    CHECK_EQ(received_msg->Size(), TEST_CONTENT.size());
    CHECK_EQ(received_msg->Type(), zevs::MessageType::PACKAGED);
    CHECK_EQ(std::string{static_cast<char*>(received_msg->Data()), received_msg->Size()}, TEST_CONTENT);
  }

  TEST_CASE("packaged_receive") {
    zevs::MocketFactory factory;

    // "Application side"
    auto bind_socket = zevs::GetCoreFactory()->CreateCoreSocket(zevs::SocketType::PAIR, zevs::MessageType::PACKAGED);
    bind_socket->Bind(ENDPOINT_NAME);
    auto test_msg = CreatePackagedMessage(MESSAGE_ID, TEST_CONTENT);
    bind_socket->Send(std::move(test_msg));

    // "Test support side"
    auto mocket       = factory.GetMocket(zevs::Endpoint::BIND, ENDPOINT_NAME);
    auto received_msg = mocket->ReceiveMessage();

    // Asserts
    CHECK_EQ(received_msg->Id(), MESSAGE_ID);
    CHECK_EQ(received_msg->Size(), TEST_CONTENT.size());
    CHECK_EQ(received_msg->Type(), zevs::MessageType::PACKAGED);
    CHECK_EQ(std::string{static_cast<char*>(received_msg->Data()), received_msg->Size()}, TEST_CONTENT);
  }

  TEST_CASE("raw_dispatch") {
    zevs::MocketFactory factory;

    // "Application side"
    zevs::MessagePtr received_msg;
    auto handler = [&](zevs::MessagePtr msg) { received_msg = std::move(msg); };

    zevs::EventLoopPtr empty_event_loop;
    auto bind_socket =
        zevs::GetCoreFactory()->CreateCoreSocket(*empty_event_loop, zevs::SocketType::PAIR, zevs::MessageType::RAW);
    bind_socket->Bind(ENDPOINT_NAME);
    bind_socket->SetHandler(handler);

    // "Test support side"
    auto mocket = factory.GetMocket(zevs::Endpoint::BIND, ENDPOINT_NAME);

    auto test_msg = CreateRawMessage(TEST_CONTENT);
    mocket->DispatchMessage(std::move(test_msg));

    // Asserts
    CHECK_EQ(received_msg->Size(), TEST_CONTENT.size());
    CHECK_EQ(received_msg->Type(), zevs::MessageType::RAW);
    CHECK_EQ(std::string{static_cast<char*>(received_msg->Data()), received_msg->Size()}, TEST_CONTENT);
  }

  TEST_CASE("raw_receive") {
    zevs::MocketFactory factory;

    // "Application side"
    auto bind_socket = zevs::GetCoreFactory()->CreateCoreSocket(zevs::SocketType::PAIR, zevs::MessageType::RAW);
    bind_socket->Bind(ENDPOINT_NAME);
    auto test_msg = CreateRawMessage(TEST_CONTENT);
    bind_socket->Send(std::move(test_msg));

    // "Test support side"
    auto mocket       = factory.GetMocket(zevs::Endpoint::BIND, ENDPOINT_NAME);
    auto received_msg = mocket->ReceiveMessage();

    // Asserts
    CHECK_EQ(received_msg->Size(), TEST_CONTENT.size());
    CHECK_EQ(received_msg->Type(), zevs::MessageType::RAW);
    CHECK_EQ(std::string{static_cast<char*>(received_msg->Data()), received_msg->Size()}, TEST_CONTENT);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access, *-use-nodiscard)

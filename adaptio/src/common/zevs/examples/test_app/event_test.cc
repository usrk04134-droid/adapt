#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "../../zevs_core.h"
#include "common/logging/application_log.h"
#include "startup_timer.h"
#include "test_msg_seq_pair.h"
#include "test_pair.h"
#include "test_publisher.h"
#include "test_subscriber.h"
#include "test_timer.h"

void ScenarioPubSub() {
  std::cout << "\n******** scenario_pub_sub **********\n";

  auto context = zevs::GetCoreFactory()->CreateContext();

  auto test_publisher = std::make_unique<TestPublisher>();
  test_publisher->Run();

  auto test_subscriber = std::make_unique<TestSubscriber>("EventLoop1");
  test_subscriber->Run();

  test_publisher.reset();  // destructor calls join

  zevs::ExitEventLoop("EventLoop1");

  test_subscriber.reset();  // destructor calls join
  std::cout << "scenario_pub_sub() all threads joined\n";
}

void ScenarioPair() {
  std::cout << "\n******** scenario_pair **********\n";

  auto context = zevs::GetCoreFactory()->CreateContext();

  size_t nr_of_messages = 10;

  auto peer2 = std::make_unique<Peer>(nr_of_messages);
  peer2->Run("ConnectSide");

  auto peer1 = std::make_unique<Peer>(nr_of_messages);
  peer1->Run("BindSide");

  peer1.reset();  // destructor calls join
  peer2.reset();  // destructor calls join
  std::cout << "scenario_pair() all threads joined\n";
}

void ScenarioMsgSeq() {
  std::cout << "\n******** scenario_msg_seq **********\n";

  auto context = zevs::GetCoreFactory()->CreateContext();

  auto both = std::make_unique<BothPeers>();
  both->Run("BothPeers");

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  zevs::ExitEventLoop("BothPeers");

  both.reset();  // destructor calls join
  std::cout << "scenario_msg_seq() all threads joined\n";
}

void TestOfTime() {
  std::cout << "\n******** TestOfTime **********\n";
  // Run it in the main thread for a change

  auto context = zevs::GetCoreFactory()->CreateContext();

  auto time_tester = std::make_unique<TimeTester>();
  time_tester->ThreadEntry("TimeTester");

  std::cout << "TestOfTime() done\n";
}

void TestStartupTimer() {
  std::cout << "\n******** TestStartupTimer **********\n";

  auto context = zevs::GetCoreFactory()->CreateContext();

  auto time_tester = std::make_unique<StartupTimer>();
  time_tester->ThreadEntry("StartupTimer");

  std::cout << "TestStartupTimer() done\n";
}

struct D {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x01000003 };
  uint32_t i1 = 0;
  std::string s1;  // to make non trivial
};

auto main() -> int {
  common::logging::InitLogging();
  common::logging::SetLogLevel(2);  // debug

  ScenarioPubSub();
  ScenarioPair();
  ScenarioMsgSeq();
  TestOfTime();
  TestStartupTimer();
}

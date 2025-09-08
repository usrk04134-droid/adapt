#include <iostream>
#include <memory>

#include "../../zevs_core.h"
#include "common/logging/application_log.h"
#include "messenger.h"

void Test() {
  std::cout << "\n******** starting test **********\n";

  auto context = zevs::GetCoreFactory()->CreateContext();

  auto* ext_if_endpoint = "tcp://127.0.0.1:5555";

  auto messenger = std::make_unique<Messenger>(ext_if_endpoint);
  messenger->Run("Messenger");

  // destructor calls join
  messenger.reset();
  std::cout << "test() all threads joined\n";
}

auto main() -> int {
  common::logging::InitLogging();
  common::logging::SetLogLevel(3);  // trace
  Test();
}

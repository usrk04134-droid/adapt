#include <signal.h>

#include <iostream>
#include <memory>

#include "../../zevs_core.h"
#include "common/logging/application_log.h"
#include "messenger.h"

void Exit(int signum) { zevs::ExitEventLoop("Messenger"); }

// Exit this program with ctrl-c
auto main() -> int {
  signal(SIGINT, Exit);
  signal(SIGTERM, Exit);
  signal(SIGHUP, Exit);

  common::logging::InitLogging();
  common::logging::SetLogLevel(2);

  auto context   = zevs::GetCoreFactory()->CreateContext();
  auto messenger = std::make_unique<Messenger>();

  messenger->ThreadEntry("Messenger");
  std::cout << "\n******** exit **********\n";
  return 0;
}

#include <chrono>
#include <cstddef>
#include <iostream>
#include <memory>

#include "../../zevs_core.h"
#include "common/logging/application_log.h"
#include "messenger.h"

void PerfTest() {
  std::cout << "\n******** perf_test **********\n";

  auto context = zevs::GetCoreFactory()->CreateContext();

  size_t nr_of_messages = 100000;  // total in both directions

  auto messenger2 = std::make_unique<Messenger>(nr_of_messages / 2);
  messenger2->Run("ConnectSide");

  auto messenger1 = std::make_unique<Messenger>(nr_of_messages / 2);
  messenger1->Run("BindSide");

  auto begin = std::chrono::high_resolution_clock::now();
  // destructors call join
  messenger1.reset();
  messenger2.reset();

  auto end     = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);

  std::cout << "Processed " << nr_of_messages << " messages in " << elapsed.count() << " milliseconds.\n";
}

auto main() -> int {
  common::logging::InitLogging();
  common::logging::SetLogLevel(0);

  PerfTest();
}

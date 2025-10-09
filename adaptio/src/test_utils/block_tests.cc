#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_OPTIONS_PREFIX "doctest-"

#include <doctest/doctest.h>
#include <fmt/core.h>
#include "metrics.h"

#include <string>

#include "common/logging/application_log.h"
#include "test_utils/reporters.h"

auto const REPORTER_CONSOLE_EXT = "consoleExt";
REGISTER_REPORTER(REPORTER_CONSOLE_EXT, 1, ConsoleExtReporter);

auto main(int argc, char** argv) -> int {
  doctest::Context context;

  common::logging::InitLogging();
  common::logging::SetLogLevel(1);  // INFO

  auto reporters_set_by_caller = false;

  for (int i = 0; i < argc; i++) {
    if (std::string arg(argv[i]); arg.find("reporters") != std::string::npos) {
      LOG_INFO("Reporters overridden by caller: {}", arg);
      reporters_set_by_caller = true;
      break;
    }
  }

  if (!reporters_set_by_caller) {
    std::string reporters = fmt::format("console,{}", REPORTER_CONSOLE_EXT);
    LOG_INFO("Setting default reporters: {}", reporters);
    context.addFilter("reporters", reporters.c_str());
  }

  context.applyCommandLine(argc, argv);

  // Metrics are initialized when the test Application starts, but in case
  // some tests do not start it, keep reporter hooks safe by lazy init to null.
  // No-op here; initialization happens in helpers.cc once registry exists.

  auto const res = context.run();

  common::logging::DeinitLogging();

  return res;
}

// NOLINTNEXTLINE
#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include "core/logging/application_log.h"

auto main(int argc, char* argv[]) -> int {
  core::logging::InitLogging();
  core::logging::SetLogLevel(-1);

#ifndef DOCTEST_CONFIG_DISABLE
  // Setup & run doctest
  doctest::Context ctx;
  ctx.setOption("abort-after", 100);  // NOLINT
  ctx.applyCommandLine(argc, argv);
  ctx.setOption("no-breaks", true);

  int const res = ctx.run();

  if (ctx.shouldExit()) {
    return res;
  }
#endif
  return 0;
}

#pragma once

#ifdef TESTLOG_DISABLED
#define TESTLOG(...)
#else
#include <fmt/core.h>

#include <filesystem>
#include <iostream>

const char* const CYAN  = "\033[36m";
const char* const RESET = "\033[0m";

#define TESTLOG(...)                                                                                         \
  std::cout << "[" << CYAN << "TEST " << RESET << std::filesystem::path(__FILE__).filename().string() << ":" \
            << std::to_string(__LINE__) << "] " << fmt::format(__VA_ARGS__) << '\n'                          \
            << std::flush;
#define TESTLOG_NOHDR(...) std::cout << fmt::format(__VA_ARGS__) << '\n' << std::flush;

#endif

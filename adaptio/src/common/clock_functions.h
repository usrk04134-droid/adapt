#pragma once
#include <chrono>
#include <functional>

namespace clock_functions {

using SystemClockNowFunc = std::function<std::chrono::time_point<std::chrono::system_clock>()>;
using SteadyClockNowFunc = std::function<std::chrono::time_point<std::chrono::steady_clock>()>;

}  // namespace clock_functions


#include "gaussian_filter_benchmark.h"
#include "sqlite_benchmark.h"

auto main() -> int {
  SqlitePerformanceTest();
  GaussianFilterTest();
}

#include "metrics.h"

#include <mutex>
#include <optional>
#include <prometheus/counter.h>
#include <prometheus/registry.h>

namespace test_metrics {

namespace {
struct MetricsState {
  prometheus::Counter* pass_total{};
  prometheus::Counter* fail_total{};
};

std::once_flag init_once;
std::optional<MetricsState> state;
}

void Initialize(prometheus::Registry* registry) {
  std::call_once(init_once, [registry]() {
    MetricsState s{};
    auto& pass_family = prometheus::BuildCounter()
                            .Name("block_tests_pass_total")
                            .Help("Total number of passing block test assertions")
                            .Register(*registry);
    s.pass_total = &pass_family.Add({});

    auto& fail_family = prometheus::BuildCounter()
                            .Name("block_tests_fail_total")
                            .Help("Total number of failing block test assertions")
                            .Register(*registry);
    s.fail_total = &fail_family.Add({});

    state = s;
  });
}

void IncPass(const std::string& /*suite*/, const std::string& /*test_case*/) {
  if (state && (*state).pass_total) {
    (*state).pass_total->Increment();
  }
}

void IncFail(const std::string& /*suite*/, const std::string& /*test_case*/) {
  if (state && (*state).fail_total) {
    (*state).fail_total->Increment();
  }
}

void AddPasses(size_t count) {
  if (state && (*state).pass_total) {
    (*state).pass_total->Increment(static_cast<double>(count));
  }
}

void AddFails(size_t count) {
  if (state && (*state).fail_total) {
    (*state).fail_total->Increment(static_cast<double>(count));
  }
}

}  // namespace test_metrics

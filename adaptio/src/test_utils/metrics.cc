#include "metrics.h"

#include <mutex>
#include <optional>
#include <atomic>
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
std::atomic<size_t> buffered_pass{0};
std::atomic<size_t> buffered_fail{0};
std::function<void()> push_handler;
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

    // Flush any buffered counts collected before initialization
    const size_t bp = buffered_pass.exchange(0);
    const size_t bf = buffered_fail.exchange(0);
    if (bp > 0) s.pass_total->Increment(static_cast<double>(bp));
    if (bf > 0) s.fail_total->Increment(static_cast<double>(bf));
  });
}

void IncPass(const std::string& /*suite*/, const std::string& /*test_case*/) {
  if (state && (*state).pass_total) { (*state).pass_total->Increment(); }
  else { buffered_pass.fetch_add(1, std::memory_order_relaxed); }
}

void IncFail(const std::string& /*suite*/, const std::string& /*test_case*/) {
  if (state && (*state).fail_total) { (*state).fail_total->Increment(); }
  else { buffered_fail.fetch_add(1, std::memory_order_relaxed); }
}

void AddPasses(size_t count) {
  if (state && (*state).pass_total) { (*state).pass_total->Increment(static_cast<double>(count)); }
  else { buffered_pass.fetch_add(count, std::memory_order_relaxed); }
}

void AddFails(size_t count) {
  if (state && (*state).fail_total) { (*state).fail_total->Increment(static_cast<double>(count)); }
  else { buffered_fail.fetch_add(count, std::memory_order_relaxed); }
}

void SetPushHandler(const std::function<void()>& handler) { push_handler = handler; }
void FlushPush() {
  if (push_handler) {
    try { push_handler(); } catch (...) { /* swallow push errors in tests */ }
  }
}

}  // namespace test_metrics

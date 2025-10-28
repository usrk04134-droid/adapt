#pragma once

#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include <functional>
#include <memory>
#include <string>

namespace test_metrics {

// Lazily initialized global metrics for block tests.
// Safe to call from test reporter even if not yet initialized (no-ops).
void Initialize(prometheus::Registry* registry);
void IncPass(const std::string& suite, const std::string& test_case);
void IncFail(const std::string& suite, const std::string& test_case);
void AddPasses(size_t count);
void AddFails(size_t count);
void SetPushHandler(const std::function<void()>& handler);
void FlushPush();

}  // namespace test_metrics

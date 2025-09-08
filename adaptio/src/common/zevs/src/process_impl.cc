#include "process_impl.h"

#include <zmq.hpp>

#include "common/logging/application_log.h"

namespace zevs::adapter {
auto GetGlobalContext() -> zmq::context_t * { return zevs::GetGlobalContext(); }
}  // namespace zevs::adapter

namespace zevs {

zmq::context_t *s_ctx = nullptr;

ContextImpl::ContextImpl() {
  // One context is shared in the process
  if (s_ctx) {
    LOG_ERROR("ContextImpl(): Context already exists!");
    return;
  }
  s_ctx = &ctx_;
}

ContextImpl::~ContextImpl() { s_ctx = nullptr; }

auto GetGlobalContext() -> zmq::context_t * { return s_ctx; }

}  // namespace zevs

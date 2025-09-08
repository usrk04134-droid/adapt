#pragma once

#include <zmq.hpp>

#include "../zevs_core.h"

namespace zevs {

auto GetGlobalContext() -> zmq::context_t*;

class ContextImpl : public Context {
 public:
  ContextImpl();
  ~ContextImpl() override;

  // noncopyable
  ContextImpl(const ContextImpl&)                    = delete;
  auto operator=(const ContextImpl&) -> ContextImpl& = delete;

 private:
  zmq::context_t ctx_;
};

}  // namespace zevs

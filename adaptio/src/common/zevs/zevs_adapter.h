#pragma once

#include "zmq.hpp"

// This header exposes the zmq header.
// The content here is intended for use where zmq sockets
// are used together with zevs classes.

namespace zevs::adapter {

auto getGlobalContext() -> zmq::context_t*;

}  // namespace zevs::adapter

#pragma once

#include <memory>

namespace controller {

class System {
 public:
  virtual ~System() = default;

  virtual void Apply() = 0;
};

using SystemPtr = std::unique_ptr<System>;

}  // namespace controller

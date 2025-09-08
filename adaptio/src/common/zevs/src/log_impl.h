#pragma once

#include <cstdint>
#include <string>

#include "../zevs_core.h"
#include "common/logging/application_log.h"

namespace zevs {

class LoggingImpl : public Logging {
 public:
  void LogInfo(const std::string &source, const std::string &msg, uint32_t number) override {
    LOG_INFO("{} {} {:#x}", source, msg, number);
  }
  void LogError(const std::string &source, const std::string &msg, uint32_t number) override {
    LOG_ERROR("{} {} {:#x}", source, msg, number);
  }
};

}  // namespace zevs

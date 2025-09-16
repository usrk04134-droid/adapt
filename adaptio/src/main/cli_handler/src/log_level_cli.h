#pragma once

#include <string>

#include "web_hmi/web_hmi.h"

namespace cli_handler {

class LogLevelCli {
 public:
  explicit LogLevelCli(web_hmi::WebHmi* web_hmi, int loglevel);

 private:
  auto SetLoglevel(const std::string& loglevel) -> bool;

  web_hmi::WebHmi* web_hmi_;
  int loglevel_ = -1;
};

}  // namespace cli_handler


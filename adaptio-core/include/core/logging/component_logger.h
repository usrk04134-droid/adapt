#pragma once

#include <boost/log/sinks.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <filesystem>
#include <string>

namespace core::logging {

/*
  The ComponentLogger logs to file using boost's logging framework. The logs
  will not be displayed in the console or in the application log. Below are
  two examples of how the ComponentLogger can be configured.

  Note: ComponentLogger instance should be used from a single thread.

  - timestamped log files
    config = {
        .component      = "abc",
        .path_directory = "log/abc",
        .file_name      = "%Y%m%d_%H%M%S.log",
        .max_file_size  = 25 * 1024 * 1024,
        .max_nb_files   = 10
    };

  - single log file
    config = {
        .component      = "def",
        .path_directory = "log/def",
        .file_name      = "def.log",
        .max_file_size  = 25 * 1024 * 1024,
        .max_nb_files   = 1,
    };
*/

struct ComponentLoggerConfig {
  std::string component;
  std::filesystem::path path_directory;
  std::filesystem::path file_name;
  int max_file_size{};
  int max_nb_files{};
};

class ComponentLogger {
 public:
  explicit ComponentLogger() = default;
  explicit ComponentLogger(const ComponentLoggerConfig &config);
  void Log(const std::string &message);

 private:
  ComponentLoggerConfig config_;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> logger_;
};

}  // namespace core::logging

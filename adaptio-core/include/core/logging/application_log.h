#pragma once

#include <fmt/chrono.h>
#include <fmt/format.h>

#include <boost/core/null_deleter.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_multifile_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "core/time/format.h"

namespace core::logging {

const int DEFAULT_LOG_FILE_SIZE = 10 * 1024 * 1024;  // 10 mb
const std::string COMPONENT     = "component";
const std::string NO_CONSOLE    = "noConsole";

inline void Log(std::string message, boost::log::trivial::severity_level severity) {
  using boost::log::sources::severity_logger;
  using boost::log::trivial::severity_level;

  severity_logger<severity_level> logger;
  BOOST_LOG_SEV(logger, severity) << message;
}

inline void LogFormatter(boost::log::record_view const &rec, boost::log::formatting_ostream &os) {
  namespace expr = boost::log::expressions;

  auto const now    = std::chrono::system_clock::now();
  auto const str_tz = core::time::TimePointTimeZoneToString(now);

  // Timestamp with RFC3339 format e.g. 2025-06-05T10:36:28.312882+02:00
  (expr::stream << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%dT%H:%M:%S.%f" + str_tz))(rec,
                                                                                                                   os);
  os << " | ";

  // Severity
  os << "[" << rec[boost::log::trivial::severity] << "] ";

  // Message
  os << "- " << rec[expr::smessage];
}

inline void InitLogging(std::filesystem::path log_path = "", int log_file_size = DEFAULT_LOG_FILE_SIZE) {
  if (log_path.empty()) {
    auto *home           = std::getenv("HOME");
    auto *xdg_cache_home = std::getenv("XDG_CACHE_HOME");
    if (xdg_cache_home != nullptr && !std::string(xdg_cache_home).empty()) {
      log_path = std::filesystem::path(xdg_cache_home) / "adaptio";
    } else if (home != nullptr && !std::string(home).empty()) {
      log_path = std::filesystem::path(home) / ".cache" / "adaptio";
    } else {
      log_path = std::filesystem::path("/var/log/adaptio/");
    }
  }
  std::filesystem::path const file_name(log_path / "ADAPTIO.log");
  std::filesystem::path const target_file_name(log_path / "ADAPTIO_%Y%m%d_%H%M%S.log");

  std::filesystem::create_directories(std::filesystem::path(log_path));

  /* store existing ADAPTIO.log file with a new name */
  if (std::filesystem::exists(file_name)) {
    try {
      auto const now           = std::chrono::system_clock::now();
      auto const str_timestamp = core::time::TimePointToString(now, core::time::FMT_PATH_SEC);

      std::filesystem::path const prev_file_name(log_path / fmt::format("ADAPTIO_{}_prev.log", str_timestamp));
      std::filesystem::rename(file_name, prev_file_name);
    } catch (const std::filesystem::filesystem_error &ex) {
      std::cerr << "Error archiving existing log file: " << ex.what() << "\n";
    }
  }

  auto core = boost::log::core::get();
  core->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);

  boost::log::add_common_attributes();

  auto console_backend = boost::make_shared<boost::log::sinks::text_ostream_backend>();
  console_backend->add_stream(boost::shared_ptr<std::ostream>(&std::clog, boost::null_deleter()));
  console_backend->auto_flush(true);

  using ConsoleSink = boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend>;

  boost::shared_ptr<ConsoleSink> const console_sink(new ConsoleSink(console_backend));

  console_sink->set_formatter(&LogFormatter);

  /* suppress component logs in the console log */
  console_sink->set_filter(!boost::log::expressions::has_attr(NO_CONSOLE));

  core->add_sink(console_sink);

  auto file_backend = boost::make_shared<boost::log::sinks::text_file_backend>(
      boost::log::keywords::file_name = file_name, boost::log::keywords::target_file_name = target_file_name,
      boost::log::keywords::rotation_size       = log_file_size,
      boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0));

  file_backend->auto_flush(true);

  using FileSink = boost::log::sinks::synchronous_sink<boost::log::sinks::text_file_backend>;

  boost::shared_ptr<FileSink> const file_sink(new FileSink(file_backend));

  file_sink->set_formatter(&LogFormatter);

  /* suppress component logs in the application log files */
  file_sink->set_filter(!boost::log::expressions::has_attr(COMPONENT));

  core->add_sink(file_sink);

  core::logging::Log(
      std::string(__FILE_NAME__)
          .append(":")
          .append(std::to_string(__LINE__).append(" | ").append(fmt::format("Log path set to: {}", log_path.c_str()))),
      boost::log::trivial::info);
}

inline void DeinitLogging() {
  auto core = boost::log::core::get();
  core->flush();
  core->remove_all_sinks();
}

inline void SetLogLevel(int verbosity) {
  auto log_level = boost::log::trivial::warning;

  switch (verbosity) {
    case -1:
      log_level = boost::log::trivial::error;
      break;
    case 0:
      log_level = boost::log::trivial::warning;
      break;
    case 1:
      log_level = boost::log::trivial::info;
      break;
    case 2:
      log_level = boost::log::trivial::debug;
      break;
    case 3:
      log_level = boost::log::trivial::trace;
      break;
    default:
      log_level = boost::log::trivial::trace;
  }

  boost::log::core::get()->set_filter(boost::log::trivial::severity >= log_level);
}

}  // namespace core::logging

#define LOG_TRACE(...)                                                                                     \
  core::logging::Log(std::string(__FILE_NAME__)                                                            \
                         .append(":")                                                                      \
                         .append(std::to_string(__LINE__).append(" | ").append(fmt::format(__VA_ARGS__))), \
                     boost::log::trivial::trace)

#define LOG_DEBUG(...)                                                                                     \
  core::logging::Log(std::string(__FILE_NAME__)                                                            \
                         .append(":")                                                                      \
                         .append(std::to_string(__LINE__).append(" | ").append(fmt::format(__VA_ARGS__))), \
                     boost::log::trivial::debug)

#define LOG_INFO(...)                                                                                      \
  core::logging::Log(std::string(__FILE_NAME__)                                                            \
                         .append(":")                                                                      \
                         .append(std::to_string(__LINE__).append(" | ").append(fmt::format(__VA_ARGS__))), \
                     boost::log::trivial::info)

#define LOG_WARNING(...)                                                                                   \
  core::logging::Log(std::string(__FILE_NAME__)                                                            \
                         .append(":")                                                                      \
                         .append(std::to_string(__LINE__).append(" | ").append(fmt::format(__VA_ARGS__))), \
                     boost::log::trivial::warning)

#define LOG_ERROR(...)                                                                                     \
  core::logging::Log(std::string(__FILE_NAME__)                                                            \
                         .append(":")                                                                      \
                         .append(std::to_string(__LINE__).append(" | ").append(fmt::format(__VA_ARGS__))), \
                     boost::log::trivial::error)

#define LOG_FATAL(...)                                                                                     \
  core::logging::Log(std::string(__FILE_NAME__)                                                            \
                         .append(":")                                                                      \
                         .append(std::to_string(__LINE__).append(" | ").append(fmt::format(__VA_ARGS__))), \
                     boost::log::trivial::fatal)


#include "core/logging/component_logger.h"

#include <boost/log/attributes/constant.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/expressions/attr.hpp>
#include <boost/log/expressions/formatters/format.hpp>
#include <boost/log/expressions/message.hpp>
#include <boost/log/keywords/auto_flush.hpp>
#include <boost/log/keywords/file_name.hpp>
#include <boost/log/keywords/filter.hpp>
#include <boost/log/keywords/format.hpp>
#include <boost/log/keywords/max_size.hpp>
#include <boost/log/keywords/open_mode.hpp>
#include <boost/log/keywords/rotation_size.hpp>
#include <boost/log/keywords/target.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <filesystem>
#include <ios>
#include <string>

#include "core/logging/application_log.h"

using core::logging::ComponentLogger;

ComponentLogger::ComponentLogger(ComponentLoggerConfig const &config) : config_(config) {
  std::filesystem::create_directories(std::filesystem::path(config.path_directory));

  namespace keywords = boost::log::keywords;
  namespace expr     = boost::log::expressions;
  add_file_log(keywords::file_name     = config_.path_directory / config_.file_name,
               keywords::rotation_size = config_.max_file_size,
               keywords::filter        = expr::attr<std::string>(COMPONENT) == config_.component,
               keywords::max_size      = config_.max_nb_files * config_.max_file_size,
               keywords::target = config_.path_directory, keywords::open_mode = std::ios::out | std::ios::app,
               keywords::auto_flush = true, keywords::format = expr::format("%1%") % expr::smessage);

  logger_.add_attribute(NO_CONSOLE, boost::log::attributes::constant<bool>(true));
  logger_.add_attribute(COMPONENT, boost::log::attributes::make_constant(config_.component));
}

void ComponentLogger::Log(const std::string &message) {
  BOOST_LOG_SEV(this->logger_, boost::log::trivial::error) << message.c_str();
}

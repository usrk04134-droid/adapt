
// NOLINTNEXTLINE
#include <bits/chrono.h>
#include <fmt/core.h>
#include <prometheus/exposer.h>
#include <prometheus/registry.h>
#include <signal.h>
#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/SQLiteCpp.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>
#include <unistd.h>

#include <boost/program_options.hpp>
#include <boost/stacktrace/stacktrace.hpp>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "application.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "configuration/config_manager_impl.h"
#include "controller/controller_factory.h"
#include "controller/controller_messenger.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/scanner_application.h"
#include "scanner/scanner_factory.h"
#include "version.h"

namespace po = boost::program_options;

std::string const PATH_DATA_DIR                      = "/var/lib/adaptio";
std::string const PATH_LOGS_DIR                      = "/var/log/adaptio";
std::vector<std::filesystem::path> const PATH_EVENTS = {"/etc/adaptio/events.yaml", "assets/events/events.yaml"};
std::vector<std::filesystem::path> const PATH_SCANNER_CALIBRATION = {"/etc/adaptio/scanner_calibration/",
                                                                     "assets/scanner_calibration/"};
auto const LOG_FILE_SIZE                                          = 256 * 1024 * 1024;
const int PROMETHEUS_PORT                                         = 9091;  // Hardcoded for now

static std::unique_ptr<Application> application;

namespace {
void CrashHandler(int sig) {
  // CONSIDERED CHOICE: We prioritize getting crash information into our logs despite potential risk with allocated
  // memory by std::string, fmt::format, and LOG_ERROR.

  auto stacktrace = boost::stacktrace::stacktrace();

  // Create simple hash fingerprint from frame addresses
  uint32_t hash = 0;
  for (size_t i = 1; i < stacktrace.size(); i++) {
    auto addr  = reinterpret_cast<uint64_t>(stacktrace[i].address());
    hash      ^= static_cast<uint32_t>(addr) ^ static_cast<uint32_t>(addr >> 32);
    hash       = hash * 31 + 17;  // Simple hash mixing
  }

  std::string log_buffer = fmt::format("Signal {} received. Crash fingerprint: 0x{:08x}\nBacktrace:\n", sig, hash);

  for (size_t i = 1; i < stacktrace.size(); i++) {
    log_buffer += fmt::format("#{}: {}\n", i, stacktrace[i].name());
  }

  LOG_ERROR("{}", log_buffer);

  // Restore default handler and re-raise signal for proper exit status and core dump
  signal(sig, SIG_DFL);
  raise(sig);
}

void Exit(int /*signum*/) {
  if (application != nullptr) {
    application->Exit();
  }
}

auto InitDatabase(SQLite::Database* database) {
  try {
    /* synchronous option 'off' gives significant performance gains, especially for inserts, but
     * there is risk for data corruption during certain failure scenarios. keep synchronous setting
     * default=normal for now.
     *
     * more info
     * https://sqlite.org/pragma.html#pragma_synchronous
     * https://sqlite.org/speed.html
     database->exec("PRAGMA synchronous=off"); */

    database->exec("PRAGMA foreign_keys=on");

    int const this_major = ADAPTIO_VERSION_MAJOR;
    int const this_minor = ADAPTIO_VERSION_MINOR;
    int const this_patch = ADAPTIO_VERSION_PATCH;
    int last_major       = 0;
    int last_minor       = 0;
    int last_patch       = 0;

    auto const version_table_name = std::string("aa_version");
    auto const exist              = database->tableExists(version_table_name);
    if (exist) {
      auto const cmd_get = fmt::format("SELECT * FROM {}", version_table_name);
      SQLite::Statement query(*database, cmd_get);
      while (query.executeStep()) {
        last_major = query.getColumn(0).getInt();
        last_minor = query.getColumn(1).getInt();
        last_patch = query.getColumn(2).getInt();
      }
    }

    int const last_version_value = last_major * 1000000 + last_minor * 1000 + last_patch;
    int const this_version_value = this_major * 1000000 + this_minor * 1000 + this_patch;
    auto const upgrade           = this_version_value > last_version_value;
    auto const downgrade         = this_version_value < last_version_value;

    if (!exist) {
      LOG_INFO("No prior verion detected");
      auto const cmd_create =
          fmt::format("CREATE TABLE {} (major INTEGER, minor INTEGER, patch INTEGER)", version_table_name);

      database->exec(cmd_create);
    } else if (upgrade) {
      LOG_INFO("Upgrade from {}.{}.{}", last_major, last_minor, last_patch);
    } else if (downgrade) {
      LOG_INFO("Downgrade from {}.{}.{}", last_major, last_minor, last_patch);
    }

    if (!exist || upgrade || downgrade) {
      auto const cmd_insert = fmt::format("INSERT INTO {} VALUES (?, ?, ?)", version_table_name);
      auto const cmd_update = fmt::format("UPDATE {} SET major=(?), minor=(?), patch=(?)", version_table_name);
      auto query            = SQLite::Statement(*database, exist ? cmd_update : cmd_insert);
      SQLite::bind(query, this_major, this_minor, this_patch);
      query.exec();
    }
  } catch (std::exception& e) {
    LOG_ERROR("SQLite exception: {}", e.what());
    exit(1);
  }
}
}  // namespace

auto main(int argc, char* argv[]) -> int {
  // Register graceful shutdown handlers
  signal(SIGINT, Exit);
  signal(SIGTERM, Exit);
  signal(SIGHUP, Exit);

  // Register crash handlers with backtrace
  signal(SIGSEGV, CrashHandler);
  signal(SIGABRT, CrashHandler);
  signal(SIGFPE, CrashHandler);
  signal(SIGILL, CrashHandler);
  signal(SIGBUS, CrashHandler);

  using std::filesystem::canonical;
  using std::filesystem::path;

  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "Show this help message");
  desc.add_options()("verbose,v", "Sets verbosity to INFO (same as --info)");
  desc.add_options()("info", "Sets verbosity to INFO");
  desc.add_options()("debug", "Sets verbosity to DEBUG");
  desc.add_options()("trace", "Sets verbosity to TRACE");
  desc.add_options()("silent,s", "Disable all output except errors");
  desc.add_options()("config-file", po::value<std::filesystem::path>(), "Sets configuration file");
  desc.add_options()("data-dir,", po::value<std::filesystem::path>()->default_value(PATH_DATA_DIR),
                     "Set path to application data directory for storage of configuration, persistent data etc.");
  desc.add_options()("logs-dir", po::value<std::filesystem::path>()->default_value(PATH_LOGS_DIR),
                     "Set path to application logs directory.");

  po::variables_map map;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).run(), map);
  } catch (const po::error& e) {
    std::cout << fmt::format("Argument error: {}", e.what()) << '\n';
    std::cout << '\n' << desc << '\n';
    exit(1);
  }
  po::notify(map);

  if (map.count("help") > 0 || map.count("h") > 0) {
    std::cout << '\n' << desc << '\n';
    exit(0);
  }

  std::filesystem::path const path_data(map["data-dir"].as<std::filesystem::path>());
  std::filesystem::path const path_logs(map["logs-dir"].as<std::filesystem::path>());

  auto check_dir = [](std::filesystem::path const& path, std::string const& info) {
    if (!std::filesystem::is_directory(path)) {
      fmt::println("ERROR: {} directory at: {} - does not exist!", info, path.string());
      exit(1);
    }

    if (access(path.c_str(), R_OK) != 0) {
      fmt::println("ERROR: {} directory at: {} - user does not have read permissions!", info, path.string());
      exit(1);
    }

    if (access(path.c_str(), W_OK) != 0) {
      fmt::println("ERROR: {} directory at: {} - user does not have write permissions!", info, path.string());
      exit(1);
    }
  };

  check_dir(path_data, "data");
  check_dir(path_logs, "logs");

  common::logging::InitLogging(path_logs, LOG_FILE_SIZE);

  LOG_INFO("Version {} {} build-type: {}", ADAPTIO_VERSION, ADAPTIO_GIT_COMMIT_HASH, BUILD_TYPE);
  LOG_INFO("SQlite3/SqliteC++ version {} / {}", SQLite::getLibVersion(), SQLITECPP_VERSION);

  int log_level = 1;
  if (map.count("silent") > 0 || map.count("s") > 0) {
    log_level = -1;
  }

  if (map.count("verbose") > 0 || map.count("v") > 0 || map.count("info") > 0) {
    log_level = 1;
  }

  if (map.count("debug") > 0) {
    log_level = 2;
  }

  if (map.count("trace") > 0) {
    log_level = 3;
  }

  common::logging::SetLogLevel(log_level);

  // Creating context here so it's not created for DOCTESTS
  auto context = zevs::GetCoreFactory()->CreateContext();

  // Create configuration
  auto default_config = std::filesystem::path("/etc/adaptio/configuration.yaml");
  const auto* path    = std::getenv("ADAPTIO_DEV_CONF");
  if (path != nullptr) {
    default_config = path;
  }

  std::optional<std::filesystem::path> cmd_line_config;
  if (map.count("config-file") > 0) {
    cmd_line_config = map["config-file"].as<std::filesystem::path>();
  }

  auto system_clock_now_func = []() { return std::chrono::system_clock::now(); };
  auto steady_clock_now_func = []() { return std::chrono::steady_clock::now(); };

  /* select first events file that exists */
  auto const path_scanner_calibration = std::invoke(
      [](const std::vector<std::filesystem::path>& paths) -> std::filesystem::path {
        for (auto path : paths) {
          if (std::filesystem::exists(path)) {
            return path;
          }
        }
        LOG_ERROR("No events file found!");
        return "";
      },
      PATH_SCANNER_CALIBRATION);
  auto configuration = std::make_unique<configuration::ConfigManagerImpl>(path_scanner_calibration);

  auto config_res = configuration->Init(default_config, cmd_line_config, path_data);

  if (!config_res) {
    LOG_ERROR("Init of configuration failed: {}", config_res.error().message());
    exit(1);
  }

  auto registry = std::make_shared<prometheus::Registry>();
  auto exposer  = std::make_unique<prometheus::Exposer>("0.0.0.0:" + std::to_string(PROMETHEUS_PORT));
  exposer->RegisterCollectable(registry);

  prometheus::BuildGauge()
      .Name("adaptio_application_info")
      .Help("Information about the Adaptio build")
      .Register(*registry)
      .Add({
          {"version",    ADAPTIO_VERSION        },
          {"hash",       ADAPTIO_GIT_COMMIT_HASH},
          {"build_type", BUILD_TYPE             },
  })
      .Set(1.0);

  auto const start_time_seconds = static_cast<double>(
      std::chrono::duration_cast<std::chrono::seconds>(system_clock_now_func().time_since_epoch()).count());
  prometheus::BuildGauge()
      .Name("adaptio_application_start_time_seconds")
      .Help("Application start time in seconds since epoch")
      .Register(*registry)
      .Add({})
      .Set(start_time_seconds);

  auto controller =
      controller::ControllerFactory::CreateController(configuration->GetController(), steady_clock_now_func);

  auto controller_messenger = std::make_unique<::controller::ControllerMessenger>(
      std::move(controller), configuration->GetController().cycle_time_ms, system_clock_now_func, "adaptio");
  controller_messenger->StartThread("Controller Messenger");

  auto scanner_config = configuration->GetScanner();

  auto ip_config  = configuration->GetImageProvider();
  auto fov_config = ip_config.fov;

  auto image_logger   = scanner::GetFactory()->CreateImageLogger();
  auto image_provider = scanner::GetFactory()->CreateImageProvider(ip_config, registry.get());

  auto const scanner_serial_number = image_provider->GetSerialNumber();
  if (scanner_serial_number.empty() && ip_config.image_provider == scanner::image_provider::ImageProviderType::BASLER) {
    LOG_ERROR("Failed to fetch serial number from scanner!");
    exit(1);
  }

  auto const scanner_calib = configuration->GetScannerCalibration(scanner_serial_number);
  if (!scanner_calib.has_value()) {
    LOG_ERROR("Failed to lookup scanner calibration file for serial number: {}!", scanner_serial_number);
    exit(1);
  }

  auto scanner_server_messenger = std::make_unique<scanner::ScannerApplication>(
      scanner_config, scanner_calib.value(), fov_config, image_provider.get(), "adaptio", path_logs, image_logger.get(),
      registry.get());
  scanner_server_messenger->StartThread("Scanner");

  std::filesystem::path const database_path(path_data / "data.db3");
  LOG_INFO("SQLite database location: {}", database_path.string());
  auto database =
      // NOLINTNEXTLINE(hicpp-signed-bitwise)
      std::make_unique<SQLite::Database>(database_path, SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
  InitDatabase(database.get());

  /* select first events file that exists */
  auto const path_events = std::invoke(
      [](const std::vector<std::filesystem::path>& paths) -> std::filesystem::path {
        for (auto path : paths) {
          if (std::filesystem::exists(path)) {
            return path;
          }
        }
        LOG_ERROR("No events file found!");
        return "";
      },
      PATH_EVENTS);

  application = std::make_unique<Application>(configuration.get(), path_events, database.get(), path_logs,
                                              system_clock_now_func, steady_clock_now_func, registry.get(), log_level);

  LOG_DEBUG("Starting Adaptio");
  auto remote_shutdown = application->Run("Application", "adaptio");

  LOG_INFO("Exiting event loops");
  zevs::ExitEventLoop("Controller Messenger");
  zevs::ExitEventLoop("Scanner");
  zevs::ExitEventLoop("Application");

  LOG_INFO("Exiting controller messenger");
  controller_messenger->JoinThread();
  controller_messenger.reset();

  LOG_INFO("Exiting scanner");
  scanner_server_messenger->JoinThread();
  scanner_server_messenger.reset();

  application.reset();

  common::logging::DeinitLogging();

  if (remote_shutdown) {
    // Preform an Adaptio PC shutdown
    exit(9);
  }
  exit(0);
}

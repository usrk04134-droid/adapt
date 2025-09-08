#pragma once
#include <any>
#include <boost/outcome.hpp>
#include <filesystem>
#include <memory>

#include "common/data/data_value.h"
#include "common/logging/application_log.h"

namespace configuration {

using common::data::DataValue;

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key, uint32_t& ref) {
  if (map.contains(key)) {
    ref = map.at(key).Value<int64_t>().value();
    LOG_TRACE("Update: {} : {}", key, ref);
  }
}

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key, double& ref) {
  if (map.contains(key)) {
    ref = map.at(key).Value<double>().value();
    LOG_TRACE("Update: {} : {}", key, ref);
  }
}

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key, int64_t& ref) {
  if (map.contains(key)) {
    ref = map.at(key).Value<int64_t>().value();
    LOG_TRACE("Update: {} : {}", key, ref);
  }
}

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key, bool& ref) {
  if (map.contains(key)) {
    ref = map.at(key).Value<bool>().value();
    LOG_TRACE("Update: {} : {}", key, ref);
  }
}

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key, std::string& ref) {
  if (map.contains(key)) {
    ref = map.at(key).ToValueString();
    LOG_TRACE("Update: {} : {}", key, ref);
  }
}

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key,
                      std::filesystem::path& ref) {
  if (map.contains(key)) {
    ref = map.at(key).Value<std::string>().value();
    LOG_TRACE("Update: {} : {}", key, ref.string());
  }
}

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key,
                      std::chrono::milliseconds& ref) {
  if (map.contains(key)) {
    auto const value = map.at(key).Value<int64_t>().value();
    ref              = std::chrono::milliseconds(value);
    LOG_TRACE("Update: {} : {} ms", key, ref.count());
  }
}

inline void TryUpdate(const std::unordered_map<std::string, DataValue>& map, const std::string& key,
                      std::chrono::seconds& ref) {
  if (map.contains(key)) {
    auto const value = map.at(key).Value<int64_t>().value();
    ref              = std::chrono::seconds(value);
    LOG_TRACE("Update: {} : {} seconds", key, ref.count());
  }
}

// Interface exposed outside of component
class ConfigurationHandle {
 public:
  virtual ~ConfigurationHandle() = default;
  // Write struct to file
  virtual auto WritePersistentData(std::any config_struct) -> boost::outcome_v2::result<void> = 0;
};

class Converter : public ConfigurationHandle {
 public:
  ~Converter() override = default;

  // Reads from file to struct
  virtual auto ReadPersistentData() -> boost::outcome_v2::result<void> = 0;

  // returns the struct
  virtual auto GetConfig() -> std::any = 0;
};
using ConverterPtr = std::unique_ptr<Converter>;

}  // namespace configuration

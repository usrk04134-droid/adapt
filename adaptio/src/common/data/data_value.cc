#include "common/data/data_value.h"

#include <fmt/core.h>

#include <boost/lexical_cast.hpp>
#include <boost/lexical_cast/bad_lexical_cast.hpp>
#include <boost/log/support/date_time.hpp>
#include <cerrno>
#include <cfloat>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <ios>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

#include "common/logging/application_log.h"

namespace common::data {

DataValue::DataValue(bool value) : value_(value), type_(DataValueType::BOOL) {}

DataValue::DataValue(int16_t value) : value_(static_cast<int64_t>(value)), type_(DataValueType::INT64_T) {}

DataValue::DataValue(int32_t value) : value_(static_cast<int64_t>(value)), type_(DataValueType::INT64_T) {}

DataValue::DataValue(int64_t value) : value_(value), type_(DataValueType::INT64_T) {}

DataValue::DataValue(uint16_t value) : value_(static_cast<uint64_t>(value)), type_(DataValueType::UINT64_T) {}

DataValue::DataValue(uint32_t value) : value_(static_cast<uint64_t>(value)), type_(DataValueType::UINT64_T) {}

DataValue::DataValue(uint64_t value) : value_(value), type_(DataValueType::UINT64_T) {}

DataValue::DataValue(float value) : value_(static_cast<double>(value)), type_(DataValueType::DOUBLE) {}

DataValue::DataValue(double value) : value_(value), type_(DataValueType::DOUBLE) {}

DataValue::DataValue(std::string value) : value_(value), type_(DataValueType::STRING) {}

DataValue::DataValue(Matrix value) : value_(value), type_(DataValueType::MATRIX) {}

auto DataValue::operator<=>(const DataValue& other) const -> int {
  if (type_ == other.type_) {
    switch (type_) {
      case BOOL: {
        auto value       = static_cast<int>(std::get<bool>(value_));
        auto other_value = static_cast<int>(std::get<bool>(other.value_));
        return value - other_value;
      }
      case INT64_T: {
        auto value       = std::get<int64_t>(value_);
        auto other_value = std::get<int64_t>(other.value_);

        if (value < other_value) {
          return -1;
        }

        if (value > other_value) {
          return 1;
        }

        return 0;
      }
      case UINT64_T: {
        auto value       = std::get<uint64_t>(value_);
        auto other_value = std::get<uint64_t>(other.value_);

        if (value < other_value) {
          return -1;
        }

        if (value > other_value) {
          return 1;
        }

        return 0;
      }
      case DOUBLE: {
        // This is not correct, but "good enough".
        auto value       = std::get<double>(value_);
        auto other_value = std::get<double>(other.value_);
        auto difference  = value - other_value;

        if (std::abs(difference) < DBL_EPSILON) {
          return 0;
        }

        if (value < other_value) {
          return -1;
        }

        if (value > other_value) {
          return 1;
        }
      }
      case STRING: {
        auto value       = std::get<std::string>(value_);
        auto other_value = std::get<std::string>(other.value_);
        return value == other_value ? 0 : -1;
      }
      case MATRIX:
        return 0;  // TODO: Implement
    }
  } else {
    throw std::bad_variant_access();
  }
}

auto DataValue::operator==(const DataValue& other) const -> bool { return this->operator<=>(other) == 0; }

auto DataValue::operator!=(const DataValue& other) const -> bool { return this->operator<=>(other) != 0; }

auto DataValue::FromString(const std::string& value_string) -> std::optional<DataValue> {
  const std::regex decimal_regex(R"(^[+\-]?[0-9]*\.[0-9]+(e[+\-]?[0-9]+)?$)");
  const std::regex integer_regex(R"(^[+\-]?[0-9]+$)");
  const std::regex bool_regex(R"(^(true|false)$)", std::regex_constants::icase);

  if (std::regex_match(value_string, decimal_regex)) {  // Probably a double
    try {
      auto value = boost::lexical_cast<double>(value_string);
      return DataValue(value);
    } catch (boost::bad_lexical_cast& e) {
      LOG_WARNING("Failed to convert {} to double (errno = {})", value_string, errno);
      return std::nullopt;
    }
  } else if (std::regex_match(value_string,
                              integer_regex)) {  // Probably an integer
    try {
      auto value = boost::lexical_cast<int64_t>(value_string);
      return DataValue(value);
    } catch (boost::bad_lexical_cast& e) {
      LOG_WARNING("Failed to convert {} to int64_t (errno = {})", value_string, errno);
      return std::nullopt;
    }
  } else if (std::regex_match(value_string, bool_regex)) {  // Probably a bool
    auto value_string_lc = boost::algorithm::to_lower_copy(value_string);

    if (value_string_lc == "true") {
      return DataValue(true);
    }

    return DataValue(false);
  } else {
    return DataValue(value_string);
  }
}

auto DataValue::ToValueString() const -> std::string {
  switch (type_) {
    case DataValueType::BOOL: {
      auto value = Value<bool>();
      if (value.has_value()) {
        return fmt::format("{}", value.value());
      }
    } break;
    case DataValueType::INT64_T: {
      auto value = Value<int64_t>();
      if (value.has_value()) {
        return fmt::format("{}", value.value());
      }
    } break;
    case DataValueType::UINT64_T: {
      auto value = Value<uint64_t>();
      if (value.has_value()) {
        return fmt::format("{}", value.value());
      }
    } break;
    case DataValueType::DOUBLE: {
      auto value = Value<double>();
      if (value.has_value()) {
        std::ostringstream out;
        out << std::scientific << std::setprecision(15) << value.value();
        return std::move(out).str();
      }
    } break;
    case DataValueType::STRING: {
      auto value = Value<std::string>();
      if (value.has_value()) {
        return fmt::format("{}", value.value());
      }
    } break;
    case DataValueType::MATRIX:
      std::stringstream string_stream;

      auto value = Value<Matrix>();

      if (value.has_value()) {
        auto matrix = value.value();

        string_stream << "[ ";
        for (int row = 0; row < matrix.rows; row++) {
          if (row > 0) {
            string_stream << "  ";
          }
          for (int column = 0; column < matrix.columns; column++) {
            string_stream << std::setprecision(15) << matrix.data.get()[(row * matrix.columns) + column];
            if (row != matrix.rows - 1 || column != matrix.columns - 1) {
              string_stream << ", ";
            }
          }

          if (row != matrix.rows - 1) {
            string_stream << '\n';
          }
        }
        string_stream << " ]";

        return string_stream.str();
      }
  }

  return "N/A";  // Should never get here if all branches are covered by the previous switch statement.
}

auto DataValue::ToString() const -> std::string {
  switch (type_) {
    case DataValueType::BOOL:
      return fmt::format("(bool) {}", ToValueString());
    case DataValueType::INT64_T:
      return fmt::format("(int64_t) {}", ToValueString());
    case DataValueType::UINT64_T:
      return fmt::format("(uint64_t) {}", ToValueString());
    case DataValueType::DOUBLE:
      return fmt::format("(double) {}", ToValueString());
    case DataValueType::STRING:
      return fmt::format("(string) {}", ToValueString());
    case DataValueType::MATRIX:
      return fmt::format("(matrix) {}", ToValueString());
  }
}
}  // namespace common::data

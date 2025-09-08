#include "common/data/data_value.h"

#include <doctest/doctest.h>

#include <cerrno>
#include <cfloat>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>

namespace common::data {

TEST_SUITE("DataValue tests") {
  TEST_CASE("Test bool") {
    auto data_value = DataValue(true);
    CHECK_EQ(data_value.Value<bool>().value(), true);
  }

  TEST_CASE("Test int") {
    uint64_t value  = 42;
    auto data_value = DataValue(value);
    CHECK_EQ(data_value.Value<uint64_t>().value(), value);
    CHECK(!data_value.Value<double>().has_value());
  }

  TEST_CASE("Test float") {
    auto value      = 3.1415;
    auto data_value = DataValue(value);
    CHECK(!data_value.Value<int64_t>().has_value());
    CHECK_EQ(data_value.Value<double>().value(), value);
  }

  TEST_CASE("Test string -> double conversion") {
    std::string value_string = "3.1415";

    auto maybe_value = DataValue::FromString(value_string);
    CHECK(maybe_value.has_value());
    CHECK_LT(maybe_value.value().Value<double>().value() - 3.1415, 0.0001);
  }

  TEST_CASE("Test string -> double conversion (eng. notation)") {
    std::string value_string = "3.1415e01";

    auto maybe_value = DataValue::FromString(value_string);
    CHECK(maybe_value.has_value());
    CHECK_LT(maybe_value.value().Value<double>().value() - 31.415, 0.0001);

    value_string = "3.1415e-01";
    maybe_value  = DataValue::FromString(value_string);
    CHECK(maybe_value.has_value());
    CHECK_LT(maybe_value.value().Value<double>().value() - 0.31415, 0.0001);
  }

  TEST_CASE("Test string -> integer conversion") {
    std::string value_string = "42";

    auto maybe_value = DataValue::FromString(value_string);
    CHECK(maybe_value.has_value());
    CHECK_EQ(maybe_value.value().Value<int64_t>().value(), 42);
  }

  TEST_CASE("Test string -> bool conversion") {
    std::string value_string = "true";

    auto maybe_value = DataValue::FromString(value_string);
    CHECK(maybe_value.has_value());
    CHECK_EQ(maybe_value.value().Value<bool>().value(), true);
  }

  TEST_CASE("Test <=>") {
    auto value       = DataValue(4);
    auto other_value = DataValue(45);

    CHECK(value < other_value);
    CHECK(value != other_value);

    value       = DataValue(31.0);
    other_value = DataValue(32.0);

    CHECK(value <= other_value);
    CHECK(value != other_value);

    value       = DataValue(31.0);
    other_value = DataValue(31.0);

    CHECK(value == other_value);

    value       = DataValue((int64_t)0);
    other_value = DataValue((int64_t)0);

    CHECK(value == other_value);
  }
}
}  // namespace common::data

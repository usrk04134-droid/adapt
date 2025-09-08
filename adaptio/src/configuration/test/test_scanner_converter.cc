#include <doctest/doctest.h>

#include <memory>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../config_manager.h"
#include "../scanner_converter.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {
TEST_SUITE("Scanner Converter") {
  TEST_CASE("Convert") {
    std::string yaml = R"(
   scanner:
    filtering:
      gray_minimum_top: 32
      gray_minimum_wall: 16
      gray_minimum_bottom: 8 )";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);

    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    scanner::ScannerConfigurationData ref{};
    ref.gray_minimum_top    = 32;
    ref.gray_minimum_wall   = 16;
    ref.gray_minimum_bottom = 8;

    scanner::ScannerConfigurationData data{};
    CHECK_EQ(ScannerConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data), boost::outcome_v2::success());

    CHECK_EQ(ref.gray_minimum_top, data.gray_minimum_top);
    CHECK_EQ(ref.gray_minimum_bottom, data.gray_minimum_bottom);
    CHECK_EQ(ref.gray_minimum_wall, data.gray_minimum_wall);
  }

  TEST_CASE("Convert - Exception") {
    std::string yaml = R"(
   scanner:
    filtering:
      gray_minimum_top: -nan
      gray_minimum_wall: 16
      gray_minimum_bottom: 8 )";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);

    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    scanner::ScannerConfigurationData data{};
    CHECK_NE(ScannerConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data), boost::outcome_v2::success());
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)

}  // namespace configuration

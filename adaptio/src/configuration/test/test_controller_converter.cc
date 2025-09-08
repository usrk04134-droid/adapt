
#include <doctest/doctest.h>

#include <boost/outcome/result.hpp>
#include <memory>
#include <optional>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep
#include <vector>

#include "../config_manager.h"
#include "../controller_config_converter.h"
#include "controller/controller_configuration.h"
#include "controller/pn_driver/pn_driver.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"

using PnDriverConfiguration = controller::pn_driver::Configuration;

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {

TEST_SUITE("ControllerConverter") {
  TEST_CASE("Convert") {
    std::string yaml = R"(controller:
  type: pn_driver
  cycle_time_ms: 100
  pn_driver:
    rema_path: /adaptio/config/assets/rema.xml
    mac_address: 8c:f3:19:cd:43:ce
    interface: enp0s20f0u1)";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);
    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    controller::ControllerConfigurationData ref{};
    ref.type                          = controller::ControllerType::PN_DRIVER;
    ref.cycle_time_ms                 = 100;
    ref.pn_driver                     = PnDriverConfiguration{};
    ref.pn_driver.value().mac_address = {0x8c, 0xf3, 0x19, 0xcd, 0x43, 0xce};
    ref.pn_driver.value().interface   = "enp0s20f0u1";
    ref.pn_driver.value().rema_path   = "/adaptio/config/assets/rema.xml";

    controller::ControllerConfigurationData data;
    CHECK_EQ(ControllerConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());

    CHECK_EQ(ref.type, data.type);
    CHECK_EQ(ref.cycle_time_ms, data.cycle_time_ms);
    CHECK(ref.pn_driver.has_value());
    CHECK_EQ(ref.pn_driver.value().mac_address, data.pn_driver.value().mac_address);
    CHECK_EQ(ref.pn_driver.value().interface, data.pn_driver.value().interface);
    CHECK_EQ(ref.pn_driver.value().rema_path, data.pn_driver.value().rema_path);
  }

  TEST_CASE("Convert - simulation") {
    std::string yaml = R"(controller:
  type: simulation
  cycle_time_ms: 100
  pn_driver:
    rema_path: assets/rema.xml
    mac_address: 8c:f3:19:cd:43:ce
    interface: enp0s20f0u1)";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);
    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    controller::ControllerConfigurationData ref{};
    ref.type          = controller::ControllerType::SIMULATION;
    ref.cycle_time_ms = 100;

    controller::ControllerConfigurationData data;
    CHECK_EQ(ControllerConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());

    CHECK_EQ(ref.type, data.type);
    CHECK_EQ(ref.cycle_time_ms, data.cycle_time_ms);
    CHECK(!ref.pn_driver.has_value());
  }

  TEST_CASE("Convert - update") {
    std::string yaml = R"(controller:
  type: simulation
  cycle_time_ms: 100
  pn_driver:
    rema_path: assets/rema.xml
    mac_address: 8c:f3:19:cd:43:ce
    interface: enp0s20f0u1)";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);
    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    controller::ControllerConfigurationData ref{};
    ref.type          = controller::ControllerType::SIMULATION;
    ref.cycle_time_ms = 100;

    controller::ControllerConfigurationData data;
    CHECK_EQ(ControllerConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());

    CHECK_EQ(ref.type, data.type);
    CHECK_EQ(ref.cycle_time_ms, data.cycle_time_ms);
    CHECK(!ref.pn_driver.has_value());

    ref.type          = controller::ControllerType::PN_DRIVER;
    ref.cycle_time_ms = 1000;

    std::string update_yaml = R"(controller:
  type: pn_driver
  cycle_time_ms: 1000)";

    maybe_configuration_yaml = common::file::Yaml::FromString(update_yaml, TAG_CONF);
    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    map = maybe_configuration_yaml.value()->AsUnorderedMap();

    CHECK_EQ(ControllerConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());

    CHECK_EQ(ref.type, data.type);
    CHECK_EQ(ref.cycle_time_ms, data.cycle_time_ms);
  }

  TEST_CASE("Hex to byte") {
    std::vector<char> octet = {'0', '0'};
    CHECK_EQ(HexOctetToByte(octet.data()).value(), 0);

    octet = {'0', '1'};
    CHECK_EQ(HexOctetToByte(octet.data()).value(), 1);

    octet = {'1', '1'};
    CHECK_EQ(HexOctetToByte(octet.data()).value(), 17);

    octet = {'F', 'F'};
    CHECK_EQ(HexOctetToByte(octet.data()).value(), 255);

    octet = {'U', 'U'};
    CHECK_EQ(HexOctetToByte(octet.data()), std::nullopt);
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
}  // namespace configuration

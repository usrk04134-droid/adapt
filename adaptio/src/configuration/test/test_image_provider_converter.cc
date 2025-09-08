#include <doctest/doctest.h>

#include <boost/outcome/result.hpp>
#include <memory>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../config_manager.h"
#include "../image_provider_converter.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"
#include "scanner/image_provider/image_provider_configuration.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {

TEST_SUITE("Image Provider Converter") {
  TEST_CASE("Convert") {
    std::string yaml = R"(
    image_provider:
      type: simulation
      fov:
        width: 3500
        height: 2500
        offset_x: 312
        offset_y: 0
      basler:
        gain: 0.0
        exposure_time: 2.000000000000000e+04
      simulation:
        realtime: false
        images: ./test/assets)";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);

    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    scanner::image_provider::ImageProviderConfigData ref{};
    ref.image_provider              = scanner::image_provider::ImageProviderType::SIMULATION;
    ref.fov.width                   = 3500;
    ref.fov.height                  = 2500;
    ref.fov.offset_x                = 312;
    ref.fov.offset_y                = 0;
    ref.basler_config.gain          = 0.0;
    ref.basler_config.exposure_time = 2.000000000000000e+04;
    ref.sim_config.realtime         = false;
    ref.sim_config.images_path      = "/adaptio/config/test/assets";

    scanner::image_provider::ImageProviderConfigData data;
    CHECK_EQ(ImageProviderCoverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());

    CHECK_EQ(ref.image_provider, data.image_provider);
    CHECK_EQ(ref.fov.height, data.fov.height);
    CHECK_EQ(ref.fov.width, data.fov.width);
    CHECK_EQ(ref.fov.offset_x, data.fov.offset_x);
    CHECK_EQ(ref.fov.offset_y, data.fov.offset_y);
    CHECK_EQ(ref.basler_config.exposure_time, data.basler_config.exposure_time);
    CHECK_EQ(ref.basler_config.gain, data.basler_config.gain);
    CHECK_EQ(ref.sim_config.realtime, data.sim_config.realtime);
    CHECK_EQ(ref.sim_config.images_path, data.sim_config.images_path);
  }

  TEST_CASE("Convert -  Exception") {
    std::string update_yaml = R"(
    image_provider:
      type: basler
      fov:
        width: 500
        height: 2500
        offset_x: 312
        offset_y: 10
      basler:
        gain: 2
        exposure_time: 3.140000000000000e+04
      simulation:
        realtime: false
        images: ./)";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(update_yaml, TAG_CONF);
    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto update_map = maybe_configuration_yaml.value()->AsUnorderedMap();

    scanner::image_provider::ImageProviderConfigData ref{};

    scanner::image_provider::ImageProviderConfigData update_data;

    CHECK_NE(ImageProviderCoverter::ToStruct(update_map, "/adaptio/config/configuration.yaml", update_data),
             boost::outcome_v2::success());
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
}  // namespace configuration

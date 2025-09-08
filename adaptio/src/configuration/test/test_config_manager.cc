
#include <doctest/doctest.h>

#include <boost/outcome/result.hpp>
#include <memory>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../config_manager_impl.h"
#include "configuration/conf_factory.h"
#include "controller/controller_configuration.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {

TEST_SUITE("Configuration") {
  TEST_CASE("Read configuration - only default config") {
    std::string default_config = R"(
circular_weld_object_calibration: 2.yaml
laser_torch_calibration: 3.yaml

controller:
  type: pn_driver
  cycle_time_ms: 10

  pn_driver:
    rema_path: /var/lib/adaptio/rema.xml
    interface: profinet1

scanner:
  filtering:
    gray_minimum_top: 32
    gray_minimum_wall: 16
    gray_minimum_bottom: 32

  joint_tolerance:
    upper_width: 10.0
    surface_angle: 0.174532925
    wall_angle: 0.13962634

image_provider:
  type: basler
  fov:
    width: 3500
    height: 2500
    offset_x: 312
    offset_y: 0
  basler:
    gain: 15.0
    exposure_time: 6000.0
  simulation:
    realtime: false
    images: ./

calibration_fixture_joint_geometry:
# Upper joint width in mm
  upper_joint_width: 50.0
  # Groove depth in mm
  groove_depth: 28.0
  # Left joint angle in radians
  left_joint_angle: 0.1396
  # Right joint angle in radians
  right_joint_angle: 0.1396
  # Left max surface angle in radians
  left_max_surface_angle: 0.34906585
  # Right max surface angle in radians
  right_max_surface_angle: 0.34906585)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/configuration.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/var/lib/adaptio/configuration.yaml")).RETURN(false);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/3.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/2.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/1.yaml")).RETURN(true);

    REQUIRE_CALL(*mock_fh, GetAbsParent("/etc/adaptio/configuration.yaml")).RETURN("/etc/adaptio");

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/etc/adaptio/configuration.yaml")).RETURN(default_config);

    ConfigManagerImpl configuration("");
    CHECK_EQ(configuration.Init("/etc/adaptio/configuration.yaml", {}, "/var/lib/adaptio"),
             boost::outcome_v2::success());

    CHECK_EQ(configuration.GetController().type, controller::ControllerType::PN_DRIVER);
    CHECK_EQ(configuration.GetController().cycle_time_ms, 10);

    CHECK_EQ(configuration.GetController().pn_driver->rema_path, "/var/lib/adaptio/rema.xml");
    CHECK_EQ(configuration.GetController().pn_driver->interface, "profinet1");
  }

  TEST_CASE("Read configuration - with user config ") {
    std::string default_config = R"(
    circular_weld_object_calibration: 2.yaml
    laser_torch_calibration: 3.yaml

    controller:
      type: pn_driver
      cycle_time_ms: 10

      pn_driver:
        rema_path: /var/lib/adaptio/rema.xml
        interface: profinet1 

    scanner:
      filtering:
        gray_minimum_top: 32
        gray_minimum_wall: 16
        gray_minimum_bottom: 32

      joint_tolerance:
        upper_width: 10.0
        surface_angle: 0.174532925
        wall_angle: 0.13962634

    image_provider:
      type: basler
      fov:
        width: 3500
        height: 2500
        offset_x: 312
        offset_y: 0
      basler:
        gain: 15.0
        exposure_time: 6000.0
      simulation:
        realtime: false
        images: ./ 
    calibration_fixture_joint_geometry:
      # Upper joint width in mm
      upper_joint_width: 50.0
      # Groove depth in mm
      groove_depth: 28.0
      # Left joint angle in radians
      left_joint_angle: 0.1396
      # Right joint angle in radians
      right_joint_angle: 0.1396
      # Left max surface angle in radians
      left_max_surface_angle: 0.34906585
      # Right max surface angle in radians
      right_max_surface_angle: 0.34906585)";

    std::string user_config = R"(
      controller:
        type: simulation
        cycle_time_ms: 100

      scanner:
        filtering:
          gray_minimum_top: 32
          gray_minimum_wall: 16
          gray_minimum_bottom: 32

        joint_tolerance:
          upper_width: 10.0
          surface_angle: 0.174532925
          wall_angle: 0.13962634

      image_provider:
        type: basler
        fov:
          width: 3500
          height: 2500
          offset_x: 312
          offset_y: 0
        basler:
          gain: 15.0
          exposure_time: 6000.0
        simulation:
          realtime: false
          images: ./

      calibration_fixture_joint_geometry:
        # Upper joint width in mm
        upper_joint_width: 50.0
        # Groove depth in mm
        groove_depth: 28.0
        # Left joint angle in radians
        left_joint_angle: 0.1396
        # Right joint angle in radians
        right_joint_angle: 0.1396
        # Left max surface angle in radians
        left_max_surface_angle: 0.34906585
        # Right max surface angle in radians
        right_max_surface_angle: 0.34906585)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/configuration.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/var/lib/adaptio/configuration.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/3.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/2.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/1.yaml")).RETURN(true);

    REQUIRE_CALL(*mock_fh, GetAbsParent("/etc/adaptio/configuration.yaml")).RETURN("/etc/adaptio");
    REQUIRE_CALL(*mock_fh, GetAbsParent("/var/lib/adaptio/configuration.yaml")).RETURN("/var/lib/adaptio");

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/etc/adaptio/configuration.yaml")).RETURN(default_config);
    REQUIRE_CALL(*mock_fh, ReadFile("/var/lib/adaptio/configuration.yaml")).RETURN(user_config);

    ConfigManagerImpl configuration("");
    CHECK_EQ(configuration.Init("/etc/adaptio/configuration.yaml", {}, "/var/lib/adaptio"),
             boost::outcome_v2::success());

    CHECK_EQ(configuration.GetController().type, controller::ControllerType::SIMULATION);
    CHECK_EQ(configuration.GetController().cycle_time_ms, 100);
  }

  TEST_CASE("Read configuration - with small user config") {
    std::string default_config = R"(
    circular_weld_object_calibration: 2.yaml
    laser_torch_calibration: 3.yaml

    controller:
      type: pn_driver
      cycle_time_ms: 10

      pn_driver:
        rema_path: /var/lib/adaptio/rema.xml
        interface: profinet1 

    scanner:
      filtering:
        gray_minimum_top: 32
        gray_minimum_wall: 16
        gray_minimum_bottom: 32

      joint_tolerance:
        upper_width: 10.0
        surface_angle: 0.174532925
        wall_angle: 0.13962634

    image_provider:
      type: basler
      fov:
        width: 3500
        height: 2500
        offset_x: 312
        offset_y: 0
      basler:
        gain: 15.0
        exposure_time: 6000.0
    calibration_fixture_joint_geometry:
      # Upper joint width in mm
      upper_joint_width: 50.0
      # Groove depth in mm
      groove_depth: 28.0
      # Left joint angle in radians
      left_joint_angle: 0.1396
      # Right joint angle in radians
      right_joint_angle: 0.1396
      # Left max surface angle in radians
      left_max_surface_angle: 0.34906585
      # Right max surface angle in radians
      right_max_surface_angle: 0.34906585)";

    std::string user_config = R"(
    circular_weld_object_calibration: 2.yaml
    laser_torch_calibration: 3.yaml
      )";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/configuration.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/var/lib/adaptio/configuration.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/var/lib/adaptio/3.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/var/lib/adaptio/2.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/var/lib/adaptio/1.yaml")).RETURN(true);

    REQUIRE_CALL(*mock_fh, GetAbsParent("/etc/adaptio/configuration.yaml")).RETURN("/etc/adaptio");
    REQUIRE_CALL(*mock_fh, GetAbsParent("/var/lib/adaptio/configuration.yaml")).RETURN("/var/lib/adaptio");

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/etc/adaptio/configuration.yaml")).RETURN(default_config);
    REQUIRE_CALL(*mock_fh, ReadFile("/var/lib/adaptio/configuration.yaml")).RETURN(user_config);

    ConfigManagerImpl configuration("");
    CHECK_EQ(configuration.Init("/etc/adaptio/configuration.yaml", {}, "/var/lib/adaptio"),
             boost::outcome_v2::success());
  }

  TEST_CASE("Read configuration - with user config cmd line") {
    std::string default_config = R"(
      circular_weld_object_calibration: 2.yaml
      laser_torch_calibration: 3.yaml

      controller:
        type: pn_driver
        cycle_time_ms: 10

        pn_driver:
          rema_path: /var/lib/adaptio/rema.xml
          interface: profinet1
      scanner:
        filtering:
          gray_minimum_top: 32
          gray_minimum_wall: 16
          gray_minimum_bottom: 32

        joint_tolerance:
          upper_width: 10.0
          surface_angle: 0.174532925
          wall_angle: 0.13962634

      image_provider:
        type: basler
        fov:
          width: 3500
          height: 2500
          offset_x: 312
          offset_y: 0
        basler:
          gain: 15.0
          exposure_time: 6000.0
        simulation:
          realtime: false
          images: ./

      calibration_fixture_joint_geometry:
        # Upper joint width in mm
        upper_joint_width: 50.0
        # Groove depth in mm
        groove_depth: 28.0
        # Left joint angle in radians
        left_joint_angle: 0.1396
        # Right joint angle in radians
        right_joint_angle: 0.1396
        # Left max surface angle in radians
        left_max_surface_angle: 0.34906585
        # Right max surface angle in radians
        right_max_surface_angle: 0.34906585)";

    std::string user_config = R"(
        controller:
          type: simulation
          cycle_time_ms: 100

        scanner:
          filtering:
            gray_minimum_top: 32
            gray_minimum_wall: 16
            gray_minimum_bottom: 32

          joint_tolerance:
            upper_width: 10.0
            surface_angle: 0.174532925
            wall_angle: 0.13962634

        image_provider:
          type: basler
          fov:
            width: 3500
            height: 2500
            offset_x: 312
            offset_y: 0
          basler:
            gain: 15.0
            exposure_time: 6000.0
          simulation:
            realtime: false
            images: ./
        calibration_fixture_joint_geometry:
          # Upper joint width in mm
          upper_joint_width: 50.0
          # Groove depth in mm
          groove_depth: 28.0
          # Left joint angle in radians
          left_joint_angle: 0.1396
          # Right joint angle in radians
          right_joint_angle: 0.1396
          # Left max surface angle in radians
          left_max_surface_angle: 0.34906585
          # Right max surface angle in radians
          right_max_surface_angle: 0.34906585)";

    std::string cmd_line_config = R"(
    controller:
      type: simulation
      cycle_time_ms: 1000
    scanner:
      filtering:
        gray_minimum_top: 32
        gray_minimum_wall: 16
        gray_minimum_bottom: 32

      joint_tolerance:
        upper_width: 10.0
        surface_angle: 0.174532925
        wall_angle: 0.13962634

    image_provider:
      type: basler
      fov:
        width: 3500
        height: 2500
        offset_x: 312
        offset_y: 0
      basler:
        gain: 15.0
        exposure_time: 6000.0
      simulation:
        realtime: false
        images: ./

    calibration_fixture_joint_geometry:
      # Upper joint width in mm
      upper_joint_width: 50.0
      # Groove depth in mm
      groove_depth: 28.0
      # Left joint angle in radians
      left_joint_angle: 0.1396
      # Right joint angle in radians
      right_joint_angle: 0.1396
      # Left max surface angle in radians
      left_max_surface_angle: 0.34906585
      # Right max surface angle in radians
      right_max_surface_angle: 0.34906585)";

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/configuration.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/var/lib/adaptio/configuration.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/tmp/configuration.yaml")).RETURN(true);

    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/3.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/2.yaml")).RETURN(true);
    ALLOW_CALL(*mock_fh, FileExist("/etc/adaptio/1.yaml")).RETURN(true);

    REQUIRE_CALL(*mock_fh, GetAbsParent("/etc/adaptio/configuration.yaml")).RETURN("/etc/adaptio");
    REQUIRE_CALL(*mock_fh, GetAbsParent("/var/lib/adaptio/configuration.yaml")).RETURN("/var/lib/adaptio");
    REQUIRE_CALL(*mock_fh, GetAbsParent("/tmp/configuration.yaml")).RETURN("/tmp");

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/etc/adaptio/configuration.yaml")).RETURN(default_config);
    REQUIRE_CALL(*mock_fh, ReadFile("/var/lib/adaptio/configuration.yaml")).RETURN(user_config);
    REQUIRE_CALL(*mock_fh, ReadFile("/tmp/configuration.yaml")).RETURN(cmd_line_config);

    ConfigManagerImpl configuration("");
    CHECK_EQ(configuration.Init("/etc/adaptio/configuration.yaml", "/tmp/configuration.yaml", "/var/lib/adaptio"),
             boost::outcome_v2::success());

    CHECK_EQ(configuration.GetController().type, controller::ControllerType::SIMULATION);
    CHECK_EQ(configuration.GetController().cycle_time_ms, 1000);
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
}  // namespace configuration

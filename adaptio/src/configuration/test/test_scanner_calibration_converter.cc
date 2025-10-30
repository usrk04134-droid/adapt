#include <doctest/doctest.h>

#include <any>
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../config_manager.h"
#include "../configuration_error.h"
#include "../scanner_calibration_converter.h"
#include "configuration/conf_factory.h"
#include "controller/pn_driver/pn_driver.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"
#include "scanner/core/scanner_calibration_configuration.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {

TEST_SUITE("Scanner Calibration Converter") {
  TEST_CASE("Convert") {
    std::string yaml = R"(# 2025-01-30T10:08:25+0000
scanner_serial_number: 40297730
intrinsic:
  projection_center_distance: 0.0
  focus_distance: 4.715378224147283
  principal_point:
    x: 0.9930139449441449
    y: 0.754875302126084
  pixel_pitch:
    x: 2.74e-06
    y: 2.74e-06
  rho: 3.141592653589793
  tau: 0.12217304763960307
  d: 0.5597753718678573
  K1: 0.007962795536909519
  K2: 0.026772094795031075
  K3: -0.014787649308636013
  P1: -0.0028219237254003744
  P2: 0.00417864043329311
  scaling_factors:
    w: 0.00565536
    m: 0.1
    K1: 0.1
    K2: 0.1
    K3: 0.1
    P1: 0.1
    P2: 0.1
extrinsic:
  R: !matrix
    rows: 3
    columns: 3
    data: [0.9999900141669034, -0.0037822314392825704, -0.0023803974071929038,
           -0.0006203294305648313, 0.41001800882613115, -0.9120772158264081,
           0.004425692925864075, 0.9120695846061277, 0.41001156822525336]
  t: !matrix
    rows: 3
    columns: 1
    data: [-0.04886608988697602, -0.04393368249422651, 0.37231035253911315]
  D: !matrix
    rows: 5
    columns: 1
    data: [0.1, 0.01, -0.001, 0, 0]
checksum: 0a37019cc78b67784c5c2f5c435fa113
)";

    scanner::ScannerCalibrationData ref{};

    ref.scanner_serial_number = "40297730";

    ref.intrinsic.projection_center_distance = 0.0;
    ref.intrinsic.focus_distance             = 4.715378224147283;
    ref.intrinsic.principal_point.x          = 0.9930139449441449;
    ref.intrinsic.principal_point.y          = 0.754875302126084;
    ref.intrinsic.pixel_pitch.x              = 2.74e-06;
    ref.intrinsic.pixel_pitch.y              = 2.74e-06;
    ref.intrinsic.rho                        = 3.141592653589793;
    ref.intrinsic.tau                        = 0.12217304763960307;
    ref.intrinsic.d                          = 0.5597753718678573;
    ref.intrinsic.K1                         = 0.007962795536909519;
    ref.intrinsic.K2                         = 0.026772094795031075;
    ref.intrinsic.K3                         = -0.014787649308636013;
    ref.intrinsic.P1                         = -0.0028219237254003744;
    ref.intrinsic.P2                         = 0.00417864043329311;
    ref.intrinsic.scaling_factors.w          = 0.00565536;
    ref.intrinsic.scaling_factors.m          = 0.1;
    ref.intrinsic.scaling_factors.K1         = 0.1;
    ref.intrinsic.scaling_factors.K2         = 0.1;
    ref.intrinsic.scaling_factors.K3         = 0.1;
    ref.intrinsic.scaling_factors.P1         = 0.1;
    ref.intrinsic.scaling_factors.P2         = 0.1;

    Eigen::Matrix3d rotation({
        {0.9999900141669034,     -0.0037822314392825704, -0.0023803974071929038},
        {-0.0006203294305648313, 0.41001800882613115,    -0.9120772158264081   },
        {0.004425692925864075,   0.9120695846061277,     0.41001156822525336   }
    });
    ref.extrinsic.rotation = rotation;

    Eigen::Vector3d translation({-0.04886608988697602, -0.04393368249422651, 0.37231035253911315});
    ref.extrinsic.translation = translation;

    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/scanner_calibration_configuration.yaml")).RETURN(yaml);

    ScannerCalibrationConverter converter(TAG_SC, "/adaptio/config/scanner_calibration_configuration.yaml");
    CHECK_EQ(converter.ReadPersistentData(), boost::outcome_v2::success());
    auto config = std::any_cast<scanner::ScannerCalibrationData>(converter.GetConfig());

    CHECK_EQ(ref.scanner_serial_number, config.scanner_serial_number);

    CHECK_EQ(ref.intrinsic.projection_center_distance, config.intrinsic.projection_center_distance);
    CHECK_EQ(ref.intrinsic.focus_distance, config.intrinsic.focus_distance);
    CHECK_EQ(ref.intrinsic.principal_point.x, config.intrinsic.principal_point.x);
    CHECK_EQ(ref.intrinsic.principal_point.y, config.intrinsic.principal_point.y);
    CHECK_EQ(ref.intrinsic.pixel_pitch.x, config.intrinsic.pixel_pitch.x);
    CHECK_EQ(ref.intrinsic.pixel_pitch.y, config.intrinsic.pixel_pitch.y);

    CHECK_EQ(ref.intrinsic.rho, config.intrinsic.rho);
    CHECK_EQ(ref.intrinsic.tau, config.intrinsic.tau);
    CHECK_EQ(ref.intrinsic.d, config.intrinsic.d);
    CHECK_EQ(ref.intrinsic.K1, config.intrinsic.K1);
    CHECK_EQ(ref.intrinsic.K2, config.intrinsic.K2);
    CHECK_EQ(ref.intrinsic.K3, config.intrinsic.K3);
    CHECK_EQ(ref.intrinsic.P1, config.intrinsic.P1);
    CHECK_EQ(ref.intrinsic.P2, config.intrinsic.P2);

    CHECK_EQ(ref.intrinsic.scaling_factors.w, config.intrinsic.scaling_factors.w);
    CHECK_EQ(ref.intrinsic.scaling_factors.m, config.intrinsic.scaling_factors.m);
    CHECK_EQ(ref.intrinsic.scaling_factors.K1, config.intrinsic.scaling_factors.K1);
    CHECK_EQ(ref.intrinsic.scaling_factors.K2, config.intrinsic.scaling_factors.K2);
    CHECK_EQ(ref.intrinsic.scaling_factors.K3, config.intrinsic.scaling_factors.K3);
    CHECK_EQ(ref.intrinsic.scaling_factors.P1, config.intrinsic.scaling_factors.P1);
    CHECK_EQ(ref.intrinsic.scaling_factors.P2, config.intrinsic.scaling_factors.P2);

    CHECK_EQ(ref.extrinsic.rotation, config.extrinsic.rotation);
    CHECK_EQ(ref.extrinsic.translation, config.extrinsic.translation);
  }
  TEST_CASE("Convert - exception") {
    std::string yaml = R"(scanner_serial_number: 1234
intrinsic:
  projection_center_distance: -nan
)";
    MockFactory mock_factory;
    auto mock = [&]() { return &mock_factory; };
    configuration::SetFactoryGenerator(mock);

    auto mock_fh = std::make_shared<MockFileHander>();

    REQUIRE_CALL(mock_factory, CreateFileHandler()).RETURN(mock_fh);
    REQUIRE_CALL(*mock_fh, ReadFile("/adaptio/config/scanner_calibration_configuration.yaml")).RETURN(yaml);

    ScannerCalibrationConverter converter(TAG_SC, "/adaptio/config/scanner_calibration_configuration.yaml");
    CHECK_EQ(converter.ReadPersistentData(),
             boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR)));
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
}  // namespace configuration

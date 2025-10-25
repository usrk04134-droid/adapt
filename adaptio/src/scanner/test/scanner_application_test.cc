
#include "../scanner_application.h"

#include <doctest/doctest.h>
#include <prometheus/registry.h>

#include <array>
#include <boost/outcome.hpp>
#include <memory>
#include <optional>
#include <trompeloeil.hpp>  // IWYU pragma: keep
#include <trompeloeil/mock.hpp>
#include <utility>

#include "common/groove/groove.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_core.h"
#include "common/zevs/zevs_socket.h"
#include "common/zevs/zevs_test_support.h"
#include "mock/core_scanner_mock.h"
#include "mock/image_logger_mock.h"
#include "mock/stub_core_factory.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/scanner.h"
#include "scanner/scanner_calibration_configuration.h"
#include "scanner/scanner_configuration.h"
#include "scanner/scanner_factory.h"
#include "scanner/slice_provider/slice_provider.h"

using trompeloeil::_;

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)
namespace scanner {

TEST_SUITE("Test Scanner Adapter Scanner") {
  TEST_CASE("Test normal flow") {
    zevs::MocketFactory mocket_factory;

    auto event_loop     = zevs::GetCoreFactory()->CreateEventLoop("mock");
    auto scanner_socket = zevs::GetFactory()->CreatePairSocket(*event_loop);
    scanner_socket->Connect("inproc://mock/scanner");

    auto image_logger_mock_unique_ptr = std::make_unique<ImageLoggerMock>();
    auto* image_logger_mock           = image_logger_mock_unique_ptr.get();

    auto scanner_mock_unique_ptr = std::make_unique<CoreScannerMock>();
    auto* scanner_mock           = scanner_mock_unique_ptr.get();
    StubCoreScannerFactory factory(std::move(scanner_mock_unique_ptr), std::move(image_logger_mock_unique_ptr));

    auto mock = [&]() { return &factory; };
    scanner::SetFactoryGenerator(mock);

    auto registry = std::make_shared<prometheus::Registry>();
    scanner::ScannerApplication scanner_application(ScannerConfigurationData{}, ScannerCalibrationData{},
                                                    image_provider::Fov{}, nullptr, "mock", std::nullopt,
                                                    image_logger_mock, registry.get());

    scanner_application.ThreadEntry("Scanner");

    auto scanner_mocket = mocket_factory.GetMocket(zevs::Endpoint::BIND, "inproc://mock/scanner");
    CHECK_NE(scanner_mocket, nullptr);

    // Start
    REQUIRE_CALL(*image_logger_mock, AddMetaData(_, _));
    REQUIRE_CALL(*scanner_mock, Start(scanner::ScannerSensitivity::NORMAL)).RETURN(boost::outcome_v2::success());

    scanner_mocket->Dispatch(
        common::msg::scanner::Start{.sensitivity = common::msg::scanner::ScannerSensitivity::NORMAL,
                                    .interval    = std::chrono::milliseconds(50)},
        {});

    auto input_msg = scanner_mocket->Receive<common::msg::scanner::StartRsp>();
    CHECK_EQ(input_msg.value().success, true);

    // FlushImageBuffer
    REQUIRE_CALL(*image_logger_mock, FlushBuffer());
    scanner_mocket->Dispatch(common::msg::scanner::FlushImageBuffer{});

    common::Groove groove;

    factory.GetScannerOutput()->ScannerOutput(groove, 1, slice_provider::SliceConfidence::HIGH);

    auto output_msg = scanner_mocket->Receive<common::msg::scanner::SliceData>();

    CHECK(output_msg.has_value());

    // Stop
    REQUIRE_CALL(*scanner_mock, Stop());
    scanner_mocket->Dispatch(common::msg::scanner::Stop{});
  }
}
}  // namespace scanner

// NOLINTEND(*-magic-numbers, misc-include-cleaner)

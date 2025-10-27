// NOLINTBEGIN(*-magic-number)

#include "scanner/scanner.h"

#include <doctest/doctest.h>
#include <prometheus/registry.h>

#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include "common/file/yaml.h"
#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image_logger/image_logger_impl.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/joint_buffer/single_joint_buffer.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_impl.h"
#include "scanner/scanner_types.h"
#include "scanner/slice_provider/slice_provider_impl.h"

using common::file::Yaml;

namespace scanner {

namespace outcome = BOOST_OUTCOME_V2_NAMESPACE;

TEST_SUITE("Scanner") {
  class SimpleProvider : public image_provider::ImageProvider {
   public:
    auto Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> override {
      started_         = true;
      m_sending_thread = boost::thread(&SimpleProvider::SendImage, this);
      return outcome::success();
    }

    void Stop() override {
      m_sending_thread.join();
      started_ = false;
    }

    auto Started() const -> bool override { return started_; }

    void ResetFOVAndGain() override {};
    void SetVerticalFOV(int offset_from_top, int height) override {};
    void SetHorizontalFOV(int offset_from_left, int width) override {};
    void AdjustGain(double factor) override {};
    auto GetVerticalFOVOffset() -> int override { return 0; };
    auto GetVerticalFOVHeight() -> int override { return 0; };
    auto GetHorizontalFOVOffset() -> int override { return 0; };
    auto GetHorizontalFOVWidth() -> int override { return 0; };
    auto GetSerialNumber() -> std::string override { return ""; };
    void SetOnImage(OnImage on_image) override { on_image_ = on_image; }

    void SendImage() {
      // Construct an image and immediately it to the image event handler
      using Eigen::Index;

      auto image_data = image::RawImageData(2500, 3500);

      // Return a straight line for now
      for (Index i = 0; i < image_data.cols(); i++) {
        image_data(1000, i) = static_cast<uint8_t>(255);
      }

      auto image = image::ImageBuilder::From(std::move(image_data), 0).Finalize().value();

      on_image_(std::move(image));
    }

   private:
    boost::thread m_sending_thread;
    bool started_;
    OnImage on_image_;
  };

  class CameraMock : public image::CameraModel {
    auto ImageToWorkspace(const image::PlaneCoordinates& coordinates, int vertical_crop_offset) const
        -> boost::outcome_v2::result<image::WorkspaceCoordinates> override {
      image::WorkspaceCoordinates wcs(3, coordinates.cols());
      wcs << coordinates.row(0).array(), coordinates.row(1).array(),
          Eigen::RowVectorXd::Zero(coordinates.cols()).array();

      return wcs;
    }

    auto WorkspaceToImage(const image::WorkspaceCoordinates& coordinates, int vertical_crop_offset) const
        -> boost::outcome_v2::result<image::PlaneCoordinates> override {
      image::PlaneCoordinates image(2, coordinates.cols());
      image << coordinates.row(0).array(), coordinates.row(1).array();

      return image;
    }
  };
  class JointModelMock : public scanner::joint_model::JointModel {
   public:
    JointModelMock(const scanner::joint_model::JointProperties& properties, image::CameraModelPtr camera_model)
        : JointModel(properties, std::move(camera_model)) {};

    auto Parse(image::Image& image, std::optional<joint_model::JointProfile> median_profile,
               std::optional<joint_model::JointProperties> updated_properties, bool use_approximation,
               std::optional<std::tuple<double, double>> approximated_abw0_abw6_horizontal)
        -> std::expected<std::tuple<joint_model::JointProfile, image::WorkspaceCoordinates, uint64_t, uint64_t>,
                         scanner::joint_model::JointModelErrorCode> {
      joint_model::JointProfile profile;
      image::WorkspaceCoordinates coordinates;

      profile.points = {
          joint_model::Point{0.1, 0.1  },
          joint_model::Point{0.3, -0.1 },
          joint_model::Point{0.4, -0.12},
          joint_model::Point{0.5, -0.14},
          joint_model::Point{0.6, -0.12},
          joint_model::Point{0.7, -0.1 },
          joint_model::Point{0.8, 0.1  }
      };

      return std::make_tuple(profile, coordinates, 0, 0);
    }
  };

  TEST_CASE("Field-of-View changes") {
    auto [o1, h1] = ScannerImpl::NewOffsetAndHeight(100, 400);
    CHECK_EQ(o1, 0);
    CHECK_EQ(h1, MINIMUM_FOV_HEIGHT);

    auto [o2, h2] = ScannerImpl::NewOffsetAndHeight(100, 500);
    CHECK_EQ(o2, 100 - WINDOW_MARGIN);
    CHECK_EQ(h2, 500 + WINDOW_MARGIN);

    auto [o3, h3] = ScannerImpl::NewOffsetAndHeight(0, 300);
    CHECK_EQ(o3, 0);
    CHECK_EQ(h3, MINIMUM_FOV_HEIGHT);
  }

  TEST_CASE("Scan single image") {
    using namespace std::chrono_literals;

    std::unique_ptr<image_provider::ImageProvider> provider =
        std::unique_ptr<image_provider::ImageProvider>(new SimpleProvider());
    auto* provider_raw = dynamic_cast<SimpleProvider*>(provider.get());

    auto camera = std::unique_ptr<image::CameraModel>(new CameraMock());

    auto joint_buffer          = std::make_unique<joint_buffer::SingleJointBuffer>();
    auto steady_clock_now_func = []() { return std::chrono::steady_clock::now(); };
    auto slice_provider =
        std::make_unique<slice_provider::SliceProviderImpl>(std::move(joint_buffer), steady_clock_now_func);
    auto properties = joint_model::JointProperties();

    auto joint_model = joint_model::JointModelPtr(new JointModelMock(properties, std::move(camera)));

    ScannerOutputCBImpl scanner_output_cb;

    auto registry = std::make_shared<prometheus::Registry>();

    auto image_logger = image_logger::ImageLoggerImpl(nullptr);
    auto scanner      = ScannerExposed(
        provider_raw, std::move(joint_model), std::move(slice_provider), [](bool state) {}, &scanner_output_cb,
        &image_logger, registry.get());

    auto on_image = [&scanner](std::unique_ptr<image::Image> img) -> void { scanner.ImageGrabbed(std::move(img)); };
    provider->SetOnImage(on_image);

    scanner.Update();

    auto result = provider_raw->Start(scanner::ScannerSensitivity::NORMAL);

    CHECK(result);

    provider_raw->Stop();

    boost::this_thread::sleep(boost::posix_time::milliseconds(200));

    scanner.Update();
  }
}
}  // namespace scanner
// NOLINTEND(*-magic-number)

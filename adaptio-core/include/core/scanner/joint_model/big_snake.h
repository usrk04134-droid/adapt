#pragma once

#include <boost/circular_buffer.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <optional>
#include <tuple>

#include "core/image/camera_model.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/scanner_configuration.h"

namespace core::scanner::joint_model {

class BigSnake : public JointModel {
 public:
  BigSnake(const JointProperties& properties, const core::scanner::ScannerConfigurationData& config_data,
           core::image::CameraModelPtr camera_model)
      : JointModel(properties, std::move(camera_model)), threshold_(config_data.gray_minimum_wall) {}

  BigSnake(const BigSnake&)                        = delete;
  auto operator=(const BigSnake&) -> BigSnake&     = delete;
  BigSnake(BigSnake&&) noexcept                    = delete;
  auto operator=(BigSnake&&) noexcept -> BigSnake& = delete;

  ~BigSnake() = default;

  /**
   * Tries to calculate ABW points from the input image coordinates.
   *
   * @param image          Image
   * @param median_profile The previous median profile, if any.
   * @return The identified groove information from the laser line coordinates.
   */
  auto Parse(core::image::Image& image, std::optional<JointProfile> median_profile,
             std::optional<JointProperties> empty_joint_properties)
      -> std::expected<std::tuple<JointProfile, core::image::WorkspaceCoordinates, uint64_t, uint64_t>,
                       JointModelErrorCode> override;

 protected:
  /**
   * Mask 4 different CCW triangles:
   * - ABW0-ABW1-ABW6 (points offset right, up, and left, respectively)
   * - ABW0-ABW5-ABW6 (points offset right, up, and left, respectively)
   * - ABW0-(ABW0_x,ABW1_y)-ABW1 (points offset (down, left), none, and left, respectively)
   * - ABW6-ABW5-(ABW6_x,ABW5_y) (points offset (down, right), right, and none, respectively)
   * Note. In the first two triangle a larger offset up from abw1 resp abw5 is used.
   *       This is because of the definition of ABW1 and ABW5 i.e. it is not sure that they are
   *       placed on the snake
   *
   * @param image            The image
   * @param median_profile   The median profile if available
   * @return
   */
  auto GenerateMask(core::image::Image& image, std::optional<JointProfile> median_profile)
      -> std::optional<image::RawImageData>;

  void CropImageHorizontal(core::image::Image& image, std::optional<JointProfile> median_profile);

  static auto CalculateJointArea(const core::scanner::ABWPoints& points) -> double;

 private:
  int threshold_;
  bool found_out_of_spec_joint_width_{};
};

}  // namespace core::scanner::joint_model

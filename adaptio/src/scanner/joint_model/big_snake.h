#pragma once

#include <boost/circular_buffer.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <optional>
#include <tuple>

#include "scanner/core/scanner_configuration.h"
#include "scanner/image/camera_model.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner::joint_model {

class BigSnake : public JointModel {
 public:
  BigSnake(const JointProperties& properties, const ScannerConfigurationData& config_data,
           image::CameraModelPtr camera_model)
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
  auto Parse(image::Image& image, std::optional<JointProfile> median_profile,
             std::optional<JointProperties> updated_properties, bool use_approximation,
             std::optional<std::tuple<double, double>> abw0_abw6_horizontal)
      -> std::expected<std::tuple<JointProfile, image::WorkspaceCoordinates, uint64_t, uint64_t>,
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
  auto GenerateMask(image::Image& image, std::optional<JointProfile> median_profile)
      -> std::optional<image::RawImageData>;

  void CropImageHorizontal(image::Image& image, std::optional<JointProfile> median_profile);

 private:
  int threshold_;
  bool found_out_of_spec_joint_width_{};
};

}  // namespace scanner::joint_model

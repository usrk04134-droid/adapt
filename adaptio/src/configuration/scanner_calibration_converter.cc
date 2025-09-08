#include "scanner_calibration_converter.h"

#include <openssl/md5.h>
#include <string.h>

#include <any>
#include <boost/outcome/result.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <string>

#include "common/data/data_value.h"
#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "conf_factory.h"
#include "configuration_error.h"
#include "scanner/scanner_calibration_configuration.h"

using configuration::ScannerCalibrationConverter;

ScannerCalibrationConverter::ScannerCalibrationConverter(const std::string& tag, const std::filesystem::path& yaml_file)
    : tag_(tag), yaml_file_(yaml_file) {
  fh_ = GetFactory()->CreateFileHandler();
}

auto ScannerCalibrationConverter::ReadPersistentData() -> boost::outcome_v2::result<void> {
  try {
    auto yaml_string              = fh_->ReadFile(yaml_file_);
    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml_string, tag_);
    if (maybe_configuration_yaml.has_error()) {
      LOG_ERROR("Could not parse yaml string with error code: {}", maybe_configuration_yaml.error().to_string());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    auto serial_number                         = map.at(tag_ + "/scanner_serial_number");
    scanner_calibration_.scanner_serial_number = serial_number.Value<std::string>().value_or(
        serial_number.Value<int64_t>().transform([](int64_t i) { return std::to_string(i); }).value_or("0"));

    auto maybe_checksum = map.at(tag_ + "/checksum").Value<std::string>();
    if (!maybe_checksum.has_value()) {
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_MISSING_CHECKSUM));
    }
    auto checksum = maybe_checksum.value();

    unsigned char result[MD5_DIGEST_LENGTH];
    size_t checksum_start = strstr(yaml_string.c_str(), "checksum") - yaml_string.c_str();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    MD5((unsigned char*)yaml_string.c_str(), checksum_start, result);
#pragma GCC diagnostic pop

    for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
      unsigned char checksum_value = result[i];
      char v1                      = checksum.c_str()[i * 2];
      char v2                      = checksum.c_str()[i * 2 + 1];
      unsigned char file_checksum_value =
          ((v1 > '9') ? (v1 - 'a' + 10) : (v1 - '0')) * 16 + ((v2 > '9') ? (v2 - 'a' + 10) : (v2 - '0'));
      if (checksum_value != file_checksum_value) {
        return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_INVALID_CHECKSUM));
      }
    }

    // Calibration calibration parameters
    scanner_calibration_.intrinsic.scaling_factors.w =
        map.at(tag_ + "/intrinsic/scaling_factors/w").Value<double>().value();

    scanner_calibration_.intrinsic.scaling_factors.m =
        map.at(tag_ + "/intrinsic/scaling_factors/m").Value<double>().value();
    scanner_calibration_.intrinsic.scaling_factors.K1 =
        map.at(tag_ + "/intrinsic/scaling_factors/K1").Value<double>().value();
    scanner_calibration_.intrinsic.scaling_factors.K2 =
        map.at(tag_ + "/intrinsic/scaling_factors/K2").Value<double>().value();
    scanner_calibration_.intrinsic.scaling_factors.K3 =
        map.at(tag_ + "/intrinsic/scaling_factors/K3").Value<double>().value();
    scanner_calibration_.intrinsic.scaling_factors.P1 =
        map.at(tag_ + "/intrinsic/scaling_factors/P1").Value<double>().value();
    scanner_calibration_.intrinsic.scaling_factors.P2 =
        map.at(tag_ + "/intrinsic/scaling_factors/P2").Value<double>().value();

    scanner_calibration_.intrinsic.projection_center_distance =
        map.at(tag_ + "/intrinsic/projection_center_distance").Value<double>().value();
    scanner_calibration_.intrinsic.focus_distance = map.at(tag_ + "/intrinsic/focus_distance").Value<double>().value();
    scanner_calibration_.intrinsic.principal_point.x =
        map.at(tag_ + "/intrinsic/principal_point/x").Value<double>().value();
    scanner_calibration_.intrinsic.principal_point.y =
        map.at(tag_ + "/intrinsic/principal_point/y").Value<double>().value();
    scanner_calibration_.intrinsic.pixel_pitch.x = map.at(tag_ + "/intrinsic/pixel_pitch/x").Value<double>().value();
    scanner_calibration_.intrinsic.pixel_pitch.y = map.at(tag_ + "/intrinsic/pixel_pitch/y").Value<double>().value();
    scanner_calibration_.intrinsic.rho           = map.at(tag_ + "/intrinsic/rho").Value<double>().value();
    scanner_calibration_.intrinsic.tau           = map.at(tag_ + "/intrinsic/tau").Value<double>().value();
    scanner_calibration_.intrinsic.d             = map.at(tag_ + "/intrinsic/d").Value<double>().value();
    scanner_calibration_.intrinsic.K1            = map.at(tag_ + "/intrinsic/K1").Value<double>().value();
    scanner_calibration_.intrinsic.K2            = map.at(tag_ + "/intrinsic/K2").Value<double>().value();
    scanner_calibration_.intrinsic.K3            = map.at(tag_ + "/intrinsic/K3").Value<double>().value();
    scanner_calibration_.intrinsic.P1            = map.at(tag_ + "/intrinsic/P1").Value<double>().value();
    scanner_calibration_.intrinsic.P2            = map.at(tag_ + "/intrinsic/P2").Value<double>().value();

    auto r_data = map.at(tag_ + "/extrinsic/R").Value<common::data::Matrix>().value();
    for (int i = 0; i < r_data.rows; i++) {
      for (int j = 0; j < r_data.columns; j++) {
        scanner_calibration_.extrinsic.rotation(i, j) = r_data.data.get()[i * r_data.columns + j];
      }
    }

    auto t_data = map.at(tag_ + "/extrinsic/t").Value<common::data::Matrix>().value();
    for (int i = 0; i < t_data.rows; i++) {
      for (int j = 0; j < t_data.columns; j++) {
        scanner_calibration_.extrinsic.translation(i, j) = t_data.data.get()[i * t_data.columns + j];
      }
    }

  } catch (const std::exception& e) {
    LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file_.string(), e.what());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
  }

  return boost::outcome_v2::success();
}

auto ScannerCalibrationConverter::WritePersistentData(std::any /*config_struct*/) -> boost::outcome_v2::result<void> {
  // No support for writing to file. Should not be called
  LOG_ERROR("No support for writing to file for tag: {}", tag_);
  return boost::outcome_v2::success();
}

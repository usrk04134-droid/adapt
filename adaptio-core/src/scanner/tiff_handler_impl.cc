#include "core/scanner/tiff_handler_impl.h"

#include <tiff.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

#include "core/image/image.h"
#include "core/time/format.h"
#include "tiffio.h"

using core::scanner::TiffHandlerImpl;

#define N(a) (sizeof(a) / sizeof(a[0]))

static const TIFFFieldInfo xtiffFieldInfo[] = {

    // FOV x offset
    {core::scanner::TIFFTAG_X_OFFSET,            1, 1, TIFF_SHORT, FIELD_CUSTOM, 0, 0, const_cast<char*>("YOffset")      },

    // FOV y offset
    {core::scanner::TIFFTAG_Y_OFFSET,            1, 1, TIFF_SHORT, FIELD_CUSTOM, 0, 0, const_cast<char*>("YOffset")      },

    // Time stamp, milli seconds since epoch. Needs to in ascii since libtiff only supports max 8 bytes
    {core::scanner::TIFFTAG_TIMESTAMP_RFC3339,   1, 1, TIFF_ASCII, FIELD_CUSTOM, 0, 0, const_cast<char*>("TimeStampRFC") },

    // Time stamp, milli seconds since epoch. Needs to in ascii since libtiff only supports max 8 bytes
    {core::scanner::TIFFTAG_TIMESTAMP,           1, 1, TIFF_ASCII, FIELD_CUSTOM, 0, 0, const_cast<char*>("TimeStamp")    },

    // Scanner serial - Null terminated ascii string in yaml format
    {core::scanner::TIFFTAG_SCANNER_CALIBRATION, 1, 1, TIFF_ASCII, FIELD_CUSTOM, 0, 0,
     const_cast<char*>("ScannerCalibration")                                                                             },

    // Joint Geometry - Null termionated ascii string in yaml format
    {core::scanner::TIFFTAG_JOINT_GEOMETRY,      1, 1, TIFF_ASCII, FIELD_CUSTOM, 0, 0, const_cast<char*>("JointGeometry")}
};

static TIFFExtendProc parent_extender = nullptr;  // In case we want a chain of extensions

static void registerCustomTIFFTags(TIFF* tif) {
  // Install the extended Tag field info
  TIFFMergeFieldInfo(tif, xtiffFieldInfo, N(xtiffFieldInfo));

  if (parent_extender != nullptr) {
    (*parent_extender)(tif);
  }
};

TiffHandlerImpl::TiffHandlerImpl() {
  if (parent_extender == nullptr) {
    parent_extender = (TIFFSetTagExtender(registerCustomTIFFTags));
  }
}

TiffHandlerImpl::TiffHandlerImpl(const std::string& joint_geometry_yaml, const std::string& scanner_calib_yaml)
    : joint_geometry_yaml_(joint_geometry_yaml), scanner_calib_yaml_(scanner_calib_yaml) {
  if (parent_extender == nullptr) {
    parent_extender = (TIFFSetTagExtender(registerCustomTIFFTags));
  }
}

void TiffHandlerImpl::Write(const core::image::Image* image, const std::filesystem::path& log_path, uint32_t x_offset,
                            uint32_t y_offset) {
  auto time_stamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(image->GetTimestamp().time_since_epoch()).count();

  std::ostringstream abs_path;
  abs_path << log_path.c_str() << "/" << time_stamp << ".tiff";

  TIFF* tif = TIFFOpen(abs_path.str().c_str(), "w");
  // Standard tags
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, image->Data().cols());

  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, image->Data().rows());
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
  TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_LZW);
  TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, image->Data().rows());

  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);

  // Convert high_resolution_clock time_point to system_clock time_point. For GCC on Linux
  // these types are currently equivalent, but not for example on macOS.
  std::chrono::system_clock::time_point now           = std::chrono::system_clock::now();
  std::chrono::high_resolution_clock::time_point hnow = std::chrono::high_resolution_clock::now();
  std::chrono::system_clock::time_point tp =
      now + std::chrono::duration_cast<std::chrono::system_clock::duration>(image->GetTimestamp() - hnow);

  // Application tags
  TIFFSetField(tif, TIFFTAG_X_OFFSET, x_offset);
  TIFFSetField(tif, TIFFTAG_Y_OFFSET, y_offset);
  TIFFSetField(tif, TIFFTAG_TIMESTAMP_RFC3339, core::time::TimePointToString(tp, core::time::FMT_TS_MS).c_str());
  TIFFSetField(tif, TIFFTAG_TIMESTAMP, std::to_string(time_stamp).c_str());
  TIFFSetField(tif, TIFFTAG_JOINT_GEOMETRY, joint_geometry_yaml_.c_str());
  TIFFSetField(tif, TIFFTAG_SCANNER_CALIBRATION, scanner_calib_yaml_.c_str());

  TIFFWriteEncodedStrip(tif, 0, static_cast<void*>(const_cast<unsigned char*>(image->Data().data())),
                        image->Data().cols() * image->Data().rows());

  // Write the directory to a file, and close it:
  TIFFWriteDirectory(tif);

  TIFFClose(tif);
}

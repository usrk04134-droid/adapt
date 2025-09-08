#include "scanner/image_logger/image_logger.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, modernize-use-trailing-return-type)

#include <trompeloeil.hpp>

class ImageLoggerMock : public trompeloeil::mock_interface<scanner::image_logger::ImageLogger> {
  IMPLEMENT_MOCK1(Update);
  IMPLEMENT_MOCK2(AddMetaData);
  IMPLEMENT_MOCK1(LogImage);
  IMPLEMENT_MOCK2(LogImageError);
  IMPLEMENT_MOCK0(FlushBuffer);
};

// NOLINTEND(*-magic-numbers, *-optional-access, modernize-use-trailing-return-type)



#include <trompeloeil.hpp>

#include "../../conf_file_handler.h"

namespace configuration {

using trompeloeil::mock_interface;

class MockFileHander : public mock_interface<FileHandler> {
 public:
  IMPLEMENT_MOCK1(ReadFile);
  IMPLEMENT_MOCK2(WriteFile);
  IMPLEMENT_MOCK1(FileExist);
  IMPLEMENT_MOCK1(GetAbsParent);
  IMPLEMENT_MOCK1(SetWritePermission);
};
}  // namespace configuration

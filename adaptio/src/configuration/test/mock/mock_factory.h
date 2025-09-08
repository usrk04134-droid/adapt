
#include <functional>
#include <trompeloeil.hpp>

#include "../../conf_factory.h"

namespace configuration {

using trompeloeil::mock_interface;

class MockFactory : public mock_interface<Factory> {
 public:
  ~MockFactory() override { SetFactoryGenerator(std::function<Factory *()>{}); }
  IMPLEMENT_MOCK2(CreateConverter);
  IMPLEMENT_MOCK0(CreateFileHandler);
};
}  // namespace configuration

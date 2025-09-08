# Adaptio core

1. [FSM](docs/FSM.md)

## macOS build

After installing the required Homebrew packages, there is a misconfiguration or conflict when both VTK and PCL are installed.

Edit /usr/local/lib/cmake/vtk-9.3/VTK-targets.cmake and add this before "set_target_properties(VTK::jsoncpp ...". This may need to be repeated after Homebrew updates.

```cmake
if(NOT TARGET JsonCpp::JsonCpp)
# Attempt to find JsonCpp library - modify paths as needed
find_library(JSONCPP_LIBRARY NAMES jsoncpp PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/lib /usr/local/lib)
find_path(JSONCPP_INCLUDE_DIR NAMES json/json.h PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/include /usr/local/include)

# Create an imported target
add_library(JsonCpp::JsonCpp SHARED IMPORTED)
set_target_properties(JsonCpp::JsonCpp PROPERTIES
    IMPORTED_LOCATION "${JSONCPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIR}")
endif()
```

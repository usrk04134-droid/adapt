cmake_minimum_required(VERSION 3.27)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)

if(DEFINED GIT_REV_SHORT)
  set(ADAPTIO_CORE_GIT_COMMIT_HASH ${GIT_REV_SHORT})
else()
  # Get the latest commit hash
  execute_process(
    COMMAND git describe --always --dirty --no-match
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE ADAPTIO_CORE_GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

# Get the current version from version.txt
file(READ version.txt version ERROR_VARIABLE ERROR_MSG)

if (ERROR_MSG)
  message(FATAL_ERROR "Failed to read file: ${ERROR_MSG}")
endif()

# Parse the content from version.txt and match it against the SemVer format
string(REGEX MATCH "([0-9]+)[.]([0-9]+)[.]([0-9]+)(-([0-9A-Za-z.-]+))?" match_semver ${version})

if (match_semver)
  set(ADAPTIO_CORE_VERSION ${CMAKE_MATCH_0})
  set(ADAPTIO_CORE_VERSION_MAJOR ${CMAKE_MATCH_1})
  set(ADAPTIO_CORE_VERSION_MINOR ${CMAKE_MATCH_2})
  set(ADAPTIO_CORE_VERSION_PATCH ${CMAKE_MATCH_3})
  set(ADAPTIO_CORE_CMAKE_PROJECT_VERSION "${ADAPTIO_CORE_VERSION_MAJOR}.${ADAPTIO_CORE_VERSION_MINOR}.${ADAPTIO_CORE_VERSION_PATCH}")

  if (CMAKE_MATCH_5)  # Check if pre-release exists
    set(ADAPTIO_CORE_VERSION_PRE_RELEASE "${CMAKE_MATCH_5}")
    # CMake project() version does not support the SemVer format for pre-release, instead we add it as .(tweak)
    set(ADAPTIO_CORE_CMAKE_PROJECT_VERSION "${ADAPTIO_CORE_CMAKE_PROJECT_VERSION}.${ADAPTIO_CORE_VERSION_PRE_RELEASE}")
  endif()
else()
  message(FATAL_ERROR "Unable to find version!")
endif()

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/assets/a2A4096-9gmPRO_40297732.pfs PFS_FILE_CONTENT)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/scanner/image_provider/basler_camera.cc ${CMAKE_CURRENT_SOURCE_DIR}/build/basler_camera_with_pfs.cc)

message(STATUS "Building artifacts for ${ADAPTIO_CORE_VERSION} ${ADAPTIO_CORE_GIT_COMMIT_HASH}")

# Project start
project(AdaptioCore VERSION "${ADAPTIO_CORE_CMAKE_PROJECT_VERSION}")

configure_file(${PROJECT_SOURCE_DIR}/include/core/version.h.in ${PROJECT_SOURCE_DIR}/include/core/version.h)

set(ADAPTIO_CORE_LIB adaptio-core)
set(ADAPTIO_CORE_TESTS adaptio-core-tests)
set(ADAPTIO_TEST_IMAGES test-images)
set(ADAPTIO_DATA_SET_RUNNER data-set-runner)
set(ADAPTIO_IMAGE_ANNOTATION image-annotation)
set(ADAPTIO_CHECK_REPORT check-report)

add_library(${ADAPTIO_CORE_LIB} SHARED)

target_include_directories(${ADAPTIO_CORE_LIB} PRIVATE include)
target_include_directories(${ADAPTIO_CORE_LIB} PRIVATE src)

set(HEADER_FILES
    include/core/version.h
    include/core/channels/buffered_channel.h
    include/core/data/data_value.h
    include/core/file/yaml.h
    include/core/fsm/state.h
    include/core/image/camera_model.h
    include/core/image/image.h
    include/core/image/image_builder.h
    include/core/image/image_data.h
    include/core/image/image_types.h
    include/core/image/tilted_perspective_camera.h
    include/core/joint_tracking/joint_slice.h
    include/core/joint_tracking/joint_tracking.h
    include/core/math/centroids.h
    include/core/math/filter.h
    include/core/math/lin_interp.h
    include/core/math/value.h
    include/core/scanner/image_logger.h
    include/core/scanner/image_logger_impl.h
    include/core/scanner/factory.h
    include/core/scanner/factory_impl.h
    include/core/scanner/image_provider.h
    include/core/scanner/image_provider/basler_camera.h
    include/core/scanner/image_provider/simulation/camera_simulation.h
    include/core/scanner/image_provider_configuration.h
    include/core/scanner/joint_buffer/circular_joint_buffer.h
    include/core/scanner/joint_buffer/joint_buffer.h
    include/core/scanner/joint_buffer/single_joint_buffer.h
    include/core/scanner/joint_model.h
    include/core/scanner/joint_model/naive.h
    include/core/scanner/joint_model/big_snake.h
    include/core/scanner/joint_model/snake.h
    include/core/scanner/joint_model/slice.h
    include/core/scanner/scanner.h
    include/core/scanner/scanner_types.h
    include/core/scanner/scanner_calibration_configuration.h
    include/core/scanner/scanner_configuration.h
    include/core/scanner/scanner_impl.h
    include/core/scanner/tiff_handler.h
    include/core/scanner/tiff_handler_impl.h
    include/core/time/format.h
    include/core/logging/application_log.h
    include/core/logging/component_logger.h
)

set(SOURCE_FILES
    src/image/image_utilities.h

    build/basler_camera_with_pfs.cc

    src/channels/buffered_channel.cc
    src/data/data_value.cc
    src/file/yaml.cc
    src/fsm/state.cc
    src/fsm/state_builder.cc
    src/fsm/transition.cc
    src/fsm/transition_builder.cc
    src/image/camera_model.cc
    src/image/image.cc
    src/image/image_builder.cc
    src/image/image_data.cc
    src/image/image_utilities.cc
    src/image/tilted_perspective_camera.cc
    src/joint_tracking/joint_slice.cc
    src/joint_tracking/joint_tracking.cc
    src/math/centroids.cc
    src/math/filter.cc
    src/math/lin_interp.cc
    src/math/value.cc
    src/scanner/image_logger_impl.cc
    src/scanner/factory_impl.cc
    src/scanner/image_provider/simulation/camera_simulation.cc
    src/scanner/joint_buffer/circular_joint_buffer.cc
    src/scanner/joint_buffer/single_joint_buffer.cc
    src/scanner/joint_model.cc
    src/scanner/joint_model/naive.cc
    src/scanner/joint_model/big_snake.cc
    src/scanner/joint_model/snake.cc
    src/scanner/joint_model/slice.cc
    src/scanner/joint_model/test/slice_test.cc
    src/scanner/scanner_impl.cc
    src/scanner/tiff_handler_impl.cc
    src/logging/component_logger.cc
    src/time/format.cc
)

target_sources(${ADAPTIO_CORE_LIB}
  PUBLIC
    FILE_SET HEADERS
    TYPE HEADERS
    BASE_DIRS include
    FILES
      ${HEADER_FILES}
)

target_sources(${ADAPTIO_CORE_LIB}
  PRIVATE
    ${SOURCE_FILES}
)

# shared libraries need PIC
set_target_properties(${ADAPTIO_CORE_LIB} PROPERTIES POSITION_INDEPENDENT_CODE 1)

# Fix compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if (CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif ()

# Test specific build settings for non-realtime systems (e.g. CI)
option(ENABLE_CPU_PROCESSING_TIME "Enable CPU-time measurement instead of wall clock" OFF)
if (ENABLE_CPU_PROCESSING_TIME)
  add_compile_definitions(CPU_PROCESSING_TIME=1)
endif ()

# Docs
if (CMAKE_BUILD_TYPE STREQUAL "Release") # Only build docs for release builds
  option(BUILD_DOC "Build documentation" ON)

  find_package(Doxygen)
  if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the
    set(INPUT ${CMAKE_CURRENT_SOURCE_DIR}/src)
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(AdaptioCoreDoxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
  else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
  endif (DOXYGEN_FOUND)
endif ()

if (APPLE)
  set(DEBUG_COMPILE_OPTIONS -gdwarf-4 -ferror-limit=0 -Wignored-optimization-argument -DMACOS)
  set(RELEASE_COMPILE_OPTIONS -gdwarf-4 -ferror-limit=0 -O3 -DMACOS)
else (APPLE)
  set(DEBUG_COMPILE_OPTIONS -Wall -Werror -pedantic -gdwarf-4 -ferror-limit=0)
  set(RELEASE_COMPILE_OPTIONS -Wall -Werror -pedantic -gdwarf-4 -O3)
endif (APPLE)
set(DEBUG_LINK_OPTIONS -v)
set(RELEASE_LINK_OPTIONS -v)
set(COMPILE_FEATURES cxx_std_23)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(${ADAPTIO_CORE_LIB} PRIVATE ${DEBUG_COMPILE_OPTIONS})
  target_link_options(${ADAPTIO_CORE_LIB} PRIVATE ${DEBUG_LINK_OPTIONS})
else ()
  target_compile_options(${ADAPTIO_CORE_LIB} PRIVATE ${RELEASE_COMPILE_OPTIONS})
  target_link_options(${ADAPTIO_CORE_LIB} PRIVATE ${RELEASE_LINK_OPTIONS})
endif ()
target_compile_features(${ADAPTIO_CORE_LIB} PRIVATE ${COMPILE_FEATURES})

# Disable doctest for library builds
# NOTE: This requires "#ifndef DOCTEST_CONFIG_DISABLE" around the test suites to avoid unused variables.
target_compile_definitions(${ADAPTIO_CORE_LIB} PRIVATE DOCTEST_CONFIG_DISABLE)

find_package(Boost 1.81 COMPONENTS program_options context log_setup log REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC ${Boost_LIBRARIES})

find_package(Ceres REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PRIVATE Ceres::ceres)

find_package(PCL 1.13 COMPONENTS sample_consensus REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PRIVATE ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC Eigen3::Eigen)

find_package(OpenCV 4.7 REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PRIVATE ${OpenCV_LIBRARIES})
add_definitions(${OPENCV_DEFINITIONS})

find_package(TIFF REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC TIFF::TIFF)

find_package(fmt REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC fmt::fmt)

find_package(prometheus-cpp REQUIRED)
target_link_libraries(${ADAPTIO_CORE_LIB} PRIVATE prometheus-cpp::pull)

find_package(yaml-cpp 0.7.0 REQUIRED)
if (APPLE)
  #execute_process(COMMAND dirname `which brew` OUTPUT_VARIABLE BREW_BINARY)
  if (EXISTS /opt/homebrew/Cellar/yaml-cpp)
    target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC /opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.0.8.0.dylib)
  else ()
    target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC /usr/local/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.0.8.0.dylib)
  endif ()
else (APPLE)
  target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC yaml-cpp)
endif (APPLE)

# Pylon
find_package(Pylon REQUIRED)
target_include_directories(${ADAPTIO_CORE_LIB} PRIVATE ${Pylon_INCLUDE_DIR})
if (APPLE)
  target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libpylonbase.7.3.1.dylib)
  target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libpylonutility.7.3.1.dylib)
  target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libGenApi_gcc_v3_1_Basler_pylon.dylib)
  target_link_libraries(${ADAPTIO_CORE_LIB} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libGCBase_gcc_v3_1_Basler_pylon.dylib)
else (APPLE)
  target_link_libraries(${ADAPTIO_CORE_LIB} PRIVATE ${Pylon_LIBRARIES})
endif (APPLE)

# Tooling
add_executable(${ADAPTIO_TEST_IMAGES} tools/test_image_set.cc)
target_include_directories(${ADAPTIO_TEST_IMAGES} PRIVATE ${Pylon_INCLUDE_DIR})
target_include_directories(${ADAPTIO_TEST_IMAGES} PRIVATE include)
target_link_libraries(${ADAPTIO_TEST_IMAGES} PRIVATE ${ADAPTIO_CORE_LIB})
target_link_libraries(${ADAPTIO_TEST_IMAGES} PRIVATE ${OpenCV_LIBRARIES})
target_link_libraries(${ADAPTIO_TEST_IMAGES} PRIVATE prometheus-cpp::pull)
target_compile_options(${ADAPTIO_TEST_IMAGES} PRIVATE ${DEBUG_COMPILE_OPTIONS})
target_compile_definitions(${ADAPTIO_TEST_IMAGES} PRIVATE DOCTEST_CONFIG_DISABLE)
target_compile_features(${ADAPTIO_TEST_IMAGES} PRIVATE ${COMPILE_FEATURES})
target_link_options(${ADAPTIO_TEST_IMAGES} PRIVATE ${DEBUG_LINK_OPTIONS})

add_executable(${ADAPTIO_DATA_SET_RUNNER} tools/data_set_runner.cc)
target_include_directories(${ADAPTIO_DATA_SET_RUNNER} PRIVATE include)
find_package(Boost 1.80 COMPONENTS program_options context log_setup log REQUIRED)
find_package(yaml-cpp 0.7.0 REQUIRED)
if (APPLE)
  target_link_libraries(${ADAPTIO_DATA_SET_RUNNER} PUBLIC /opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.0.8.0.dylib)
else (APPLE)
  target_link_libraries(${ADAPTIO_DATA_SET_RUNNER} PUBLIC yaml-cpp)
endif (APPLE)
target_link_libraries(${ADAPTIO_DATA_SET_RUNNER} PUBLIC ${Boost_LIBRARIES})
target_compile_options(${ADAPTIO_DATA_SET_RUNNER} PRIVATE ${DEBUG_COMPILE_OPTIONS})
target_compile_features(${ADAPTIO_DATA_SET_RUNNER} PRIVATE ${COMPILE_FEATURES})
target_link_options(${ADAPTIO_DATA_SET_RUNNER} PRIVATE ${DEBUG_LINK_OPTIONS})

add_executable(${ADAPTIO_IMAGE_ANNOTATION} tools/image_annotation.cc)
target_include_directories(${ADAPTIO_IMAGE_ANNOTATION} PRIVATE include)
target_link_libraries(${ADAPTIO_IMAGE_ANNOTATION} PRIVATE ${ADAPTIO_CORE_LIB})
target_compile_options(${ADAPTIO_IMAGE_ANNOTATION} PRIVATE ${DEBUG_COMPILE_OPTIONS})
target_compile_definitions(${ADAPTIO_IMAGE_ANNOTATION} PRIVATE DOCTEST_CONFIG_DISABLE)
target_compile_features(${ADAPTIO_IMAGE_ANNOTATION} PRIVATE ${COMPILE_FEATURES})
target_link_options(${ADAPTIO_IMAGE_ANNOTATION} PRIVATE ${DEBUG_LINK_OPTIONS})

add_executable(${ADAPTIO_CHECK_REPORT} tools/check_report.cc)
target_include_directories(${ADAPTIO_CHECK_REPORT} PRIVATE include)
find_package(Boost 1.80 COMPONENTS program_options context log_setup log REQUIRED)
find_package(yaml-cpp 0.7.0 REQUIRED)
if (APPLE)
  target_link_libraries(${ADAPTIO_CHECK_REPORT} PUBLIC /opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.0.8.0.dylib)
else (APPLE)
  target_link_libraries(${ADAPTIO_CHECK_REPORT} PUBLIC yaml-cpp)
endif (APPLE)
target_link_libraries(${ADAPTIO_CHECK_REPORT} PUBLIC ${Boost_LIBRARIES})
target_compile_options(${ADAPTIO_CHECK_REPORT} PRIVATE ${DEBUG_COMPILE_OPTIONS})
target_compile_features(${ADAPTIO_CHECK_REPORT} PRIVATE ${COMPILE_FEATURES})
target_link_options(${ADAPTIO_CHECK_REPORT} PRIVATE ${DEBUG_LINK_OPTIONS})

# Unit tests
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(doctest REQUIRED)
  add_executable(${ADAPTIO_CORE_TESTS} tests/main.cc)
  target_include_directories(${ADAPTIO_CORE_TESTS} PUBLIC ${DOCTEST_INCLUDE_DIR})
  target_link_libraries(${ADAPTIO_CORE_TESTS} LINK_PRIVATE doctest::doctest)

  target_include_directories(${ADAPTIO_CORE_TESTS} PRIVATE include)
  target_include_directories(${ADAPTIO_CORE_TESTS} PRIVATE src)
  target_include_directories(${ADAPTIO_CORE_TESTS} PRIVATE tests)
  target_sources(${ADAPTIO_CORE_TESTS}
      PRIVATE
        ${HEADER_FILES}
        ${SOURCE_FILES}
        src/scanner/joint_model/test_data.h
  )

  find_package(Eigen3 REQUIRED)
  target_include_directories(${ADAPTIO_CORE_TESTS} PRIVATE ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(${ADAPTIO_CORE_TESTS} PRIVATE Eigen3::Eigen)

  find_package(Boost 1.80 COMPONENTS program_options context log_setup log REQUIRED)
  target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC ${Boost_LIBRARIES})

  find_package(PCL 1.13 REQUIRED)
  target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC ${PCL_LIBRARIES})
  add_definitions(${PCL_DEFINITIONS})

  find_package(fmt REQUIRED)
  target_link_libraries(${ADAPTIO_CORE_TESTS} PRIVATE fmt::fmt)

  find_package(yaml-cpp 0.7.0 REQUIRED)

  find_package(prometheus-cpp REQUIRED)
  target_link_libraries(${ADAPTIO_CORE_TESTS} PRIVATE prometheus-cpp::pull)

  # Pylon
  find_package(Pylon REQUIRED)
  target_include_directories(${ADAPTIO_CORE_TESTS} PRIVATE ${Pylon_INCLUDE_DIR})

  if (APPLE)
    if (EXISTS /opt/homebrew/Cellar/yaml-cpp)
      target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC /opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.0.8.0.dylib)
    else ()
      target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC /usr/local/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.0.8.0.dylib)
    endif ()
    target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libpylonbase.7.3.1.dylib)
    target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libpylonutility.7.3.1.dylib)
    target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libGenApi_gcc_v3_1_Basler_pylon.dylib)
    target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC /Library/Frameworks/pylon.framework/Versions/A/Libraries/libGCBase_gcc_v3_1_Basler_pylon.dylib)
  else (APPLE)
    target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC yaml-cpp)
    target_link_libraries(${ADAPTIO_CORE_TESTS} PRIVATE ${Pylon_LIBRARIES})
  endif (APPLE)

  find_package(Ceres REQUIRED)
  target_link_libraries(${ADAPTIO_CORE_TESTS} PRIVATE Ceres::ceres)

  find_package(OpenCV 4.7 REQUIRED)
  target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC ${OpenCV_LIBRARIES})
  add_definitions(${OPENCV_DEFINITIONS})

  find_package(TIFF REQUIRED)
  target_link_libraries(${ADAPTIO_CORE_TESTS} PUBLIC TIFF::TIFF)

  target_compile_options(${ADAPTIO_CORE_TESTS} PRIVATE ${DEBUG_COMPILE_OPTIONS})
  target_link_options(${ADAPTIO_CORE_TESTS} PRIVATE ${DEBUG_LINK_OPTIONS})
  target_compile_features(${ADAPTIO_CORE_TESTS} PRIVATE ${COMPILE_FEATURES})
endif ()

# Install libraries
install(TARGETS ${ADAPTIO_CORE_LIB}
  EXPORT AdaptioCoreTargets
  LIBRARY DESTINATION lib
  COMPONENT ${ADAPTIO_CORE_LIB}
  FILE_SET HEADERS
  COMPONENT ${ADAPTIO_CORE_LIB}
  )

# Install tools
install(TARGETS ${ADAPTIO_TEST_IMAGES} # Needs adaptio lib
  RUNTIME DESTINATION bin
  COMPONENT ${ADAPTIO_TEST_IMAGES}
  )
install(TARGETS ${ADAPTIO_CORE_LIB}
  LIBRARY DESTINATION lib
  COMPONENT ${ADAPTIO_TEST_IMAGES}
  EXCLUDE_FROM_ALL
  )

install(TARGETS ${ADAPTIO_DATA_SET_RUNNER} # Does not need adaptio lib
  RUNTIME DESTINATION bin
  COMPONENT ${ADAPTIO_DATA_SET_RUNNER}
  )

install(TARGETS ${ADAPTIO_IMAGE_ANNOTATION} # Needs adaptio lib
  RUNTIME DESTINATION bin
  COMPONENT ${ADAPTIO_IMAGE_ANNOTATION}
  )
install(TARGETS ${ADAPTIO_CORE_LIB}
  LIBRARY DESTINATION lib
  COMPONENT ${ADAPTIO_IMAGE_ANNOTATION}
  EXCLUDE_FROM_ALL
  )

# Install tests
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  install(TARGETS ${ADAPTIO_CORE_TESTS}
    RUNTIME DESTINATION bin
    COMPONENT ${ADAPTIO_CORE_TESTS}
    )
endif()

# Install the project config
include(CMakePackageConfigHelpers)
write_basic_package_version_file("AdaptioCoreConfigVersion.cmake" VERSION "${ADAPTIO_CORE_VERSION}" COMPATIBILITY AnyNewerVersion)

install(EXPORT AdaptioCoreTargets
  FILE AdaptioCoreTargets.cmake
  NAMESPACE AdaptioCore::
  DESTINATION lib/cmake/AdaptioCore
  COMPONENT ${ADAPTIO_CORE_LIB}
  )

install(FILES "AdaptioCoreConfig.cmake" "${CMAKE_CURRENT_BINARY_DIR}/AdaptioCoreConfigVersion.cmake"
  DESTINATION lib/cmake/AdaptioCore
  COMPONENT ${ADAPTIO_CORE_LIB}
  )

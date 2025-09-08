cmake_minimum_required(VERSION 3.27)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(DEFINED GIT_REV_SHORT)
  set(ADAPTIO_GIT_COMMIT_HASH ${GIT_REV_SHORT})
else()
  # Get the latest commit hash
  execute_process(
    COMMAND git describe --always --dirty --no-match
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE ADAPTIO_GIT_COMMIT_HASH
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
  set(ADAPTIO_VERSION ${CMAKE_MATCH_0})
  set(ADAPTIO_VERSION_MAJOR ${CMAKE_MATCH_1})
  set(ADAPTIO_VERSION_MINOR ${CMAKE_MATCH_2})
  set(ADAPTIO_VERSION_PATCH ${CMAKE_MATCH_3})
  set(ADAPTIO_PROJECT_VERSION "${ADAPTIO_VERSION_MAJOR}.${ADAPTIO_VERSION_MINOR}.${ADAPTIO_VERSION_PATCH}")

  if (CMAKE_MATCH_5)  # Check if pre-release exists
    set(ADAPTIO_VERSION_PRE_RELEASE "${CMAKE_MATCH_5}")
    # CMake project() version does not support the SemVer format for pre-release, instead we add it as .(tweak)
    set(ADAPTIO_PROJECT_VERSION "${ADAPTIO_PROJECT_VERSION}.${ADAPTIO_VERSION_PRE_RELEASE}")
  endif()
else()
  message(FATAL_ERROR "Unable to find version!")
endif()

message(STATUS "Building artifacts for ${ADAPTIO_VERSION} ${ADAPTIO_GIT_COMMIT_HASH}")

# Project start
project(Adaptio VERSION "${ADAPTIO_PROJECT_VERSION}")

# Add our sources
add_subdirectory(src)

# Fix compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

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
    add_custom_target(Doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
  else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
  endif (DOXYGEN_FOUND)
endif ()

# Add git hooks
find_package(Git QUIET)

if (GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
  execute_process(COMMAND ${GIT_EXECUTABLE} config core.hooksPath .githooks
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE GIT_SUBMOD_RESULT)
  if (NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "Unable to configure git hooks!")
  endif ()
endif ()

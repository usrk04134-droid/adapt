cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(benchmarks CXX)

# place binaries and libraries according to GNU standards

include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

find_package(fmt REQUIRED)
find_package(SQLiteCpp REQUIRED)
find_package(Boost 1.81 COMPONENTS program_options context log_setup log REQUIRED)

add_executable(
    benchmarks
    main.cc
    sqlite_benchmark.cc
    gaussian_filter_benchmark.cc
)

target_compile_features(
  benchmarks
  PRIVATE cxx_std_23
)
target_compile_options(benchmarks PRIVATE -Wall -Werror -pedantic -gdwarf-4)

target_link_libraries(
    benchmarks
    PRIVATE fmt::fmt
    PRIVATE SQLiteCpp
    PRIVATE ${Boost_LIBRARIES}
)

target_include_directories(benchmarks PRIVATE ../..)

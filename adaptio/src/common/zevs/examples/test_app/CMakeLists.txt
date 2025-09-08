cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(cppzmq-examples CXX)

# place binaries and libraries according to GNU standards

include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

find_package(Threads)
find_package(cppzmq)
find_package(fmt REQUIRED)
find_package(Boost 1.81 COMPONENTS program_options context log_setup log REQUIRED)

add_executable(
    event_test
    event_test.cc
    ../../src/eventloop_impl.cc
    ../../src/coresocket_impl.cc
    ../../src/coretimer_impl.cc
    ../../src/timer_backend.cc
    ../../src/timer_thread.cc
    ../../src/corefactory_impl.cc
    ../../src/process_impl.cc
    ../../src/message_impl.cc
    ../../src/factory_impl.cc
)

target_compile_features(
  event_test
  PRIVATE cxx_std_23
)

target_link_libraries(
    event_test
    PRIVATE cppzmq ${CMAKE_THREAD_LIBS_INIT}
    PRIVATE fmt::fmt
    PRIVATE ${Boost_LIBRARIES}
)

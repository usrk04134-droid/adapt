cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

target_sources(${ADAPTIO_APPLICATION}
   PRIVATE
   simulator_interface.h
   point3d.h
   sim-config.h
)

project(simlib-project, CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

add_executable(
  simcal
  simulator_interface.h
  collision-listener-interface.h
  point3d.h
  sim-config.h
)

add_executable(
  simviz
  simulator_interface.h
)


add_library(
  depsim
  STATIC
  simulator_interface.h
  point3d.h
  sim-config.h
  torch-interfaces.h
  collision-listener-interface.h
)

target_compile_features(
  simcal
  PRIVATE 
  cxx_std_23
)

target_compile_features(
  simviz
  PRIVATE 
  cxx_std_23
)

target_compile_features(
  depsim
  PRIVATE 
  cxx_std_23
)


set_target_properties(depsim PROPERTIES LINKER_LANGUAGE CXX)

find_package(fmt REQUIRED)
target_link_libraries(depsim PRIVATE fmt::fmt)

find_package(Eigen3 REQUIRED)
target_link_libraries(depsim PUBLIC Eigen3::Eigen)
target_link_libraries(simviz PUBLIC depsim)
target_link_libraries(simcal PUBLIC depsim)

add_subdirectory(src)
add_subdirectory(examples/visualize)
add_subdirectory(examples/calibrate)

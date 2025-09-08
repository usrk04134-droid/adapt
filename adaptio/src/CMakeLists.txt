set(ADAPTIO_APPLICATION adaptio)

# Uncomment line below to build a debug Adaptio using PN Driver debug lib
#set(ADAPTIO_DEBUG_MODE "On")

add_executable(${ADAPTIO_APPLICATION} main.cc)
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Sanitizer shut off due too pylon. Investigate solution
  #target_compile_options(${ADAPTIO_APPLICATION} PRIVATE -Wall -Werror -pedantic -gdwarf-4 -fsanitize=address
  if (ADAPTIO_DEBUG_MODE STREQUAL "On")
    target_compile_options(${ADAPTIO_APPLICATION} PRIVATE -Wall -O0 -pedantic -gdwarf-4
          -ferror-limit=0)
  else ()
      target_compile_options(${ADAPTIO_APPLICATION} PRIVATE -Wall -Werror -pedantic -gdwarf-4
            -ferror-limit=0)
  endif ()
  # Sanitizer shut off due too pylon. Investigate solution
  #target_link_options(${ADAPTIO_APPLICATION} PRIVATE -v -fsanitize=address)
  target_link_options(${ADAPTIO_APPLICATION} PRIVATE -v -rdynamic)
else ()
  target_compile_options(${ADAPTIO_APPLICATION} PRIVATE -Wall -Werror -pedantic -gdwarf-4)
  target_link_options(${ADAPTIO_APPLICATION} PRIVATE -v -rdynamic)
endif ()
target_compile_features(${ADAPTIO_APPLICATION} PRIVATE cxx_std_23)
add_compile_definitions(BUILD_TYPE="${CMAKE_BUILD_TYPE}")

target_sources(${ADAPTIO_APPLICATION}
        PRIVATE
        version.h
)

set(UNIT_TESTS_APPLICATION adaptio-unit-tests)
add_executable(${UNIT_TESTS_APPLICATION} test_utils/unit_tests.cc)
target_compile_options(${UNIT_TESTS_APPLICATION} PRIVATE -Wall -Werror -pedantic -gdwarf-4
        -ferror-limit=0 -fsanitize=address,undefined)
target_link_options(${UNIT_TESTS_APPLICATION} PRIVATE -v -fsanitize=address,undefined)
target_compile_features(${UNIT_TESTS_APPLICATION} PRIVATE cxx_std_23)

set(BLOCK_TESTS_APPLICATION adaptio-block-tests)
add_executable(${BLOCK_TESTS_APPLICATION} test_utils/block_tests.cc)
target_compile_options(${BLOCK_TESTS_APPLICATION} PRIVATE -Wall -Werror -pedantic -gdwarf-4
        -ferror-limit=0 -fsanitize=address,undefined)
target_link_options(${BLOCK_TESTS_APPLICATION} PRIVATE -v -fsanitize=address,undefined)
target_compile_features(${BLOCK_TESTS_APPLICATION} PRIVATE cxx_std_23)

set(TEST_IMAGES_APPLICATION test-images)
add_executable(${TEST_IMAGES_APPLICATION} scanner/tools/test_image_set.cc)
target_compile_options(${TEST_IMAGES_APPLICATION} PRIVATE -Wall -Werror -pedantic -gdwarf-4
        -ferror-limit=0)
target_link_options(${TEST_IMAGES_APPLICATION} PRIVATE -v)
target_compile_features(${TEST_IMAGES_APPLICATION} PRIVATE cxx_std_23)

add_subdirectory(benchmarks)
add_subdirectory(block_tests)
add_subdirectory(common)
add_subdirectory(configuration)
add_subdirectory(controller)
add_subdirectory(main)
add_subdirectory(scanner)
add_subdirectory(test_utils)

get_target_property(ADAPTIO_APPLICATION_SOURCES ${ADAPTIO_APPLICATION} SOURCES)
string(REPLACE "main.cc" "" ADAPTIO_APPLICATION_SOURCES "${ADAPTIO_APPLICATION_SOURCES}")

target_sources(${UNIT_TESTS_APPLICATION} PRIVATE "${ADAPTIO_APPLICATION_SOURCES}")
target_include_directories(${UNIT_TESTS_APPLICATION} PRIVATE ${CMAKE_BINARY_DIR}/generated/)

target_sources(${BLOCK_TESTS_APPLICATION} PRIVATE "${ADAPTIO_APPLICATION_SOURCES}")
target_include_directories(${BLOCK_TESTS_APPLICATION} PRIVATE ${CMAKE_BINARY_DIR}/generated/)

target_sources(${TEST_IMAGES_APPLICATION} PRIVATE "${ADAPTIO_APPLICATION_SOURCES}")
target_include_directories(${TEST_IMAGES_APPLICATION} PRIVATE ${CMAKE_BINARY_DIR}/generated/)

find_package(PnDriver REQUIRED)
target_include_directories(${ADAPTIO_APPLICATION} PRIVATE ${PnDriver_INCLUDE_DIR})
target_include_directories(${UNIT_TESTS_APPLICATION} PRIVATE ${PnDriver_INCLUDE_DIR})
target_include_directories(${BLOCK_TESTS_APPLICATION} PRIVATE ${PnDriver_INCLUDE_DIR})
target_include_directories(${TEST_IMAGES_APPLICATION} PRIVATE ${PnDriver_INCLUDE_DIR})
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE ${PnDriver_LIBRARIES})
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE ${PnDriver_LIBRARIES})
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE ${PnDriver_LIBRARIES})
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE ${PnDriver_LIBRARIES})

find_package(Eigen3 REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE Eigen3::Eigen)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE Eigen3::Eigen)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE Eigen3::Eigen)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE Eigen3::Eigen)

find_package(Boost 1.81 COMPONENTS program_options context log_setup log stacktrace_basic REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE ${Boost_LIBRARIES})
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE ${Boost_LIBRARIES})
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE ${Boost_LIBRARIES})
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE ${Boost_LIBRARIES})

find_package(cppzmq REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE cppzmq)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE cppzmq)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE cppzmq)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE cppzmq)

find_package(OpenSSL REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE ssl crypto)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE ssl crypto)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE ssl crypto)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE ssl crypto)

find_package(fmt REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE fmt::fmt)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE fmt::fmt)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE fmt::fmt)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE fmt::fmt)

find_package(prometheus-cpp REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE prometheus-cpp::pull)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE prometheus-cpp::pull)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE prometheus-cpp::pull)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE prometheus-cpp::pull)

find_package(yaml-cpp 0.7.0 REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE yaml-cpp)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE yaml-cpp)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE yaml-cpp)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE yaml-cpp)

find_package(SQLiteCpp REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE SQLiteCpp)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE SQLiteCpp)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE SQLiteCpp)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE SQLiteCpp)

find_package(TIFF REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE TIFF::TIFF)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE TIFF::TIFF)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE TIFF::TIFF)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE TIFF::TIFF)

find_package(doctest REQUIRED)
target_include_directories(${UNIT_TESTS_APPLICATION} PUBLIC ${DOCTEST_INCLUDE_DIR})
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE doctest::doctest)
target_include_directories(${BLOCK_TESTS_APPLICATION} PUBLIC ${DOCTEST_INCLUDE_DIR})
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE doctest::doctest)
target_include_directories(${TEST_IMAGES_APPLICATION} PUBLIC ${DOCTEST_INCLUDE_DIR})
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE doctest::doctest)

find_package(trompeloeil REQUIRED)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE trompeloeil::trompeloeil)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE trompeloeil::trompeloeil)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE depsim)
target_include_directories(${BLOCK_TESTS_APPLICATION} PUBLIC common/deposition_simulator)

find_package(Pylon REQUIRED)
target_include_directories(${ADAPTIO_APPLICATION} PRIVATE ${Pylon_INCLUDE_DIR})
target_include_directories(${UNIT_TESTS_APPLICATION} PRIVATE ${Pylon_INCLUDE_DIR})
target_include_directories(${BLOCK_TESTS_APPLICATION} PRIVATE ${Pylon_INCLUDE_DIR})
target_include_directories(${TEST_IMAGES_APPLICATION} PRIVATE ${Pylon_INCLUDE_DIR})
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE ${Pylon_LIBRARIES})
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE ${Pylon_LIBRARIES})
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE ${Pylon_LIBRARIES})
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE ${Pylon_LIBRARIES})

find_package(OpenCV 4.7 REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE ${OpenCV_LIBRARIES})
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE ${OpenCV_LIBRARIES})
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE ${OpenCV_LIBRARIES})
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE ${OpenCV_LIBRARIES})
add_definitions(${OPENCV_DEFINITIONS})

find_package(Ceres REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE Ceres::ceres)
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE Ceres::ceres)
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE Ceres::ceres)
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE Ceres::ceres)

find_package(PCL 1.13 COMPONENTS sample_consensus REQUIRED)
target_link_libraries(${ADAPTIO_APPLICATION} PRIVATE ${PCL_LIBRARIES})
target_link_libraries(${UNIT_TESTS_APPLICATION} PRIVATE ${PCL_LIBRARIES})
target_link_libraries(${BLOCK_TESTS_APPLICATION} PRIVATE ${PCL_LIBRARIES})
target_link_libraries(${TEST_IMAGES_APPLICATION} PRIVATE ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../assets/basler/a2A4096-9gmPRO_40297732.pfs PFS_FILE_CONTENT)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scanner/image_provider/basler_camera.cc ${CMAKE_CURRENT_SOURCE_DIR}/../build/basler_camera_with_pfs.cc)

configure_file(${PROJECT_SOURCE_DIR}/src/version.h.in ${PROJECT_BINARY_DIR}/src/version.h)

install(TARGETS ${ADAPTIO_APPLICATION} DESTINATION bin)
install(DIRECTORY ${CMAKE_BINARY_DIR}/generated/assets/ DESTINATION assets)
install(DIRECTORY ../assets/configuration DESTINATION assets)
install(DIRECTORY ../assets/events DESTINATION assets)
install(DIRECTORY ../assets/scanner_calibration DESTINATION assets)

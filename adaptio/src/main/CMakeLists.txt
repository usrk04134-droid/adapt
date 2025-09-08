add_subdirectory(bead_control)
add_subdirectory(calibration)
add_subdirectory(cli_handler)
add_subdirectory(coordination)
add_subdirectory(event_handler)
add_subdirectory(image_logging)
add_subdirectory(joint_geometry)
add_subdirectory(kinematics)
add_subdirectory(lpcs)
add_subdirectory(macs)
add_subdirectory(management)
add_subdirectory(scanner_client)
add_subdirectory(slice_translator)
add_subdirectory(tracking)
add_subdirectory(web_hmi)
add_subdirectory(weld_control)
add_subdirectory(weld_system_client)

target_include_directories(${ADAPTIO_APPLICATION} PRIVATE .)
target_include_directories(${UNIT_TESTS_APPLICATION} PRIVATE .)
target_include_directories(${BLOCK_TESTS_APPLICATION} PRIVATE .)
target_include_directories(${TEST_IMAGES_APPLICATION} PRIVATE .)

target_sources(${ADAPTIO_APPLICATION}
        PRIVATE
        application.h

        application.cc
)

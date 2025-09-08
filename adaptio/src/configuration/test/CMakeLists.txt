target_sources(${UNIT_TESTS_APPLICATION}
    PRIVATE

    test_controller_converter.cc
    test_image_provider_converter.cc
    test_scanner_converter.cc
    test_scanner_calibration_converter.cc
    test_config_manager.cc
    test_calib_fix_joint_geometry.cc
    test_tolerances_converter.cc
)

add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    config_manager.h
    config_manager_impl.h
    controller_config_converter.h
    scanner_calibration_converter.h
    scanner_converter.h
    factory.h
    file_handler.h
    calib_fix_joint_geometry_converter.h
    configuration_error.h
    weld_control_converter.h
    calibration_converter.h
    image_logging_converter.h

    config_manager_impl.cc
    scanner_calibration_converter.cc
    factory.cc
    file_handler.cc
    configuration_error.cc
)


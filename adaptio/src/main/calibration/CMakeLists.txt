target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    src/calibration_metrics.h
    calibration_configuration.h
    src/calibration_solver.h
    src/calibration_solver_impl.h
    src/calibration_manager_impl.h
    src/calibration_grid_generator.h
    src/stored_laser_torch_configuration.h
    src/stored_calibration_result.h
    src/calibration_sequence_runner.h
    src/calibration_handler.h
    src/cw_calibration_handler.h
    src/lw_calibration_handler.h
    src/stored_lw_cal_result.h
    
    src/calibration_metrics.cc
    src/calibration_solver_impl.cc
    src/calibration_manager_impl.cc
    src/stored_laser_torch_configuration.cc
    src/stored_calibration_result.cc
    src/calibration_sequence_runner.cc
    src/cw_calibration_handler.cc
    src/lw_calibration_handler.cc
    src/stored_lw_cal_result.cc
)


add_subdirectory(test)

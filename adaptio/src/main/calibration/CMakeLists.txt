target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    calibration_manager.h
    calibration_metrics.h
    calibration_types.h
    calibration_configuration.h
    src/calibration_solver.h
    src/calibration_solver_impl.h
    src/calibration_manager_v2_impl.h
    src/calibration_grid_generator.h
    src/stored_laser_torch_configuration.h
    src/stored_calibration_result.h
    src/calibration_sequence_runner.h
    
    calibration_metrics.cc
    src/calibration_solver_impl.cc
    src/calibration_manager_v2_impl.cc
    src/stored_laser_torch_configuration.cc
    src/stored_calibration_result.cc
    src/calibration_sequence_runner.cc
)


add_subdirectory(test)

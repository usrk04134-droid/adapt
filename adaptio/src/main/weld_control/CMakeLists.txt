add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    weld_control.h
    weld_control_types.h
    weld_state_observer.h

    src/weld_control_impl.h
    src/weld_data_set.h
    src/weld_program.h
    src/abp_parameters.h
    src/delay_buffer.h
    src/weld_process_parameters.h
    src/weld_calculations.h
    src/sql_table_names.h
    src/weld_sequence_config.h
    src/weld_sequence_config_impl.h
    
    src/weld_metrics.h
    src/settings.h
    src/settings_provider.h
    src/confident_slice_buffer.h

    src/weld_control_impl.cc
    src/weld_data_set.cc
    src/weld_process_parameters.cc
    src/abp_parameters.cc
    src/weld_program.cc
    src/weld_sequence_config_impl.cc
    
    src/weld_metrics.cc
    src/settings.cc
    src/settings_provider.cc
  )

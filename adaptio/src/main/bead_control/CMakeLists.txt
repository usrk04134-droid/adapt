add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    bead_control.h
    bead_control_types.h

    src/bead_control_impl.h
    src/bead_calculations.h
    src/weld_position_data_storage.h

    src/bead_control_impl.cc
    src/bead_calculations.cc
  )

add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    relative_position_buffer.h
    position_buffer.h
    slot_buffer.h
    resizable_slot_buffer.h
)

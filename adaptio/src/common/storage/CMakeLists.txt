target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    single_storage.h
    multi_storage.h
    sql_single_storage.h
    sql_multi_storage.h
    slot_buffer_storage.h
)

add_subdirectory(test)

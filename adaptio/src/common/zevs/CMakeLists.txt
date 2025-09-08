add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    zevs_core.h
    zevs_socket.h
    zevs_adapter.h
    zevs_test_support.h
)

add_subdirectory(src)

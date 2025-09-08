add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    big_snake.h
    naive.h
    joint_model.h
    slice.h
    snake.h

    big_snake.cc
    naive.cc
    joint_model.cc
    slice.cc
    snake.cc
)

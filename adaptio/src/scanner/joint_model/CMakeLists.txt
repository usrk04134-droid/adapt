add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    big_snake.h
    joint_model.h
    src/naive.h
    src/slice.h
    src/snake.h

    src/big_snake.cc
    src/naive.cc
    src/joint_model.cc
    src/slice.cc
    src/snake.cc
)

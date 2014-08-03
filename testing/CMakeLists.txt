set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(decoder_tests
    decoder_tests.cpp
    seq_gen.hpp
    test_data_seq.hpp
)

add_subdirectory(generator)

cmake_minimum_required(VERSION 3.0)
project(lzma-cpp)

if (WIN32)
    add_definitions(-DUNICODE -D_UNICODE)
endif()

if(MSVC)
    add_definitions(/W4 /WX)
else()
    add_definitions(-std=c++11 -Wall)
endif()

include_directories(include)

add_subdirectory(testing)

cmake_minimum_required(VERSION 3.29)
project(combine_algs)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.5.0
)
FetchContent_MakeAvailable(Catch2)

add_subdirectory(lab2)
add_subdirectory(lab3)
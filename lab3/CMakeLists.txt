cmake_minimum_required(VERSION 3.16)
project(triangulation)

set(CMAKE_CXX_STANDARD 20)

set(SFML_DIR "/opt/homebrew/Cellar/sfml/3.0.0_1/lib/cmake/SFML")
set(OpenCV_DIR "/opt/homebrew/opt/opencv/share/opencv4")

add_executable(triangulation
        edge/edge.h
        edge/edge.cpp
        verge/verge.h
        verge/verge.cpp
        vertex/vertex.h
        vertex/vertex.cpp
        triangulation/triangulation.h
        triangulation/triangulation.cpp
        main.cpp)
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)
find_package(OpenCV REQUIRED)
include_directories(${SFML_INCLUDE_DIRS})

target_link_libraries(triangulation
        SFML::System
        SFML::Window
        SFML::Graphics
)
target_link_libraries(triangulation ${OpenCV_LIBS})

cmake_minimum_required(VERSION 3.15)
project(vjezbanje)

set(CMAKE_CXX_STANDARD17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONSOFF)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Main src/main.cpp)
target_link_libraries(Main ${OpenCV_LIBS})
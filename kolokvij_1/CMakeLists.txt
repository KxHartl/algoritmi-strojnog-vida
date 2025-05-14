cmake_minimum_required(VERSION 3.5)
project(kolokvij_1)

set(CMAKE_CXX_STANDARD17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONSOFF)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Zadatak2 src/zadatak_2.cpp)
target_link_libraries(Zadatak2 ${OpenCV_LIBS})
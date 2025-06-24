cmake_minimum_required(VERSION 3.5)
project(OpenCV3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(OpenCV3 src/vj3.cpp)
target_link_libraries(OpenCV3 ${OpenCV_LIBS} )

add_executable(test src/test.cpp)
target_link_libraries(test ${OpenCV_LIBS} )

add_executable(Zadatak3 src/zadatak3.cpp)
target_link_libraries(Zadatak3 ${OpenCV_LIBS} )

add_executable(Zadatak1 src/zadatak1.cpp)
target_link_libraries(Zadatak1 ${OpenCV_LIBS} )

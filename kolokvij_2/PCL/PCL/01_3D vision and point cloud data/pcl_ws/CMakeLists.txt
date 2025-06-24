cmake_minimum_required(VERSION 3.5)
project(GeneratePointCloud)

# Nađi PCL biblioteku
find_package(PCL 1.10 REQUIRED)

# Uključi zaglavlja i biblioteke
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Dodaj izvršnu datoteku i poveži s PCL-om
add_executable(generate_point_cloud src/p_01_generiranje_PC.cpp)
target_link_libraries(generate_point_cloud ${PCL_LIBRARIES})

add_executable(pointer src/p_02_pointer.cpp)
target_link_libraries(pointer ${PCL_LIBRARIES})

add_executable(citanje_pc src/z_01_citanje_pc.cpp)
target_link_libraries(citanje_pc ${PCL_LIBRARIES})

add_executable(dimenzije src/p_03_dimenzije_PC.cpp)
target_link_libraries(dimenzije ${PCL_LIBRARIES})

add_executable(pass_through_primjer src/p_04_pass_through_primjer.cpp)
target_link_libraries(pass_through_primjer ${PCL_LIBRARIES})

add_executable(statistical_outlier_primjer src/p_05_statistical_outlier_primjer.cpp)
target_link_libraries(statistical_outlier_primjer ${PCL_LIBRARIES})

add_executable(vokselizacija_primjer src/p_06_vokselizacija_pc.cpp)
target_link_libraries(vokselizacija_primjer ${PCL_LIBRARIES})


# # Zadatak 1: čitanje point clouda
# add_executable(citanje_pc_z1 src/z_01_citanje_pc.cpp)
# target_link_libraries(citanje_pc_z1 ${PCL_LIBRARIES})

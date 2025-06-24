cmake_minimum_required(VERSION 3.5)
project(GeneratePointCloud)

# Nađi PCL biblioteku
find_package(PCL 1.10 REQUIRED)

# Uključi zaglavlja i biblioteke
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable datoteke s originalnim nazivima
add_executable(p_04_1_pass_through_primjer src/p_04_1_pass_through_primjer.cpp)
target_link_libraries(p_04_1_pass_through_primjer ${PCL_LIBRARIES})

add_executable(p_04_2_pass_through_primjer src/p_04_2_pass_through_primjer.cpp)
target_link_libraries(p_04_2_pass_through_primjer ${PCL_LIBRARIES})

add_executable(p_04_pass_through_primjer src/p_04_pass_through_primjer.cpp)
target_link_libraries(p_04_pass_through_primjer ${PCL_LIBRARIES})

add_executable(p_05_statistical_outlier_primjer src/p_05_statistical_outlier_primjer.cpp)
target_link_libraries(p_05_statistical_outlier_primjer ${PCL_LIBRARIES})

add_executable(p_06_vokselizacija_pc src/p_06_vokselizacija_pc.cpp)
target_link_libraries(p_06_vokselizacija_pc ${PCL_LIBRARIES})

add_executable(p_07_ransac_plane src/p_07_ransac_plane.cpp)
target_link_libraries(p_07_ransac_plane ${PCL_LIBRARIES})
    
add_executable(p_08_ransac src/p_08_ransac.cpp)
target_link_libraries(p_08_ransac ${PCL_LIBRARIES})

add_executable(p_09_ciscenje src/p_09_ciscenje.cpp)
target_link_libraries(p_09_ciscenje ${PCL_LIBRARIES})

add_executable(p_10_klasterizacija src/p_10_klasterizacija.cpp)
target_link_libraries(p_10_klasterizacija ${PCL_LIBRARIES})

add_executable(p_11_transformacija_PC src/p_11_transformacija_PC.cpp)
target_link_libraries(p_11_transformacija_PC ${PCL_LIBRARIES})

add_executable(p_12_ICP src/p_12_ICP.cpp)
target_link_libraries(p_12_ICP ${PCL_LIBRARIES})

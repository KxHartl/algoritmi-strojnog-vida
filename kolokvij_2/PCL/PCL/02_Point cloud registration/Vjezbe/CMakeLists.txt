cmake_minimum_required(VERSION 3.20...3.26 FATAL_ERROR)
project(kfpcs_registration)

# Set CMake policies to avoid warnings
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Update CMAKE_PREFIX_PATH to include installation paths
# list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake")
# list(APPEND CMAKE_PREFIX_PATH "/home/roc42/Qt/6.8.0/gcc_64")

# Find Qt6
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Check if PCL_DIR exists before setting it
# if(NOT DEFINED PCL_DIR)
#   set(PCL_DIR "/usr/local/share/pcl-1.14")
# endif()

# Find PCL with necessary components
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization registration)

# Find Boost thread library
find_package(Boost REQUIRED COMPONENTS thread)

# Find threading support
find_package(Threads REQUIRED)

# Find Super4PCS
# find_library(SUPER4PCS_ALGO_LIB libsuper4pcs_algo.a PATHS /usr/local/lib REQUIRED)
# find_library(SUPER4PCS_IO_LIB libsuper4pcs_io.a PATHS /usr/local/lib REQUIRED)
# find_path(SUPER4PCS_INCLUDE_DIR super4pcs/shared4pcs.h PATHS /usr/local/include REQUIRED)

# Find Go-ICP
# find_library(GOICP_LIB NAMES goicpcore goicp PATHS /usr/local/lib)
# find_path(GOICP_INCLUDE_DIR NAMES jly_goicp.h PATHS /usr/local/include/goicp)
# if(NOT GOICP_LIB OR NOT GOICP_INCLUDE_DIR)
#   message(FATAL_ERROR "Go-ICP library or headers not found. Please install Go-ICP to /usr/local.")
# endif()

# Collect include directories
set(ADDITIONAL_INCLUDE_DIRS
    ${PCL_INCLUDE_DIRS}
    # ${SUPER4PCS_INCLUDE_DIR}
    # ${GOICP_INCLUDE_DIR}
)

# Define a macro to simplify the addition of executables
macro(add_pcl_executable target_name source_file)
  add_executable(${target_name} ${source_file})
  target_include_directories(${target_name} PRIVATE ${ADDITIONAL_INCLUDE_DIRS})
  target_link_libraries(${target_name}
      PRIVATE
          ${PCL_LIBRARIES}
          Qt5::Core
          Qt5::Widgets
          Boost::thread
          Threads::Threads
          # ${SUPER4PCS_ALGO_LIB}
          # ${SUPER4PCS_IO_LIB}
          # ${GOICP_LIB}
  )
  set_target_properties(${target_name} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
endmacro()

# Add executables
# add_pcl_executable(fpcs_registration fpcs_registration.cpp)
# add_pcl_executable(pcd_viewer pcd_viewer.cpp)
add_pcl_executable(pcd_transformation src/pcd_transformation.cpp)
# add_pcl_executable(pcd_transformation_color pcd_transformation_color.cpp)
add_pcl_executable(kfpcs_registration src/kfpcs_registration.cpp)
# add_pcl_executable(kfpcs_color kfpcs_color.cpp)
# add_pcl_executable(kfpcs_multiple kfpcs_multiple.cpp)
add_pcl_executable(downsampling_voxels src/downsampling_voxels.cpp)
# add_pcl_executable(super4pcs_registration super4pcs_registration.cpp)
add_pcl_executable(cutting_pcd src/cutting_pcd.cpp)
# add_pcl_executable(theoretical_matrix_calculation theoretical_matrix_calculation.cpp)
# add_pcl_executable(gauss gauss.cpp)
add_pcl_executable(bounding_box src/bounding_box.cpp)
# add_pcl_executable(pcd_scaling pcd_scaling.cpp)
# add_pcl_executable(super4pcs_iter super4pcs_iter.cpp)
# add_pcl_executable(GO-ICP GO-ICP.cpp)
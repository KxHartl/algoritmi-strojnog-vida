# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/build"

# Include any dependencies generated for this target.
include CMakeFiles/cutting_pcd.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/cutting_pcd.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/cutting_pcd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cutting_pcd.dir/flags.make

CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o: CMakeFiles/cutting_pcd.dir/flags.make
CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o: /home/ivan/Desktop/Algoritmi\ strojnog\ vida/Kol_2/02_Point\ cloud\ registration/Vjezbe/src/cutting_pcd.cpp
CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o: CMakeFiles/cutting_pcd.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o -MF CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o.d -o CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o -c "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/src/cutting_pcd.cpp"

CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/src/cutting_pcd.cpp" > CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.i

CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/src/cutting_pcd.cpp" -o CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.s

# Object files for target cutting_pcd
cutting_pcd_OBJECTS = \
"CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o"

# External object files for target cutting_pcd
cutting_pcd_EXTERNAL_OBJECTS =

cutting_pcd: CMakeFiles/cutting_pcd.dir/src/cutting_pcd.cpp.o
cutting_pcd: CMakeFiles/cutting_pcd.dir/build.make
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
cutting_pcd: /usr/lib/libOpenNI.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libflann_cpp.so.1.9.2
cutting_pcd: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.83.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_io.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpng.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libz.so
cutting_pcd: /usr/lib/libOpenNI.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_features.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_search.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpcl_common.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.83.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.83.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.83.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.83.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOCore-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libfreetype.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOImage-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingUI-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkkissfft-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libGLEW.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libX11.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.15.13
cutting_pcd: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.13
cutting_pcd: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.13
cutting_pcd: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.13
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-9.1.so.9.1.0
cutting_pcd: /usr/lib/x86_64-linux-gnu/libtbb.so.12.11
cutting_pcd: /usr/lib/x86_64-linux-gnu/libvtksys-9.1.so.9.1.0
cutting_pcd: /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libpthread.a
cutting_pcd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
cutting_pcd: /usr/lib/x86_64-linux-gnu/liblz4.so
cutting_pcd: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.83.0
cutting_pcd: CMakeFiles/cutting_pcd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir="/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cutting_pcd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cutting_pcd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cutting_pcd.dir/build: cutting_pcd
.PHONY : CMakeFiles/cutting_pcd.dir/build

CMakeFiles/cutting_pcd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cutting_pcd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cutting_pcd.dir/clean

CMakeFiles/cutting_pcd.dir/depend:
	cd "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe" "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe" "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/build" "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/build" "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/build/CMakeFiles/cutting_pcd.dir/DependInfo.cmake" "--color=$(COLOR)"
.PHONY : CMakeFiles/cutting_pcd.dir/depend


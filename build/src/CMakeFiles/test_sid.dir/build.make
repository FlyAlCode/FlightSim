# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/li/KDE/Li_Project/FlightSim/FlightSim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/li/KDE/Li_Project/FlightSim/FlightSim/build

# Include any dependencies generated for this target.
include src/CMakeFiles/test_sid.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/test_sid.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/test_sid.dir/flags.make

src/CMakeFiles/test_sid.dir/test/test_sid.cc.o: src/CMakeFiles/test_sid.dir/flags.make
src/CMakeFiles/test_sid.dir/test/test_sid.cc.o: ../src/test/test_sid.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/li/KDE/Li_Project/FlightSim/FlightSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/test_sid.dir/test/test_sid.cc.o"
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_sid.dir/test/test_sid.cc.o -c /home/li/KDE/Li_Project/FlightSim/FlightSim/src/test/test_sid.cc

src/CMakeFiles/test_sid.dir/test/test_sid.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_sid.dir/test/test_sid.cc.i"
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/li/KDE/Li_Project/FlightSim/FlightSim/src/test/test_sid.cc > CMakeFiles/test_sid.dir/test/test_sid.cc.i

src/CMakeFiles/test_sid.dir/test/test_sid.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_sid.dir/test/test_sid.cc.s"
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/li/KDE/Li_Project/FlightSim/FlightSim/src/test/test_sid.cc -o CMakeFiles/test_sid.dir/test/test_sid.cc.s

src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.requires:

.PHONY : src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.requires

src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.provides: src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.requires
	$(MAKE) -f src/CMakeFiles/test_sid.dir/build.make src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.provides.build
.PHONY : src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.provides

src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.provides.build: src/CMakeFiles/test_sid.dir/test/test_sid.cc.o


src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o: src/CMakeFiles/test_sid.dir/flags.make
src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o: ../src/mrsid/sid.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/li/KDE/Li_Project/FlightSim/FlightSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o"
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_sid.dir/mrsid/sid.cc.o -c /home/li/KDE/Li_Project/FlightSim/FlightSim/src/mrsid/sid.cc

src/CMakeFiles/test_sid.dir/mrsid/sid.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_sid.dir/mrsid/sid.cc.i"
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/li/KDE/Li_Project/FlightSim/FlightSim/src/mrsid/sid.cc > CMakeFiles/test_sid.dir/mrsid/sid.cc.i

src/CMakeFiles/test_sid.dir/mrsid/sid.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_sid.dir/mrsid/sid.cc.s"
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/li/KDE/Li_Project/FlightSim/FlightSim/src/mrsid/sid.cc -o CMakeFiles/test_sid.dir/mrsid/sid.cc.s

src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.requires:

.PHONY : src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.requires

src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.provides: src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.requires
	$(MAKE) -f src/CMakeFiles/test_sid.dir/build.make src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.provides.build
.PHONY : src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.provides

src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.provides.build: src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o


# Object files for target test_sid
test_sid_OBJECTS = \
"CMakeFiles/test_sid.dir/test/test_sid.cc.o" \
"CMakeFiles/test_sid.dir/mrsid/sid.cc.o"

# External object files for target test_sid
test_sid_EXTERNAL_OBJECTS =

../bin/test_sid: src/CMakeFiles/test_sid.dir/test/test_sid.cc.o
../bin/test_sid: src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o
../bin/test_sid: src/CMakeFiles/test_sid.dir/build.make
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
../bin/test_sid: ../third_lib/lib/mrsid/libltidsdk.so
../bin/test_sid: ../third_lib/lib/mrsid/libltidsdk.so.9
../bin/test_sid: ../third_lib/lib/mrsid/libtbb.so
../bin/test_sid: ../third_lib/lib/mrsid/libtbb.so.2
../bin/test_sid: ../third_lib/lib/shp/libshp.so
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
../bin/test_sid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
../bin/test_sid: src/CMakeFiles/test_sid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/li/KDE/Li_Project/FlightSim/FlightSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/test_sid"
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_sid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/test_sid.dir/build: ../bin/test_sid

.PHONY : src/CMakeFiles/test_sid.dir/build

src/CMakeFiles/test_sid.dir/requires: src/CMakeFiles/test_sid.dir/test/test_sid.cc.o.requires
src/CMakeFiles/test_sid.dir/requires: src/CMakeFiles/test_sid.dir/mrsid/sid.cc.o.requires

.PHONY : src/CMakeFiles/test_sid.dir/requires

src/CMakeFiles/test_sid.dir/clean:
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src && $(CMAKE_COMMAND) -P CMakeFiles/test_sid.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/test_sid.dir/clean

src/CMakeFiles/test_sid.dir/depend:
	cd /home/li/KDE/Li_Project/FlightSim/FlightSim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/li/KDE/Li_Project/FlightSim/FlightSim /home/li/KDE/Li_Project/FlightSim/FlightSim/src /home/li/KDE/Li_Project/FlightSim/FlightSim/build /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src /home/li/KDE/Li_Project/FlightSim/FlightSim/build/src/CMakeFiles/test_sid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/test_sid.dir/depend


# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/drone_assets

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/csanrod/TFG/2022-tfg-cristian-sanchez/build/drone_assets

# Include any dependencies generated for this target.
include CMakeFiles/wall3plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/wall3plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/wall3plugin.dir/flags.make

CMakeFiles/wall3plugin.dir/plugins/wall3.cc.o: CMakeFiles/wall3plugin.dir/flags.make
CMakeFiles/wall3plugin.dir/plugins/wall3.cc.o: /home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/drone_assets/plugins/wall3.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/csanrod/TFG/2022-tfg-cristian-sanchez/build/drone_assets/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/wall3plugin.dir/plugins/wall3.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/wall3plugin.dir/plugins/wall3.cc.o -c /home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/drone_assets/plugins/wall3.cc

CMakeFiles/wall3plugin.dir/plugins/wall3.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/wall3plugin.dir/plugins/wall3.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/drone_assets/plugins/wall3.cc > CMakeFiles/wall3plugin.dir/plugins/wall3.cc.i

CMakeFiles/wall3plugin.dir/plugins/wall3.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/wall3plugin.dir/plugins/wall3.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/drone_assets/plugins/wall3.cc -o CMakeFiles/wall3plugin.dir/plugins/wall3.cc.s

# Object files for target wall3plugin
wall3plugin_OBJECTS = \
"CMakeFiles/wall3plugin.dir/plugins/wall3.cc.o"

# External object files for target wall3plugin
wall3plugin_EXTERNAL_OBJECTS =

/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: CMakeFiles/wall3plugin.dir/plugins/wall3.cc.o
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: CMakeFiles/wall3plugin.dir/build.make
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libgazebo_ros_api_plugin.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libgazebo_ros_paths_plugin.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libroslib.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/librospack.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libtf.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libtf2_ros.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libactionlib.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libmessage_filters.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libroscpp.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libtf2.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/librosconsole.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/librostime.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /opt/ros/noetic/lib/libcpp_common.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.7.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libccd.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libfcl.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.2.1
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.4.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.9.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.10.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.0
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so: CMakeFiles/wall3plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/csanrod/TFG/2022-tfg-cristian-sanchez/build/drone_assets/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/wall3plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/wall3plugin.dir/build: /home/csanrod/TFG/2022-tfg-cristian-sanchez/devel/.private/drone_assets/lib/libwall3plugin.so

.PHONY : CMakeFiles/wall3plugin.dir/build

CMakeFiles/wall3plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/wall3plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/wall3plugin.dir/clean

CMakeFiles/wall3plugin.dir/depend:
	cd /home/csanrod/TFG/2022-tfg-cristian-sanchez/build/drone_assets && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/drone_assets /home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/drone_assets /home/csanrod/TFG/2022-tfg-cristian-sanchez/build/drone_assets /home/csanrod/TFG/2022-tfg-cristian-sanchez/build/drone_assets /home/csanrod/TFG/2022-tfg-cristian-sanchez/build/drone_assets/CMakeFiles/wall3plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/wall3plugin.dir/depend


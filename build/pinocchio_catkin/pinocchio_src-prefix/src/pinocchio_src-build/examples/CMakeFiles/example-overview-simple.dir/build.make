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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build

# Include any dependencies generated for this target.
include examples/CMakeFiles/example-overview-simple.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example-overview-simple.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example-overview-simple.dir/flags.make

examples/CMakeFiles/example-overview-simple.dir/overview-simple.cpp.o: examples/CMakeFiles/example-overview-simple.dir/flags.make
examples/CMakeFiles/example-overview-simple.dir/overview-simple.cpp.o: /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/overview-simple.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/example-overview-simple.dir/overview-simple.cpp.o"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example-overview-simple.dir/overview-simple.cpp.o -c /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/overview-simple.cpp

examples/CMakeFiles/example-overview-simple.dir/overview-simple.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-overview-simple.dir/overview-simple.cpp.i"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/overview-simple.cpp > CMakeFiles/example-overview-simple.dir/overview-simple.cpp.i

examples/CMakeFiles/example-overview-simple.dir/overview-simple.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-overview-simple.dir/overview-simple.cpp.s"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/overview-simple.cpp -o CMakeFiles/example-overview-simple.dir/overview-simple.cpp.s

# Object files for target example-overview-simple
example__overview__simple_OBJECTS = \
"CMakeFiles/example-overview-simple.dir/overview-simple.cpp.o"

# External object files for target example-overview-simple
example__overview__simple_EXTERNAL_OBJECTS =

examples/example-overview-simple: examples/CMakeFiles/example-overview-simple.dir/overview-simple.cpp.o
examples/example-overview-simple: examples/CMakeFiles/example-overview-simple.dir/build.make
examples/example-overview-simple: examples/CMakeFiles/example-overview-simple.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example-overview-simple"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-overview-simple.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example-overview-simple.dir/build: examples/example-overview-simple

.PHONY : examples/CMakeFiles/example-overview-simple.dir/build

examples/CMakeFiles/example-overview-simple.dir/clean:
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples && $(CMAKE_COMMAND) -P CMakeFiles/example-overview-simple.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example-overview-simple.dir/clean

examples/CMakeFiles/example-overview-simple.dir/depend:
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples/CMakeFiles/example-overview-simple.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example-overview-simple.dir/depend

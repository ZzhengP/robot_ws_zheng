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
include unittest/CMakeFiles/value.dir/depend.make

# Include the progress variables for this target.
include unittest/CMakeFiles/value.dir/progress.make

# Include the compile flags for this target's objects.
include unittest/CMakeFiles/value.dir/flags.make

unittest/CMakeFiles/value.dir/value.cpp.o: unittest/CMakeFiles/value.dir/flags.make
unittest/CMakeFiles/value.dir/value.cpp.o: /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/value.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object unittest/CMakeFiles/value.dir/value.cpp.o"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/value.dir/value.cpp.o -c /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/value.cpp

unittest/CMakeFiles/value.dir/value.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/value.dir/value.cpp.i"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/value.cpp > CMakeFiles/value.dir/value.cpp.i

unittest/CMakeFiles/value.dir/value.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/value.dir/value.cpp.s"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/value.cpp -o CMakeFiles/value.dir/value.cpp.s

# Object files for target value
value_OBJECTS = \
"CMakeFiles/value.dir/value.cpp.o"

# External object files for target value
value_EXTERNAL_OBJECTS =

unittest/value: unittest/CMakeFiles/value.dir/value.cpp.o
unittest/value: unittest/CMakeFiles/value.dir/build.make
unittest/value: src/libpinocchio.so
unittest/value: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
unittest/value: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
unittest/value: /usr/lib/x86_64-linux-gnu/libboost_system.so
unittest/value: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
unittest/value: unittest/CMakeFiles/value.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable value"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/value.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
unittest/CMakeFiles/value.dir/build: unittest/value

.PHONY : unittest/CMakeFiles/value.dir/build

unittest/CMakeFiles/value.dir/clean:
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && $(CMAKE_COMMAND) -P CMakeFiles/value.dir/cmake_clean.cmake
.PHONY : unittest/CMakeFiles/value.dir/clean

unittest/CMakeFiles/value.dir/depend:
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest/CMakeFiles/value.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unittest/CMakeFiles/value.dir/depend

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
include unittest/CMakeFiles/contact-dynamics-derivatives.dir/depend.make

# Include the progress variables for this target.
include unittest/CMakeFiles/contact-dynamics-derivatives.dir/progress.make

# Include the compile flags for this target's objects.
include unittest/CMakeFiles/contact-dynamics-derivatives.dir/flags.make

unittest/CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.o: unittest/CMakeFiles/contact-dynamics-derivatives.dir/flags.make
unittest/CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.o: /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/contact-dynamics-derivatives.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object unittest/CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.o"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.o -c /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/contact-dynamics-derivatives.cpp

unittest/CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.i"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/contact-dynamics-derivatives.cpp > CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.i

unittest/CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.s"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest/contact-dynamics-derivatives.cpp -o CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.s

# Object files for target contact-dynamics-derivatives
contact__dynamics__derivatives_OBJECTS = \
"CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.o"

# External object files for target contact-dynamics-derivatives
contact__dynamics__derivatives_EXTERNAL_OBJECTS =

unittest/contact-dynamics-derivatives: unittest/CMakeFiles/contact-dynamics-derivatives.dir/contact-dynamics-derivatives.cpp.o
unittest/contact-dynamics-derivatives: unittest/CMakeFiles/contact-dynamics-derivatives.dir/build.make
unittest/contact-dynamics-derivatives: src/libpinocchio.so
unittest/contact-dynamics-derivatives: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
unittest/contact-dynamics-derivatives: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
unittest/contact-dynamics-derivatives: /usr/lib/x86_64-linux-gnu/libboost_system.so
unittest/contact-dynamics-derivatives: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
unittest/contact-dynamics-derivatives: unittest/CMakeFiles/contact-dynamics-derivatives.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable contact-dynamics-derivatives"
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/contact-dynamics-derivatives.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
unittest/CMakeFiles/contact-dynamics-derivatives.dir/build: unittest/contact-dynamics-derivatives

.PHONY : unittest/CMakeFiles/contact-dynamics-derivatives.dir/build

unittest/CMakeFiles/contact-dynamics-derivatives.dir/clean:
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest && $(CMAKE_COMMAND) -P CMakeFiles/contact-dynamics-derivatives.dir/cmake_clean.cmake
.PHONY : unittest/CMakeFiles/contact-dynamics-derivatives.dir/clean

unittest/CMakeFiles/contact-dynamics-derivatives.dir/depend:
	cd /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/unittest /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/unittest/CMakeFiles/contact-dynamics-derivatives.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unittest/CMakeFiles/contact-dynamics-derivatives.dir/depend


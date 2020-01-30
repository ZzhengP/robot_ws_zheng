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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zheng/robot_ws_zheng/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zheng/robot_ws_zheng/build

# Include any dependencies generated for this target.
include orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/depend.make

# Include the progress variables for this target.
include orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/progress.make

# Include the compile flags for this target's objects.
include orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/flags.make

orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.o: orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/flags.make
orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.o: /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/src/DummyDynamicsComputations.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zheng/robot_ws_zheng/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.o"
	cd /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.o -c /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/src/DummyDynamicsComputations.cpp

orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.i"
	cd /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/src/DummyDynamicsComputations.cpp > CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.i

orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.s"
	cd /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/src/DummyDynamicsComputations.cpp -o CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.s

# Object files for target idyntree-high-level-kdl
idyntree__high__level__kdl_OBJECTS = \
"CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.o"

# External object files for target idyntree-high-level-kdl
idyntree__high__level__kdl_EXTERNAL_OBJECTS =

lib/libidyntree-high-level-kdl.a: orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/src/DummyDynamicsComputations.cpp.o
lib/libidyntree-high-level-kdl.a: orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/build.make
lib/libidyntree-high-level-kdl.a: orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zheng/robot_ws_zheng/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../../../../lib/libidyntree-high-level-kdl.a"
	cd /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl && $(CMAKE_COMMAND) -P CMakeFiles/idyntree-high-level-kdl.dir/cmake_clean_target.cmake
	cd /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/idyntree-high-level-kdl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/build: lib/libidyntree-high-level-kdl.a

.PHONY : orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/build

orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/clean:
	cd /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl && $(CMAKE_COMMAND) -P CMakeFiles/idyntree-high-level-kdl.dir/cmake_clean.cmake
.PHONY : orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/clean

orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/depend:
	cd /home/zheng/robot_ws_zheng/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zheng/robot_ws_zheng/src /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl /home/zheng/robot_ws_zheng/build /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl /home/zheng/robot_ws_zheng/build/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/CMakeFiles/idyntree-high-level-kdl.dir/depend


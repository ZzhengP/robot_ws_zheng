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
include orca/examples/basic/CMakeFiles/00-load_robot_model.dir/depend.make

# Include the progress variables for this target.
include orca/examples/basic/CMakeFiles/00-load_robot_model.dir/progress.make

# Include the compile flags for this target's objects.
include orca/examples/basic/CMakeFiles/00-load_robot_model.dir/flags.make

orca/examples/basic/CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.o: orca/examples/basic/CMakeFiles/00-load_robot_model.dir/flags.make
orca/examples/basic/CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.o: /home/zheng/robot_ws_zheng/src/orca/examples/basic/00-load_robot_model.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zheng/robot_ws_zheng/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object orca/examples/basic/CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.o"
	cd /home/zheng/robot_ws_zheng/build/orca/examples/basic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.o -c /home/zheng/robot_ws_zheng/src/orca/examples/basic/00-load_robot_model.cc

orca/examples/basic/CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.i"
	cd /home/zheng/robot_ws_zheng/build/orca/examples/basic && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zheng/robot_ws_zheng/src/orca/examples/basic/00-load_robot_model.cc > CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.i

orca/examples/basic/CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.s"
	cd /home/zheng/robot_ws_zheng/build/orca/examples/basic && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zheng/robot_ws_zheng/src/orca/examples/basic/00-load_robot_model.cc -o CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.s

# Object files for target 00-load_robot_model
00__load_robot_model_OBJECTS = \
"CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.o"

# External object files for target 00-load_robot_model
00__load_robot_model_EXTERNAL_OBJECTS =

/home/zheng/robot_ws_zheng/devel/lib/orca/00-load_robot_model: orca/examples/basic/CMakeFiles/00-load_robot_model.dir/00-load_robot_model.cc.o
/home/zheng/robot_ws_zheng/devel/lib/orca/00-load_robot_model: orca/examples/basic/CMakeFiles/00-load_robot_model.dir/build.make
/home/zheng/robot_ws_zheng/devel/lib/orca/00-load_robot_model: /home/zheng/robot_ws_zheng/devel/lib/liborca.so
/home/zheng/robot_ws_zheng/devel/lib/orca/00-load_robot_model: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.5.2
/home/zheng/robot_ws_zheng/devel/lib/orca/00-load_robot_model: orca/examples/basic/CMakeFiles/00-load_robot_model.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zheng/robot_ws_zheng/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zheng/robot_ws_zheng/devel/lib/orca/00-load_robot_model"
	cd /home/zheng/robot_ws_zheng/build/orca/examples/basic && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/00-load_robot_model.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
orca/examples/basic/CMakeFiles/00-load_robot_model.dir/build: /home/zheng/robot_ws_zheng/devel/lib/orca/00-load_robot_model

.PHONY : orca/examples/basic/CMakeFiles/00-load_robot_model.dir/build

orca/examples/basic/CMakeFiles/00-load_robot_model.dir/clean:
	cd /home/zheng/robot_ws_zheng/build/orca/examples/basic && $(CMAKE_COMMAND) -P CMakeFiles/00-load_robot_model.dir/cmake_clean.cmake
.PHONY : orca/examples/basic/CMakeFiles/00-load_robot_model.dir/clean

orca/examples/basic/CMakeFiles/00-load_robot_model.dir/depend:
	cd /home/zheng/robot_ws_zheng/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zheng/robot_ws_zheng/src /home/zheng/robot_ws_zheng/src/orca/examples/basic /home/zheng/robot_ws_zheng/build /home/zheng/robot_ws_zheng/build/orca/examples/basic /home/zheng/robot_ws_zheng/build/orca/examples/basic/CMakeFiles/00-load_robot_model.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : orca/examples/basic/CMakeFiles/00-load_robot_model.dir/depend

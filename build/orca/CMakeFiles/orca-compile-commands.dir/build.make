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

# Utility rule file for orca-compile-commands.

# Include the progress variables for this target.
include orca/CMakeFiles/orca-compile-commands.dir/progress.make

orca/CMakeFiles/orca-compile-commands: orca/compile_commands.json
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zheng/robot_ws_zheng/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Copying 'compile_commands.json' to '/home/zheng/robot_ws_zheng/src/orca'"
	cd /home/zheng/robot_ws_zheng/build/orca && /usr/local/bin/cmake -E copy /home/zheng/robot_ws_zheng/build/orca/compile_commands.json /home/zheng/robot_ws_zheng/src/orca

orca-compile-commands: orca/CMakeFiles/orca-compile-commands
orca-compile-commands: orca/CMakeFiles/orca-compile-commands.dir/build.make

.PHONY : orca-compile-commands

# Rule to build all files generated by this target.
orca/CMakeFiles/orca-compile-commands.dir/build: orca-compile-commands

.PHONY : orca/CMakeFiles/orca-compile-commands.dir/build

orca/CMakeFiles/orca-compile-commands.dir/clean:
	cd /home/zheng/robot_ws_zheng/build/orca && $(CMAKE_COMMAND) -P CMakeFiles/orca-compile-commands.dir/cmake_clean.cmake
.PHONY : orca/CMakeFiles/orca-compile-commands.dir/clean

orca/CMakeFiles/orca-compile-commands.dir/depend:
	cd /home/zheng/robot_ws_zheng/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zheng/robot_ws_zheng/src /home/zheng/robot_ws_zheng/src/orca /home/zheng/robot_ws_zheng/build /home/zheng/robot_ws_zheng/build/orca /home/zheng/robot_ws_zheng/build/orca/CMakeFiles/orca-compile-commands.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : orca/CMakeFiles/orca-compile-commands.dir/depend


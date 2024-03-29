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
CMAKE_SOURCE_DIR = /home/zheng/robot_ws_zheng/src/ros_wrap_safe_motion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion

# Include any dependencies generated for this target.
include examples/CMakeFiles/torch_test.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/torch_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/torch_test.dir/flags.make

examples/CMakeFiles/torch_test.dir/torch_test.cpp.o: examples/CMakeFiles/torch_test.dir/flags.make
examples/CMakeFiles/torch_test.dir/torch_test.cpp.o: /home/zheng/robot_ws_zheng/src/ros_wrap_safe_motion/examples/torch_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/torch_test.dir/torch_test.cpp.o"
	cd /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/torch_test.dir/torch_test.cpp.o -c /home/zheng/robot_ws_zheng/src/ros_wrap_safe_motion/examples/torch_test.cpp

examples/CMakeFiles/torch_test.dir/torch_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/torch_test.dir/torch_test.cpp.i"
	cd /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zheng/robot_ws_zheng/src/ros_wrap_safe_motion/examples/torch_test.cpp > CMakeFiles/torch_test.dir/torch_test.cpp.i

examples/CMakeFiles/torch_test.dir/torch_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/torch_test.dir/torch_test.cpp.s"
	cd /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zheng/robot_ws_zheng/src/ros_wrap_safe_motion/examples/torch_test.cpp -o CMakeFiles/torch_test.dir/torch_test.cpp.s

# Object files for target torch_test
torch_test_OBJECTS = \
"CMakeFiles/torch_test.dir/torch_test.cpp.o"

# External object files for target torch_test
torch_test_EXTERNAL_OBJECTS =

/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: examples/CMakeFiles/torch_test.dir/torch_test.cpp.o
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: examples/CMakeFiles/torch_test.dir/build.make
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /home/zheng/libtorch/lib/libtorch.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /home/zheng/libtorch/lib/libc10.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libcuda.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libnvrtc.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libnvToolsExt.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libcudart.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /home/zheng/libtorch/lib/libc10_cuda.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/lib/libprotobuf.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /home/zheng/libtorch/lib/libc10_cuda.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /home/zheng/libtorch/lib/libc10.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda/lib64/libnvToolsExt.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda/lib64/libcudart.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libcufft.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libcurand.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libcublas.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libcublas_device.a
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libcudnn.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/local/cuda-9.0/lib64/libcudart.so
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test: examples/CMakeFiles/torch_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test"
	cd /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/torch_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/torch_test.dir/build: /home/zheng/robot_ws_zheng/devel/.private/ros_wrap_safe_motion/lib/ros_wrap_safe_motion/torch_test

.PHONY : examples/CMakeFiles/torch_test.dir/build

examples/CMakeFiles/torch_test.dir/clean:
	cd /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/examples && $(CMAKE_COMMAND) -P CMakeFiles/torch_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/torch_test.dir/clean

examples/CMakeFiles/torch_test.dir/depend:
	cd /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zheng/robot_ws_zheng/src/ros_wrap_safe_motion /home/zheng/robot_ws_zheng/src/ros_wrap_safe_motion/examples /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/examples /home/zheng/robot_ws_zheng/build/ros_wrap_safe_motion/examples/CMakeFiles/torch_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/torch_test.dir/depend


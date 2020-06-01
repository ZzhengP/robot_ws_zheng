# CMake generated Testfile for 
# Source directory: /home/zheng/robot_ws_zheng/src/openni2_camera/openni2_launch
# Build directory: /home/zheng/robot_ws_zheng/build/openni2_launch
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_openni2_launch_roslaunch-check_launch "/home/zheng/robot_ws_zheng/build/openni2_launch/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/zheng/robot_ws_zheng/build/openni2_launch/test_results/openni2_launch/roslaunch-check_launch.xml" "--return-code" "/usr/local/bin/cmake -E make_directory /home/zheng/robot_ws_zheng/build/openni2_launch/test_results/openni2_launch" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/zheng/robot_ws_zheng/build/openni2_launch/test_results/openni2_launch/roslaunch-check_launch.xml' '/home/zheng/robot_ws_zheng/src/openni2_camera/openni2_launch/launch' ")
set_tests_properties(_ctest_openni2_launch_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;143;add_test;/opt/ros/melodic/share/roslaunch/cmake/roslaunch-extras.cmake;58;catkin_run_tests_target;/home/zheng/robot_ws_zheng/src/openni2_camera/openni2_launch/CMakeLists.txt;15;roslaunch_add_file_check;/home/zheng/robot_ws_zheng/src/openni2_camera/openni2_launch/CMakeLists.txt;0;")
subdirs("gtest")

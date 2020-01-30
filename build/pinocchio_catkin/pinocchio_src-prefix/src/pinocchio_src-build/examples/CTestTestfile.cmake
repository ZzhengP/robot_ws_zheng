# CMake generated Testfile for 
# Source directory: /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples
# Build directory: /home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/examples
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(example-overview-simple "/example-overview-simple")
set_tests_properties(example-overview-simple PROPERTIES  DEPENDS "ctest_build_tests" _BACKTRACE_TRIPLES "/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/cmake/test.cmake;57;ADD_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;20;ADD_UNIT_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;0;")
add_test(example-overview-lie "/example-overview-lie")
set_tests_properties(example-overview-lie PROPERTIES  DEPENDS "ctest_build_tests" _BACKTRACE_TRIPLES "/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/cmake/test.cmake;57;ADD_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;20;ADD_UNIT_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;0;")
add_test(example-i-inverse-kinematics "/example-i-inverse-kinematics")
set_tests_properties(example-i-inverse-kinematics PROPERTIES  DEPENDS "ctest_build_tests" _BACKTRACE_TRIPLES "/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/cmake/test.cmake;57;ADD_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;20;ADD_UNIT_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;0;")
add_test(py-example-i-inverse-kinematics "/usr/bin/python" "/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/i-inverse-kinematics.py")
set_tests_properties(py-example-i-inverse-kinematics PROPERTIES  ENVIRONMENT "PYTHONPATH=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/bindings/python;/opt/openrobots/lib/python2.7/site-packages:/home/zheng/robot_ws_zheng/devel/lib/python2.7/dist-packages:/home/zheng/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages" _BACKTRACE_TRIPLES "/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/cmake/test.cmake;79;ADD_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;38;ADD_PYTHON_UNIT_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;0;")
add_test(py-example-overview-simple "/usr/bin/python" "/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/overview-simple.py")
set_tests_properties(py-example-overview-simple PROPERTIES  ENVIRONMENT "PYTHONPATH=/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src-build/bindings/python;/opt/openrobots/lib/python2.7/site-packages:/home/zheng/robot_ws_zheng/devel/lib/python2.7/dist-packages:/home/zheng/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages" _BACKTRACE_TRIPLES "/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/cmake/test.cmake;79;ADD_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;38;ADD_PYTHON_UNIT_TEST;/home/zheng/robot_ws_zheng/build/pinocchio_catkin/pinocchio_src-prefix/src/pinocchio_src/examples/CMakeLists.txt;0;")

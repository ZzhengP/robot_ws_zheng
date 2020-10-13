execute_process(COMMAND "/home/zheng/robot_ws_zheng/build/openni2_launch/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zheng/robot_ws_zheng/build/openni2_launch/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

add_executable(load_robot_0 load_robot.cpp)
target_link_libraries(load_robot_0 PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} robot_controller_library)

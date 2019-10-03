add_executable(load_robot_0 load_robot.cpp)
target_link_libraries(load_robot_0 PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} robot_controller_library)


add_executable(gazebo_sim gazebo_sim.cpp)
target_link_libraries(gazebo_sim PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} robot_controller_library)

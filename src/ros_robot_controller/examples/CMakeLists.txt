
add_executable(ros_gazebo_sim ros_gazebo_sim.cpp)
target_link_libraries(ros_gazebo_sim PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ros_robot_controller_library)



add_executable(franka_example franka_example.cpp)
target_link_libraries(franka_example PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ${rviz_visual_tools_LIBRARIE} ros_robot_controller_library)


add_executable(markers markers.cpp)
target_link_libraries(markers PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ros_robot_controller_library)

add_executable(obsavoidancedemo obsavoidancedemo.cpp)
target_link_libraries(obsavoidancedemo PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} glpk ros_robot_controller_library)

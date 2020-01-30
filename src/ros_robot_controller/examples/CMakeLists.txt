
add_executable(ros_gazebo_sim ros_gazebo_sim.cpp)
target_link_libraries(ros_gazebo_sim PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ros_robot_controller_library)



add_executable(franka_example franka_example.cpp)
target_link_libraries(franka_example PRIVATE ${catkin_LIBRARIES} ${iDynTree_LIBRARIES}  ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ${rviz_visual_tools_LIBRARIE} glpk ros_robot_controller_library)


add_executable(markers markers.cpp)
target_link_libraries(markers PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ros_robot_controller_library)

add_executable(obsavoidancedemo obsavoidancedemo.cpp)
target_link_libraries(obsavoidancedemo PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_robot_controller_library)

add_executable(setinit setinit.cpp)
target_link_libraries(setinit PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} glpk ros_robot_controller_library)

add_executable(lpsolvetest lpsolve_test.cpp)
target_link_libraries(lpsolvetest PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} ${lpsolve_LIBRAIRIES} glpk ros_robot_controller_library)



add_executable(iDynTreeTest iDynTreeTest.cpp)
target_link_libraries(iDynTreeTest PRIVATE ${catkin_LIBRARIES} ${iDynTree_LIBRARIES}
                      ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ${rviz_visual_tools_LIBRARIE} glpk ros_robot_controller_library)

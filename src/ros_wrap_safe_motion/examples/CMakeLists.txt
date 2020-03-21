

add_executable(RosMovingobstacleavoidance movingobstacleavoidance.cpp)
target_link_libraries(RosMovingobstacleavoidance PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library)

add_executable(robotAndObsVerticesSubscriber robotAndObsVerticesSubscriber.cpp)
target_link_libraries(robotAndObsVerticesSubscriber PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library)


add_executable(obsVerticesPublisher obsVerticesPublisher.cpp)
target_link_libraries(obsVerticesPublisher PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library)

add_executable(simpletest simpletest.cpp)
target_link_libraries(simpletest PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )

add_executable(simpleMatrixReceive simpleMatrixReceive.cpp)
target_link_libraries(simpleMatrixReceive PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )

add_executable(constraintSolver constraintSolver.cpp)
target_link_libraries(constraintSolver PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )
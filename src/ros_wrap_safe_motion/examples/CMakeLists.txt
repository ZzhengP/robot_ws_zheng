

add_executable(RosMovingobstacleavoidance movingobstacleavoidance.cpp)
target_link_libraries(RosMovingobstacleavoidance PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library)

add_executable(separatingPlane separatingPlane.cpp)
target_link_libraries(separatingPlane PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library)


add_executable(obsVerticesPublisher obsVerticesPublisher.cpp)
target_link_libraries(obsVerticesPublisher PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library)

add_executable(simpletest simpletest.cpp)
target_link_libraries(simpletest PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ${PCL_LIBRARIES} glpk ros_wrap_safe_motion_library )

add_executable(simpleMatrixReceive simpleMatrixReceive.cpp)
target_link_libraries(simpleMatrixReceive PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )


add_executable(DynamicAndMPCSolver DynamicAndMPCSolver.cpp)
target_link_libraries(DynamicAndMPCSolver PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )


add_executable(InteractiveMarkers InteractiveMarkers.cpp)
target_link_libraries(InteractiveMarkers PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )


add_executable(obsInteractiveMarkersPublisher obsInteractiveMarkersPublisher.cpp)
target_link_libraries(obsInteractiveMarkersPublisher PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )

add_executable(humanperceptiontest humanperceptiontest.cpp)
target_link_libraries(humanperceptiontest PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES}  glpk ros_wrap_safe_motion_library )

add_executable(poseDepthTest poseDepthTest.cpp)
target_link_libraries(poseDepthTest PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ${OpenCV_LIBRARIES} glpk ros_wrap_safe_motion_library)

add_executable(MRtest MRtest.cpp)
target_link_libraries(MRtest PRIVATE ${catkin_LIBRARIES} ${qpOASES_LIBRARIES}   ros_wrap_safe_motion_library )

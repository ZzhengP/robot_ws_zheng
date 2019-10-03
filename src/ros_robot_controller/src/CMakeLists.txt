set(HEADER_LIST "${ros_robot_controller_SOURCE_DIR}/include/robotmodel/RosRobotModel.h"
    )

add_library(ros_robot_controller_library
            robotmodel/RosRobotModel.cpp)

target_include_directories(ros_robot_controller_library PUBLIC ../include)
target_link_libraries(ros_robot_controller_library PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} )



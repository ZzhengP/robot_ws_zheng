set(HEADER_LIST "${ros_robot_controller_SOURCE_DIR}/include/robotmodel/RosRobotModel.h"
                "${ros_robot_controller_SOURCE_DIR}/include/commun/color.h"
                "${ros_robot_controller_SOURCE_DIR}/include/commun/RosCommunDefinition.h"
                "${ros_robot_controller_SOURCE_DIR}/include/constraint/RosGenericCst.h"
                "${ros_robot_controller_SOURCE_DIR}/include/constraint/RosJointAccCst.h"
                "${ros_robot_controller_SOURCE_DIR}/include/constraint/RosJointPosCst.h"
                "${ros_robot_controller_SOURCE_DIR}/include/constraint/RosJointVelCst.h"
                "${ros_robot_controller_SOURCE_DIR}/include/solver/RosMpcSolver.h"
                "${ros_robot_controller_SOURCE_DIR}/include/task/RosMpcTask.h"
                "${ros_robot_controller_SOURCE_DIR}/include/visualization/RosMarkers.h")

add_library(ros_robot_controller_library
            robotmodel/RosRobotModel.cpp
            constraint/RosGenericCst.cpp
            constraint/RosJointPosCst.cpp
            constraint/RosJointVelCst.cpp
            solver/RosMpcSolver.cpp
            task/RosMpcTask.cpp
            visualization/RosMarkers.cpp)

target_include_directories(ros_robot_controller_library PUBLIC ../include)
target_link_libraries(ros_robot_controller_library PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} )



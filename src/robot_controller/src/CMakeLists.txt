set(HEADER_LIST "${robot_controller_SOURCE_DIR}/include/robotmodel/robotmodel.h"
                "${robot_controller_SOURCE_DIR}/include/constraint/joint_vel_cst.h"
                "${robot_controller_SOURCE_DIR}/include/solver/mpc_solver.h"
                "${robot_controller_SOURCE_DIR}/include/task/mpc_task.h")

add_library(robot_controller_library
    robotmodel/robotmodel.cpp
    constraint/joint_vel_cst.cpp
    solver/mpc_solver.cpp
    task/mpc_task.cpp
    ${HEADER_LIST})

target_include_directories(robot_controller_library PUBLIC ../include)

target_link_libraries(robot_controller_library PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} )

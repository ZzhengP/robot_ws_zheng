set(HEADER_LIST "${ros_wrap_safe_motion_SOURCE_DIR}/include/robotmodel/RosWrapRobotModel.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/robotmodel/RosWrapRobotDynTree.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/robotmodel/RosWrapchainjnttojacdotsolver.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/commun/color.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/commun/RosWrapCommunDefinition.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/constraint/RosWrapGenericCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/constraint/RosWrapJointAccCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/constraint/RosWrapJointPosCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/constraint/RosWrapJointVelCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/constraint/RosWrapCartVelCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/constraint/RosWrapAvoidanceCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/solver/RosWrapMpcSolver.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/solver/RosWrapPlaneSolver.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/solver/RosWraplpsolve.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/task/RosWrapMpcTask.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/visualization/RosWrapMarkers.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/sepPlane/RosWrapsepPlane.h"
                )

add_library(ros_wrap_safe_motion_library
            RosWraprobotmodel/RosWrapRobotModel.cpp
            RosWraprobotmodel/RosWrapRobotDynTree.cpp
            RosWraprobotmodel/RosWrapchainjnttojacdotsolver.cpp
            RosWrapconstraint/RosWrapGenericCst.cpp
            RosWrapconstraint/RosWrapJointPosCst.cpp
            RosWrapconstraint/RosWrapJointVelCst.cpp
            RosWrapconstraint/RosWrapAvoidanceCst.cpp
            RosWrapsolver/RosWrapMpcSolver.cpp
            RosWraptask/RosWrapMpcTask.cpp
            RosWrapvisualization/RosWrapMarkers.cpp
            RosWrapsepPlane/RosWrapsepPlane.cpp
            RosWrapsolver/RosWrapPlaneSolver.cpp
            RosWrapsolver/RosWraplpsolve.cpp)

target_include_directories(ros_wrap_safe_motion_library PUBLIC ../include)
target_link_libraries(ros_wrap_safe_motion_library PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${qpOASES_LIBRARIES} ${iDynTree_LIBRARIES} glpk caffe ${OpenPose_LIBS} boost_system glog jsoncpp)



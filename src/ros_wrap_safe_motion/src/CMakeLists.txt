set(HEADER_LIST "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWraprobotmodel/RosWrapRobotModel.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWraprobotmodel/RosWrapRobotDynTree.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWraprobotmodel/RosWrapchainjnttojacdotsolver.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapcommun/color.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapcommun/RosWrapCommunDefinition.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapconstraint/RosWrapGenericCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapconstraint/RosWrapJointAccCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapconstraint/RosWrapJointPosCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapconstraint/RosWrapJointVelCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapconstraint/RosWrapCartVelCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapconstraint/RosWrapAvoidanceCst.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapsolver/RosWrapMpcSolver.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapsolver/RosWrapPlaneSolver.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapsolver/RosWraplpsolve.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWraptask/RosWrapMpcTask.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapvisualization/RosWrapMarkers.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapsepPlane/RosWrapsepPlane.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapperception/RosWrapHumanMotion.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapperception/RosWrapHumanOccupancy.h"
                "${ros_wrap_safe_motion_SOURCE_DIR}/include/RosWrapperception/RosWrapHumanOptimalTorque.h"
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



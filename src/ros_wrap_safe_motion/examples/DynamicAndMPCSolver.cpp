#include "RosWrapcommun/RosWrapCommunDefinition.h"
#include "RosWraprobotmodel/RosWrapRobotModel.h"
#include "RosWrapsolver/RosWrapMpcSolver.h"
#include "RosWrapsolver/RosWrapPlaneSolver.h"
#include "RosWraptask/RosWrapMpcTask.h"
#include "RosWrapconstraint/RosWrapGenericCst.h"
#include "RosWrapconstraint/RosWrapJointAccCst.h"
#include "RosWrapconstraint/RosWrapJointVelCst.h"
#include "RosWrapconstraint/RosWrapJointPosCst.h"
#include "RosWrapconstraint/RosWrapCartVelCst.h"
#include "RosWrapconstraint/RosWrapAvoidanceCst.h"
#include "RosWrapvisualization/RosWrapMarkers.h"
#include "RosWrapsepPlane/RosWrapsepPlane.h"
#include "RosWrapsolver/RosWraplpsolve.h"

#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include <boost/timer.hpp>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"

#include <sstream>
#include "cmath"
#include "qpOASES.hpp"
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"
#include "memory.h"
#include <stdio.h>
#include <stdlib.h>

#include "iostream"
// ros
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <sdf/parser_urdf.hh>
#include <fstream>
#include <thread>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"

#include <boost/shared_ptr.hpp>
#include "glpk.h"
#include <iostream>
#include <math.h>
#include "nav_msgs/Path.h"
#include <chrono>
#include "eigen_conversions/eigen_msg.h"
#include "eigen_conversions/eigen_kdl.h"



int main(int argc, char **argv)
{


    ros::init(argc,argv,"MPController");
    ros::NodeHandle n;
    int N, ndof ;  
    n.getParam("/N",N);
    ndof = 7;

    double dt;
    n.getParam("/dt",dt);
    int constraintSize = 2*ndof*N + 8;
   
    std::shared_ptr<mpc_solve> MPController = std::make_shared<mpc_solve>(&n,N,ndof,constraintSize,dt );

    std::string panda_urdf = "/home/zheng/robot_ws_zheng/src/franka_description/robots/panda_arm.urdf";

    KDL::JntArray panda_q_init(ndof), panda_dotq_init(ndof),panda_q_des(ndof), panda_q_des_back(ndof);

    // initialize robot's joint position and velocity according to roslaunch initialization
    panda_q_init.data << 1.11222,
                         0.80472,
                         -0.535436,
                         -1.98801,
                         0.764916,
                         2.5814,
                         0.0391129;
    //    panda_q_init.data<<0.5, 0., 0., -1.8,0.,1.57 , 0. ;

    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd lb, ub;
    lb.resize(N*ndof);
    ub.resize(N*ndof);

    lb.setConstant(-8);
    ub.setConstant(8);


    Eigen::VectorXd ddqMin, ddqMax, dqMin, dqMax, qMin, qMax, vMax, vMin;


    ddqMin.resize(N*ndof), ddqMax.resize(N*ndof);
    ddqMax = ub, ddqMin = lb;
    qMin.resize(N*ndof), qMax.resize(N*ndof);
    qMin.setConstant(-3.14), qMax.setConstant(3.14);

    dqMin.resize(N*ndof), dqMax.resize(N*ndof);
    dqMin.setConstant(-3.10/2), dqMax.setConstant(3.14/2);
     dqMin.tail(ndof).setZero(), dqMax.tail(ndof).setZero();


    Eigen::VectorXd cartVelMin, cartVelMax ;
    cartVelMin.resize(3*N);
    cartVelMax.resize(3*N);

    for (int k(0); k <N; k ++){
        cartVelMin.segment(3*k,3) << -0.6, -0.6, -0.6;
        cartVelMax.segment(3*k,3) << 0.6, 0.6, 0.6 ;
    }

    MPController -> initTaskData();
    MPController -> initArmData(panda_urdf,panda_q_init,panda_dotq_init);
    MPController -> initMPCData(lb,ub);
    MPController -> initRobotConstraintData(ddqMin,ddqMax,dqMin,dqMax,qMin,qMax,cartVelMin,cartVelMax);

    MPController ->run();    

    return 0;
}
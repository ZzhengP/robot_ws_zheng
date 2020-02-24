#include "robotmodel/RosRobotModel.h"
#include "solver/RosMpcSolver.h"
#include "task/RosMpcTask.h"
#include "constraint/RosGenericCst.h"
#include "constraint/RosJointAccCst.h"
#include "constraint/RosJointVelCst.h"
#include "constraint/RosJointPosCst.h"
#include "visualization/RosMarkers.h"
#include "sepPlane/sepPlane.h"
#include "solver/lpsolve.h"
#include "iostream"
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include <boost/timer.hpp>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"
// ros
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <sdf/parser_urdf.hh>
#include <fstream>
#include <thread>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include "cmath"
#include "qpOASES.hpp"
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"
#include "memory.h"
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include "glpk.h"
#include <iostream>
#include <fstream>

#include "robotmodel/RobotDynTree.h"

int main(int argc, char *argv[])
{
    const std::string& panda_urdf = "/home/zheng/robot_ws_zheng/src/franka_description/robots/panda_arm.urdf";


    auto robot = std::make_shared<robot::RobotDynClass>();
        if(!robot->loadModelFromFile(panda_urdf))
        {
            return -1;
        }
        robot->setBaseFrame("panda_link0");
        robot->setGravity(Eigen::Vector3d(0,0,-9.81));

        Eigen::VectorXd jointPos ;
        jointPos.resize(7);
        jointPos <<  0.986518, -0.0831163,  -0.586771,   -1.97076, -0.0485966,    1.90111,   0.417105;

        robot::EigenRobotState state;
        state.setFixedBase(); // sets world to base to identity and base velocity to zero
        state.resize(robot->getNrOfDegreesOfFreedom());

        state.jointPos= jointPos;
        state.jointVel.setZero();
        robot->setRobotState(state.jointPos,state.jointVel);

        robot->print();


        std::cout <<" Jacobian \n" << robot->getJacobian("0","8") <<'\n';
    return 0;
}

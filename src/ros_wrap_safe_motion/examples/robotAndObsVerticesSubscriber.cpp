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
#include <math.h>
#include "nav_msgs/Path.h"
#include <chrono>

using namespace  std;
using namespace qpOASES;
using namespace  plane;
const double pi = 3.1415927;


Eigen::MatrixXd ComputeObstacleVertices(Eigen::Vector3d centre, Eigen::Vector3d size){

    Eigen::MatrixXd Vertices;
    Vertices.resize(3,8);

    Vertices.block(0,0,1,8) << centre(0) - size(0), centre(0) - size(0), centre(0) - size(0), centre(0) - size(0),
                               centre(0) + size(0), centre(0) + size(0), centre(0) + size(0), centre(0) + size(0);

    Vertices.block(1,0,1,8) << centre(1) + size(1), centre(1) + size(1), centre(1) - size(1), centre(1) - size(1),
                               centre(1) + size(1), centre(1) + size(1), centre(1) - size(1), centre(1) - size(1);

    Vertices.block(2,0,1,8) << centre(2) - size(2), centre(2) + size(2), centre(2) - size(2), centre(2) + size(2),
                               centre(2) - size(2), centre(2) + size(2), centre(2) - size(2), centre(2) + size(2);

    return  Vertices;
}


int main(int argc, char **argv)
{


    // ------------------------------------   Initialize ------------------------------------------
    ros::init(argc,argv,"robot_vertices_subsriber");
    ros::NodeHandle n;
    int N ;
    n.getParam("/N",N);

    Eigen::Vector3d obsSize, obsCentre;

    int numberOfObstacle;
    n.getParam("/numberOfObstacle", numberOfObstacle);

    for (size_t i(0); i < 3; i++ ){
        std::string s = std::to_string(i); ;
        n.getParam("/obsSize/s"+s , obsSize[i]);
        n.getParam("/obsCentre/s"+s ,obsCentre[i]);


    }

    
    Eigen::MatrixXd obsVertices;
    std::vector<Eigen::MatrixXd> obsVerticesAugmented;
    int obsNbrVertices = 8;
    obsVertices.resize(3*numberOfObstacle,obsNbrVertices);

    obsVertices = ComputeObstacleVertices(obsCentre,obsSize);

    obsVerticesAugmented.resize(numberOfObstacle);
    for (int i(0); i < numberOfObstacle; i++){
         obsVerticesAugmented[i].resize(3,obsNbrVertices*N);
    }

    for (int j(0); j < numberOfObstacle; j++){
        for (int i(0) ; i < N ; i++) {
            obsVerticesAugmented[j].block(0,i*obsNbrVertices,3,obsNbrVertices) = obsVertices;
        }
    }
    int nbrObs = 1;
    double dsafe = 0.1;
    std::shared_ptr<Planes> plane = std::make_shared<Planes>(&n,N,obsVerticesAugmented,dsafe);

    plane -> run();
    // ros::spin();

    return 0;
}
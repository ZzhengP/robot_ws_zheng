#include "robotmodel/RosRobotModel.h"
#include "solver/RosMpcSolver.h"
#include "task/RosMpcTask.h"
#include "constraint/RosGenericCst.h"
#include "constraint/RosJointAccCst.h"
#include "constraint/RosJointVelCst.h"
#include "constraint/RosJointPosCst.h"
#include "visualization/RosMarkers.h"
#include "sepPlane/sepPlane.h"

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
using namespace  std;
using namespace qpOASES;
const double pi = 3.1415927;


int main(int argc, char **argv)
{

    // ------------------------------------   Initialize ------------------------------------------
    // Initialize ros node for collision avoidance controller
    ros::init(argc,argv,"panda_safety_controller");
    ros::NodeHandle n;

    double dt(0.05);
    // load robot's kinematic dynamic description
    const std::string& panda_urdf = "/home/zheng/robot_ws_zheng/src/franka_description/robots/pandaArm.urdf";

    // set robot's dof and predictive step
    const int N=6, ndof = 7;
    KDL::JntArray panda_q_init(ndof), panda_dotq_init(ndof),panda_q_des(ndof), panda_q_des_back(ndof);

    // initialize robot's joint position and velocity according to roslaunch initialization
    panda_q_init.data << 1.57, 0, 0, -1.8, 0, 1.57, 0;
    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;

    // initialize robot's kinematic class
    arm_kinematic pandaArm(panda_urdf, ndof,"panda_link0", "panda_link7");
    pandaArm.init(panda_q_init.data, panda_dotq_init.data, N);

    // panda's end-effector location and set up desired frame
    KDL::Frame des_frame, back_des_frame, panda_ee_frame, panda_des_frame;
    KDL::Frame forearm, elbow;
    panda_ee_frame = pandaArm.getSegmentPosition("panda_link7");
    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    forearm = pandaArm.getSegmentPosition(3);
    panda_des_frame.p[0] = 0.5 ;
    panda_des_frame.p[1] = -0.3 ;
    panda_des_frame.p[2] = 0.3 ;
    panda_des_frame.M = panda_des_frame.M;
    panda_des_frame.M.DoRotX(- pi);
    pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);

    // define an cube shape as an obstacle
    rviz_visual_tools::RvizVisualToolsPtr cubeObstacleMarkers;
    rviz_visual_tools::colors color = rviz_visual_tools::RED;

    cubeObstacleMarkers.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/simpleCube"));

    // define cube location (we suppose this is only a point)
    geometry_msgs::Pose cubeLocation;
    cubeLocation.position.x = 0.5;
    cubeLocation.position.y = 0.;
    cubeLocation.position.z = 0.7;
    // define cube's size (x,y,z)
    Eigen::Vector3d obsSize;
    obsSize << 0.1,0.1,0.1;
    Eigen::MatrixXd obsVertices;
    obsVertices.resize(3,N);
    for (int i(0); i < N; i ++ ){
        obsVertices.block(0,i,3,1) << cubeLocation.position.x,
                                      cubeLocation.position.y,
                                      cubeLocation.position.z;
    }
    // define robot's vertices to be controlled (only two points)
    // We choose the base points, forearm and end-effcotor, initialized these
    std::vector<Eigen::MatrixXd> robotVertices;
    robotVertices.resize(3);
    for (int i(0); i<3;i++){
        robotVertices[i].resize(3,N);
    }

    for (int i(0); i < N; i ++){
        robotVertices[0].block(0,i,3,1) << 0,0,0;
        robotVertices[1].block(0,i,3,1) << panda_ee_frame.p.x(),panda_ee_frame.p.y(),panda_ee_frame.p.z();
        robotVertices[2].block(0,i,3,1) << forearm.p.x(),forearm.p.y(),forearm.p.z();

    }

    // put vertices into ros msg to visualize them in rviz
    geometry_msgs::PoseArray robotVerticesPose;
    robotVerticesPose.poses.resize(3);
    robotVerticesPose.poses[0].position.x = 0;
    robotVerticesPose.poses[0].position.y = 0;
    robotVerticesPose.poses[0].position.z = 0;
    robotVerticesPose.poses[1].position.x = panda_ee_frame.p.x();
    robotVerticesPose.poses[1].position.y = panda_ee_frame.p.y();
    robotVerticesPose.poses[1].position.z = panda_ee_frame.p.z();
    robotVerticesPose.poses[2].position.x = forearm.p.x();
    robotVerticesPose.poses[2].position.y = forearm.p.y();
    robotVerticesPose.poses[2].position.z = forearm.p.z();

    // ------------------------------------   Initialize MPC  ------------------------------------------
    Eigen::VectorXd optimalSolution;

    // Define MPC task
    Eigen::VectorXd pandaState, qHorizon;
    bool panda_ok;
    MPC_Task pandaTask(N,1,0.001,ndof,dt, "panda");
    panda_ok = pandaTask.init();

    // Augmented state
    Eigen::MatrixXd pandaPx, pandaPu, pandaPxdq, pandaPudq;
    pandaPx.resize(N*ndof,2*ndof);
    pandaPu.resize(N*ndof,N*ndof);
    pandaPx = pandaTask.getMatrixPx();
    pandaPu = pandaTask.getMatrixPu();

    pandaPxdq.resize(N*ndof,2*ndof);
    pandaPudq.resize(N*ndof,N*ndof);
    pandaPxdq = pandaTask.getMatrixPxdq();
    pandaPudq = pandaTask.getMatrixPudq();

    pandaState.resize(2*ndof);
    pandaState = pandaArm.getRobotState();

    optimalSolution.resize(ndof*N);
    optimalSolution.setZero();
    qHorizon.resize(ndof*N);

    Eigen::MatrixXd jacobianHorizon;
    jacobianHorizon.resize(N*6, N*ndof);

    Eigen::VectorXd qHorizonDes;
    qHorizonDes.resize(ndof*N);
    for (size_t i=0; i<N; i++)
    {
         qHorizonDes.segment(ndof*i,ndof) = panda_q_des.data;
    }

    Eigen::MatrixXd  stateA, stateB;
    stateA = pandaTask.getStateA();
    stateB = pandaTask.getStateB();
    pandaArm.computeqEnlarged(optimalSolution,pandaPx,pandaPu);
    qHorizon = pandaPx*pandaState + pandaPu*optimalSolution;

    // define mpc constraints
    int Cst = 0;
    Eigen::VectorXd ddqMin, ddqMax, ddqLb, ddqUb, dqMin, dqMax, qMin, qMax;
    Eigen::MatrixXd ddqC;
    ddqMin.resize(N*ndof), ddqMax.resize(N*ndof);
    ddqLb.resize(N*ndof), ddqUb.resize(N*ndof);
    ddqC.resize(N*ndof, N*ndof), ddqC.setIdentity();
    dqMin.resize(N*ndof), dqMax.resize(N*ndof);
    qMin.resize(N*ndof), qMax.resize(N*ndof);
    ddqMin.setConstant(-10);
    ddqMax.setConstant(10);
    double jntVelLim = 1;
    dqMin.setConstant(-jntVelLim);
    dqMax.setConstant(jntVelLim);
    qMin.setConstant(-pi);
    qMax.setConstant(pi);

    // Define joint acceleration limit
    jntAccCst jnt_acc_cst(ndof, N);
    jnt_acc_cst.setLimit(ddqMin,ddqMax);
    jnt_acc_cst.setLowerBound(pandaState, pandaPx);
    jnt_acc_cst.setUpperBound(pandaState, pandaPx);
    ddqLb = jnt_acc_cst.getLowerBound();
    ddqUb = jnt_acc_cst.getUpperBound();

    // Define joint velocity limit
    jntVelCst jnt_vel_cst(ndof,N);
    jnt_vel_cst.setLimit(dqMin, dqMax);
    jnt_vel_cst.setLowerBound(pandaState,pandaPxdq);
    jnt_vel_cst.setUpperBound(pandaState,pandaPxdq);
    jnt_vel_cst.setConstraintMatrix(pandaPudq);

    Cst += 1 ;
    // Define joint position limit
    jntPosCst jnt_pos_cst(ndof,N);
    jnt_pos_cst.setLimit(qMin,qMax);
    jnt_pos_cst.setLowerBound(pandaState,pandaPx);
    jnt_pos_cst.setUpperBound(pandaState,pandaPx);
    jnt_pos_cst.setConstraintMatrix(pandaPu);

    ros::Rate loop_rate(100);

    // create ros topic for sending msg
    ros::Publisher panda_joint_state_1_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint1_position_controller/command", 1000);
    ros::Publisher panda_joint_state_2_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint2_position_controller/command", 1000);
    ros::Publisher panda_joint_state_3_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint3_position_controller/command", 1000);
    ros::Publisher panda_joint_state_4_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint4_position_controller/command", 1000);
    ros::Publisher panda_joint_state_5_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint5_position_controller/command", 1000);
    ros::Publisher panda_joint_state_6_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint6_position_controller/command", 1000);
    ros::Publisher panda_joint_state_7_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint7_position_controller/command", 1000);
    while(ros::ok())
    {











    cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[0],obsSize[0],obsSize[1],obsSize[2]);
    cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[1],obsSize[0],obsSize[1],obsSize[2]);
    cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[2],obsSize[0],obsSize[1],obsSize[2]);
    cubeObstacleMarkers->publishCuboid(cubeLocation,obsSize[0],obsSize[1],obsSize[2],color);

    cubeObstacleMarkers->trigger();

    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}

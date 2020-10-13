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

using namespace  std;
using namespace qpOASES;
const double pi = 3.1415927;


int main(int argc, char **argv)
{


    // ------------------------------------   Initialize ------------------------------------------
    // Initialize ros node for collision avoidance controller
    ros::init(argc,argv,"panda_safety_controller");
    ros::NodeHandle n;

    double dt(0.1);
    // load robot's kinematic dynamic description
    const std::string& panda_urdf = "/home/zheng/robot_ws_zheng/src/franka_description/robots/panda_arm.urdf";

    // set robot's dof and predictive step
    const int N=5, ndof = 7;
    KDL::JntArray panda_q_init(ndof), panda_dotq_init(ndof),panda_q_des(ndof), panda_q_des_back(ndof);

    // initialize robot's joint position and velocity according to roslaunch initialization
//    panda_q_init.data << 1.57, 0, 0, -1.8, 0, 1.57, 0;
//    panda_q_init.data<<0.5, 0., 0., -1.8,0.,1.57 , 0. ;
    panda_q_init.data << 1.11222,
                         0.80472,
                         -0.535436,
                         -1.98801,
                         0.764916,
                         2.5814,
                         0.0391129;
    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;

    // initialize robot's kinematic class
    arm_kinematic pandaArm(panda_urdf, ndof,"panda_link0", "panda_link8");

//    auto robot = std::make_shared<arm_kinematic>(panda_urdf, ndof,"panda_link0", "panda_link7");
    pandaArm.init(panda_q_init.data, panda_dotq_init.data, N);
//    robot->init(panda_q_init.data,panda_dotq_init.data,N)
//    return 0;
    // panda's end-effector location and set up desired frame
    KDL::Frame des_frame, back_des_frame, panda_ee_frame, panda_des_frame, base_frame;
    KDL::Frame forearm, elbow;
    panda_ee_frame = pandaArm.getSegmentPosition(8);
    base_frame = pandaArm.getSegmentPosition(0);
    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    forearm = pandaArm.getSegmentPosition(3);
    panda_des_frame.p[0] = panda_ee_frame.p[0] ;
    panda_des_frame.p[1] = panda_ee_frame.p[1] ;
    panda_des_frame.p[2] = panda_ee_frame.p[2] ;
    panda_des_frame.M = base_frame.M;
    panda_des_frame.M.DoRotX(- pi);
    pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);


    // ------------------------------------   Initialize MPC  ------------------------------------------
    Eigen::VectorXd optimalSolution;

    // Define MPC task
    Eigen::VectorXd pandaState, qHorizon;
    bool panda_ok;
    MPC_Task pandaTask(N,1,0.0001,ndof,dt, "panda");
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
    qHorizon = pandaPx*pandaState + pandaPu*optimalSolution;
    pandaArm.setJointHorizon(qHorizon);
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
    dqMin.setConstant(-1);
    dqMax.setConstant(1);
    qMin.setConstant(-pi);
    qMax.setConstant(pi);

    // Define joint acceleration limit
    jntAccCst jnt_acc_cst(ndof, N,dt, "jointAccCst",pandaPx,pandaPu);
    jnt_acc_cst.setLimit(ddqMin,ddqMax);
    jnt_acc_cst.setLowerBound(pandaState);
    jnt_acc_cst.setUpperBound(pandaState);
    ddqLb = jnt_acc_cst.getLowerBound();
    ddqUb = jnt_acc_cst.getUpperBound();


     // ------------------------------------   Initialize qpOASES solver  ------------------------------------------


    // Define QP solver
    // compute the total number of constraint

    Eigen::MatrixXd H, cstA;
    H.resize(N*ndof,N*ndof);
    Eigen::VectorXd g, lb, ub, lbA, ubA;
    g.resize(N*ndof), lb.resize(N*ndof), ub.resize(N*ndof);


    mpc_solve qpSolver(N, ndof,5);

    qpSolver.setDefaultOptions();
    qpSolver.initData(H,g,lb,ub);
//    qpSolver.initData(H,g,cstA,lb,ub,lbA,ubA);
    // ------------------------------------   Initialize Ros connextion  ------------------------------------------

    ros::Rate loop_rate(10);

    // create ros topic for sending msg
    ros::Publisher panda_joint_state_1_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint1_position_controller/command", 100);
    ros::Publisher panda_joint_state_2_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint2_position_controller/command", 100);
    ros::Publisher panda_joint_state_3_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint3_position_controller/command", 100);
    ros::Publisher panda_joint_state_4_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint4_position_controller/command", 100);
    ros::Publisher panda_joint_state_5_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint5_position_controller/command", 100);
    ros::Publisher panda_joint_state_6_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint6_position_controller/command", 100);
    ros::Publisher panda_joint_state_7_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint7_position_controller/command", 100);
    while(ros::ok())
    {

        pandaState = pandaArm.getRobotState();
        qHorizon = pandaPx*pandaState + pandaPu*optimalSolution;
        pandaArm.setJointHorizon(qHorizon);
        qHorizon = pandaArm.getqEnlarged();

        pandaArm.computeJacobianHorz(qHorizon);

        jacobianHorizon = pandaArm.getJacobianHorz();


        pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
        H = pandaTask.getMatrixH();

        g = pandaTask.getVectorg();

        lb.setConstant(-10);
        ub.setConstant(10);


        qpSolver.solve(H,g,lb,ub);

        optimalSolution = qpSolver.getSolution();

        pandaState = stateA*pandaState + stateB*optimalSolution.segment(0,ndof);

         std_msgs::Float64 v1,v2,v3,v4,v5,v6 ; // robot joint's velocity
        std_msgs::Float64 panda_t1,panda_t2,panda_t3,panda_t4,panda_t5,panda_t6, panda_t7 ; // robot joint's positions

        panda_t1.data = pandaState[0];
        panda_t2.data = pandaState[1];
        panda_t3.data = pandaState[2];
        panda_t4.data = pandaState[3];
        panda_t5.data = pandaState[4];
        panda_t6.data = pandaState[5];
        panda_t7.data = pandaState[6];

        panda_joint_state_1_pub.publish(panda_t1);
        panda_joint_state_2_pub.publish(panda_t2);
        panda_joint_state_3_pub.publish(panda_t3);
        panda_joint_state_4_pub.publish(panda_t4);
        panda_joint_state_5_pub.publish(panda_t5);
        panda_joint_state_6_pub.publish(panda_t6);
        panda_joint_state_7_pub.publish(panda_t7);

        panda_ee_frame = pandaArm.getSegmentPosition(8);
        forearm = pandaArm.getSegmentPosition(3);

        pandaArm.setState(pandaState.head(ndof),pandaState.tail(ndof));

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout<<"panda joint final is : \n" << pandaArm.getRobotState().segment(0,ndof) << std::endl;
    std::cout<<"panda ee position is :\n " << pandaArm.getSegmentPosition(8).p << std::endl;
    std::cout<<"Calcul donné par KDL est :\n " << panda_q_des.data << std::endl;
    return 0;
}

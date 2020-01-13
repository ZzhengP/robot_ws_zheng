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
    const int N=2, ndof = 7;
    KDL::JntArray panda_q_init(ndof), panda_dotq_init(ndof),panda_q_des(ndof), panda_q_des_back(ndof);

    // initialize robot's joint position and velocity according to roslaunch initialization
    panda_q_init.data << 1.57, 0, 0, -1.8, 0, 1.57, 0;
    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;

    // initialize robot's kinematic class
    arm_kinematic pandaArm(panda_urdf, ndof,"panda_link0", "panda_link7");
//    auto robot = std::make_shared<arm_kinematic>(panda_urdf, ndof,"panda_link0", "panda_link7");
    pandaArm.init(panda_q_init.data, panda_dotq_init.data, N);
//    robot->init(panda_q_init.data,panda_dotq_init.data,N)
//    return 0;
    // panda's end-effector location and set up desired frame
    KDL::Frame des_frame, back_des_frame, panda_ee_frame, panda_des_frame;
    KDL::Frame forearm, elbow;
    panda_ee_frame = pandaArm.getSegmentPosition("panda_link7");
    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    forearm = pandaArm.getSegmentPosition(3);
    panda_des_frame.p[0] = 0.5 ;
    panda_des_frame.p[1] = 0.2 ;
    panda_des_frame.p[2] = panda_ee_frame.p.z();
//    panda_des_frame = panda_ee_frame;
    panda_des_frame.M = panda_des_frame.M;
    panda_des_frame.M.DoRotX(- pi);
    pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);

    // define an cube shape as an obstacle
    rviz_visual_tools::RvizVisualToolsPtr cubeObstacleMarkers;
    rviz_visual_tools::colors color = rviz_visual_tools::RED;
    rviz_visual_tools::colors color2 = rviz_visual_tools::CYAN;

    cubeObstacleMarkers.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/simpleCube"));

    // define cube location (we suppose this is only a point)
    geometry_msgs::Pose cubeLocation;
    cubeLocation.position.x = 0.5;
    cubeLocation.position.y = forearm.p.y();
    cubeLocation.position.z = forearm.p.z();
    // define cube's size (x,y,z)
    Eigen::Vector3d obsSize;
    obsSize << 0.1,0.1,0.1;
    Eigen::MatrixXd obsVertices, obsVerticesAugmented;
    obsVertices.resize(3,1);
    obsVerticesAugmented.resize(3,N);
    for (int i(0); i < 1; i ++ ){
        obsVertices.block(0,i,3,1) << cubeLocation.position.x,
                                      cubeLocation.position.y,
                                      cubeLocation.position.z;
    }

    for (int i(0) ; i < N ; i++) {
        obsVerticesAugmented.block(0,i,3,1) = obsVertices;
    }
    // define robot's vertices to be controlled (only two points)
    // We choose the base points, forearm and end-effcotor, initialized these
    std::vector<Eigen::MatrixXd> robotVertices, robotVerticesAugmented  ;
    robotVertices.resize(1);
    for (int i(0); i<robotVertices.size();i++){
        // 2 means there is two vertices to determine robot's link
        robotVertices[i].resize(3,2);
    }

    for (int i(0); i < 1; i ++){
//        robotVertices[0].block(0,2*i,3,1) << 0,0,0;
//        robotVertices[0].block(0,2*i+1,3,1) << forearm.p.x(),forearm.p.y(),forearm.p.z();
//        robotVertices[0].block(0,2*i,3,1) << forearm.p.x(),forearm.p.y(),forearm.p.z();
        robotVertices[0].block(0,2*i,3,1) <<  panda_ee_frame.p.x(),panda_ee_frame.p.y(),panda_ee_frame.p.z();
        robotVertices[0].block(0,2*i+1,3,1) <<  forearm.p.x(),forearm.p.y(),forearm.p.z();
    }

    // Define augmented data for mpc
    robotVerticesAugmented.resize(1);
    robotVerticesAugmented[0].resize(3,2*N);
//    robotVerticesAugmented[1].resize(3,2*N);
    for (int i(0);i<N;i++){
        robotVerticesAugmented[0].block(0,2*i,3,2) = robotVertices[0];
//        robotVerticesAugmented[1].block(0,2*i,3,2) = robotVertices[1];

    }
    std::cout <<"  robotVerticesAugmented :\n " << robotVerticesAugmented[0] << std::endl;
    // put vertices into ros msg to visualize them in rviz
    geometry_msgs::PoseArray robotVerticesPose;
    robotVerticesPose.poses.resize(1);
//    robotVerticesPose.poses[0].position.x = 0;
//    robotVerticesPose.poses[0].position.y = 0;
//    robotVerticesPose.poses[0].position.z = 0;
    robotVerticesPose.poses[0].position.x = panda_ee_frame.p.x();
    robotVerticesPose.poses[0].position.y = panda_ee_frame.p.y();
    robotVerticesPose.poses[0].position.z = panda_ee_frame.p.z();
//    robotVerticesPose.poses[1].position.x = forearm.p.x();
//    robotVerticesPose.poses[1].position.y = forearm.p.y();
//    robotVerticesPose.poses[1].position.z = forearm.p.z();

    // ------------------------------------   Initialize plane  ------------------------------------------
    int nbrRobotPart = 1, nbrObsPart=1;

    Planes plane(N,nbrRobotPart,nbrObsPart,robotVertices,obsVertices);
    Eigen::VectorXd singlePlane, singlePlane2;
    singlePlane.resize(5);
    singlePlane2.resize(5);
    double x_width = 2, y_width = 2;

    Planes::PlaneData planedata;
    planedata = plane.getPlanes();
    singlePlane << planedata.planeLocation[0].block(0,0,5,1);
//    singlePlane2 << planedata.planeLocation[1].block(0,0,5,1);
    // ------------------------------------   Initialize Lps solvers  ------------------------------------------

    int Nv = 5, Nc = 6;
    Eigen::VectorXd lpSolution;
    lpSolution.resize(5);
    std::shared_ptr<lpSolver> lp = std::make_shared<lpSolver>(Nv,Nc);
    lp->setCost();
    lp->setRowConstraint();

//    for (int i(0);i<nbrRobotPart; i++){
//        for (int j(0);j<nbrObsPart;j++){
//            for (int k(0); k < N-1; k++){
////                lpSolver.computeNormCst(robotVerticesAugmented[i].block(0,k,3,2),obsVerticesAugmented.block(0,0,3,2), planedata.planeLocation[i].block(0,k,3,1), epsilon);
////                lpSolver.solve();
////                planedata.planeLocation[i].block(0,k,5,1) = lpSolver.getSolution();
////                planedata.planeLocation[i].block(0,k,3,1) = planedata.planeLocation[i].block(0,k,3,1)/planedata.planeLocation[i].block(0,k,3,1).norm();
////                planedata.planeLocation[i](3,k) = planedata.planeLocation[i](3,k)*planedata.planeLocation[i].block(0,k,3,1).norm();
////                planedata.planeLocation[i](4,k) = planedata.planeLocation[i](4,k);

//            lp->setConstraintMatrix(robotVerticesAugmented[i].block(0,2*k,3,4),obsVerticesAugmented.block(0,0,3,2), planedata.planeLocation[i].block(0,k,3,1));
//            lp->setEqualityConstraintMatrix();
//            lp->loadEqualityMatrix();

////            lp->print();
//            lp->solve();
//            lpSolution = lp->getSolution();
//            std::cout << BOLD(FRED("lps solver solution is: "))  <<lpSolution.transpose()<< '\n';
//            planedata.planeLocation[i].block(0,k,5,1) = lp->getSolution();
//            planedata.planeLocation[i].block(0,k,3,1) = planedata.planeLocation[i].block(0,k,3,1)/planedata.planeLocation[i].block(0,k,3,1).norm();
//            planedata.planeLocation[i](3,k) = planedata.planeLocation[i](3,k)*planedata.planeLocation[i].block(0,k,3,1).norm();
//            planedata.planeLocation[i](4,k) = planedata.planeLocation[i](4,k);

//            }
//        }
//    }
    std::cout << "plane data after solver  : \n " <<  planedata.planeLocation[0] << "\n" ;
//    std::cout << "plane data after solver  : \n " <<  planedata.planeLocation[1] << "\n" ;
   singlePlane.segment(0,5) << planedata.planeLocation[0].block(0,0,5,1);

//    singlePlane2.segment(0,3) << planedata.planeLocation[1].block(0,0,3,1)/planedata.planeLocation[1].block(0,0,3,1).norm();
//    singlePlane2[3] =planedata.planeLocation[1](3,0)/planedata.planeLocation[1].block(0,0,3,1).norm();
//    singlePlane2[4] =planedata.planeLocation[1](4,0);
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
    dqMin.setConstant(-0.);
    dqMax.setConstant(0.);
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

    Cst += 1;
    std::vector<Eigen::VectorXd> cstArray_lbA(Cst), cstArray_ubA(Cst);
    std::vector<Eigen::MatrixXd> cstArray_A(Cst);

//    cstArray_lbA[1] =  jnt_pos_cst.getLowerBound();
    cstArray_lbA[0] = jnt_vel_cst.getLowerBound();
//    cstArray_ubA[1] = jnt_pos_cst.getUpperBound();
    cstArray_ubA[0] =jnt_vel_cst.getUpperBound();

//    cstArray_A[1] = jnt_pos_cst.getConstraintMatrix();
    cstArray_A[0] = jnt_vel_cst.getConstraintMatrix();

     // ------------------------------------   Initialize qpOASES solver  ------------------------------------------


    // Define QP solver
    // compute the total number of constraint
    unsigned int cstNbr = 0 ;
    for(size_t t(0) ; t < 1; t++){
        cstNbr += cstArray_lbA[t].size();
    }

    Eigen::MatrixXd H, cstA;
    H.resize(N*ndof,N*ndof);
    cstA.resize(cstNbr,ndof*N);
    cstA.setIdentity();
    Eigen::VectorXd g, lb, ub, lbA, ubA;
    g.resize(N*ndof), lb.resize(N*ndof), ub.resize(N*ndof);
    lbA.resize(cstNbr), ubA.resize(cstNbr);
    lbA.setConstant(-100000);
    ubA.setConstant(100000);


    mpc_solve qpSolver(N, ndof,cstNbr);

    qpSolver.setDefaultOptions();
   qpSolver.initData(H,g,lb,ub);
//    qpSolver.initData(H,g,cstA,lb,ub,lbA,ubA);
    // ------------------------------------   Initialize Ros connextion  ------------------------------------------

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

        pandaState = pandaArm.getRobotState();
        pandaArm.computeqEnlarged(optimalSolution,pandaPx,pandaPu);
        std::cout<<"optimal solution \n" <<optimalSolution<< std::endl;
        qHorizon = pandaArm.getqEnlarged();

        pandaArm.computeJacobianHorz(qHorizon);

        jacobianHorizon = pandaArm.getJacobianHorz();


        pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
        H = pandaTask.getMatrixH();

        g = pandaTask.getVectorg();

        lb.setConstant(-10);
        ub.setConstant(10);

        jnt_pos_cst.update(pandaState,pandaPx,pandaPu);
        jnt_vel_cst.update(pandaState,pandaPxdq,pandaPudq);

        cstArray_lbA[0] = jnt_vel_cst.getLowerBound();
//        cstArray_lbA[1] = jnt_vel_cst.getLowerBound();
        cstArray_ubA[0] = jnt_vel_cst.getUpperBound();
//        cstArray_ubA[1] = jnt_vel_cst.getUpperBound();
        cstArray_A[0] = jnt_vel_cst.getConstraintMatrix();
//        cstArray_A[1] = jnt_vel_cst.getConstraintMatrix();

        lbA.segment(0,cstArray_lbA[0].rows())=  cstArray_lbA[0];
        ubA.segment(0,cstArray_ubA[0].rows()) =  cstArray_ubA[0];
        cstA.block(0,0,cstArray_A[0].rows(),ndof*N) = cstArray_A[0];
//        lbA.segment(cstArray_lbA[0].rows(),cstArray_lbA[1].rows()) =  cstArray_lbA[1];
//        ubA.segment( cstArray_ubA[0].rows(),cstArray_ubA[1].rows()) =  cstArray_ubA[1];
//        cstA.block(cstArray_A[0].rows(),0,cstArray_A[1].rows(),ndof*N) = cstArray_A[1];

//        qpSolver.solve(H,g,cstA,lb,ub,lbA,ubA);
        qpSolver.solve(H,g,lb,ub);

        optimalSolution = qpSolver.getSolution();

        pandaState = stateA*pandaState + stateB*optimalSolution.segment(0,ndof);
        std::cout << "panda state \n" << pandaState << std::endl;

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

        panda_ee_frame = pandaArm.getSegmentPosition("panda_link7");
        robotVerticesPose.poses[0].position.x = panda_ee_frame.p.x();
        robotVerticesPose.poses[0].position.y = panda_ee_frame.p.y();
        robotVerticesPose.poses[0].position.z = panda_ee_frame.p.z();
        cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[0],obsSize[0],obsSize[1],obsSize[2]);
//    cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[1],obsSize[0],obsSize[1],obsSize[2]);
//    cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[2],obsSize[0],obsSize[1],obsSize[2]);
        cubeObstacleMarkers->publishCuboid(cubeLocation,obsSize[0],obsSize[1],obsSize[2],color);

    // Update robot's vertices locations for separating plane problem
        for (int i(0); i < 1; i ++){

            robotVertices[0].block(0,2*i,3,1) <<  panda_ee_frame.p.x(),panda_ee_frame.p.y(),panda_ee_frame.p.z();
            robotVertices[0].block(0,2*i+1,3,1) <<  forearm.p.x(),forearm.p.y(),forearm.p.z();
        }

        for (int i(0);i<N;i++){
            robotVerticesAugmented[0].block(0,2*i,3,2) = robotVertices[0];
        }
        for (int i(0);i<nbrRobotPart; i++){
            for (int j(0);j<nbrObsPart;j++){
                for (int k(0); k < N-1; k++){
//                lpSolver.computeNormCst(robotVerticesAugmented[i].block(0,k,3,2),obsVerticesAugmented.block(0,0,3,2), planedata.planeLocation[i].block(0,k,3,1), epsilon);
//                lpSolver.solve();
//                planedata.planeLocation[i].block(0,k,5,1) = lpSolver.getSolution();
//                planedata.planeLocation[i].block(0,k,3,1) = planedata.planeLocation[i].block(0,k,3,1)/planedata.planeLocation[i].block(0,k,3,1).norm();
//                planedata.planeLocation[i](3,k) = planedata.planeLocation[i](3,k)*planedata.planeLocation[i].block(0,k,3,1).norm();
//                planedata.planeLocation[i](4,k) = planedata.planeLocation[i](4,k);

                lp->setConstraintMatrix(robotVerticesAugmented[i].block(0,2*k,3,4),obsVerticesAugmented.block(0,0,3,2), planedata.planeLocation[i].block(0,k,3,1));
                lp->setEqualityConstraintMatrix();
                lp->loadEqualityMatrix();

//            lp->print();
                lp->solve();
                lpSolution = lp->getSolution();
                planedata.planeLocation[i].block(0,k,5,1) = lp->getSolution();
                planedata.planeLocation[i].block(0,k,3,1) = planedata.planeLocation[i].block(0,k,3,1)/planedata.planeLocation[i].block(0,k,3,1).norm();
                planedata.planeLocation[i](3,k) = planedata.planeLocation[i](3,k)/planedata.planeLocation[i].block(0,k,3,1).norm();
                planedata.planeLocation[i](4,k) = planedata.planeLocation[i](4,k);

                }
            }
        }
        std::cout << "plane data after solver  : \n " <<  planedata.planeLocation[0] << "\n" ;
        singlePlane.segment(0,5) << planedata.planeLocation[0].block(0,0,5,1);



        cubeObstacleMarkers->publishABCDPlane(singlePlane[0],singlePlane[1],singlePlane[2],-singlePlane[3],color,x_width,y_width);
//        cubeObstacleMarkers->publishABCDPlane(0.62,-0.62,0.478,-.61,color,x_width,y_width);

        cubeObstacleMarkers->trigger();
        cubeObstacleMarkers->deleteAllMarkers();
        pandaArm.setState(pandaState.head(ndof),pandaState.tail(ndof));

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout<<"panda joint final is : \n" << pandaArm.getRobotState().segment(0,ndof) << std::endl;
    std::cout<<"panda ee position is :\n " << pandaArm.getSegmentPosition("panda_link7").p << std::endl;
    std::cout<<"Calcul donné par KDL est :\n " << panda_q_des.data << std::endl;
    return 0;
}

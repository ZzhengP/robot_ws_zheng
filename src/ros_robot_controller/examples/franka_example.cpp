#include "robotmodel/RosRobotModel.h"
#include "solver/RosMpcSolver.h"
#include "task/RosMpcTask.h"
#include "constraint/RosGenericCst.h"
#include "constraint/RosJointAccCst.h"
#include "constraint/RosJointVelCst.h"
#include "constraint/RosJointPosCst.h"
#include "constraint/RosCartVelCst.h"
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
#include <chrono>
using namespace qpOASES;
using namespace  plane;
const double pi = 3.1415927;


int main(int argc, char **argv)
{


    // ------------------------------------   Initialize ------------------------------------------
    // Initialize ros node for collision avoidance controller
    ros::init(argc,argv,"panda_safety_controller");
    ros::NodeHandle n;

    double dt(0.04);
    // load robot's kinematic dynamic description
    const std::string& panda_urdf = "/home/zheng/robot_ws_zheng/src/franka_description/robots/panda_arm.urdf";

    // set robot's dof and predictive step
    const int N=1, ndof = 7;
    KDL::JntArray panda_q_init(ndof), panda_dotq_init(ndof),panda_q_des(ndof), panda_q_des_back(ndof), panda_q, panda_dq;

    // initialize robot's joint position and velocity according to roslaunch initialization
    panda_q_init.data << 1.57, 0, 0, -1.8, 0, 1.57, 0;
    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;

    // initialize robot's kinematic class
    arm_kinematic pandaArm(panda_urdf, ndof,"panda_link0", "panda_link8");

    pandaArm.init(panda_q_init.data, panda_dotq_init.data, N);

    // panda's end-effector location and set up desired frame
    KDL::Frame des_frame, back_des_frame, panda_ee_frame, panda_des_frame, base_frame;
    KDL::Frame forearm, elbow, base_link;
    panda_ee_frame = pandaArm.getSegmentPosition(7);
    base_frame = pandaArm.getSegmentPosition(0);
    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    forearm = pandaArm.getSegmentPosition(3);
    base_link = pandaArm.getSegmentPosition(1);
    panda_des_frame.p[0] = 0.5 ;
    panda_des_frame.p[1] = 0.2 ;
    panda_des_frame.p[2] = 0.5 ;
//    panda_des_frame.p[0] = panda_ee_frame.p[0] ;
//    panda_des_frame.p[1] = panda_ee_frame.p[1] ;
//    panda_des_frame.p[2] = panda_ee_frame.p[2] ;
    panda_des_frame.M = base_frame.M;
    panda_des_frame.M.DoRotX(- pi);
    pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);
    std::cout <<" desired q :\n" << panda_q_des.data.transpose() << std::endl;
    // define an cube shape as an obstacle
    rviz_visual_tools::RvizVisualToolsPtr cubeObstacleMarkers;
    rviz_visual_tools::colors color = rviz_visual_tools::RED;
    rviz_visual_tools::colors color2 = rviz_visual_tools::CYAN;


    // put vertices into ros msg to visualize them in rviz
    geometry_msgs::PoseArray robotVerticesPose;
    robotVerticesPose.poses.resize(1);

    robotVerticesPose.poses[0].position.x = panda_ee_frame.p.x();
    robotVerticesPose.poses[0].position.y = panda_ee_frame.p.y();
    robotVerticesPose.poses[0].position.z = panda_ee_frame.p.z();


    // ------------------------------------   Initialize Lps solvers  ------------------------------------------

    int Nv = 5, Nc = 8;
    Eigen::VectorXd lpSolution;
    lpSolution.resize(5);
    std::shared_ptr<lpSolver> lp = std::make_shared<lpSolver>(Nv,Nc);
    lp->setCost();
    lp->setRowConstraint();


    // ------------------------------------   Initialize MPC  ------------------------------------------
    Eigen::VectorXd optimalSolution;

    // Define MPC task
    Eigen::VectorXd pandaState, q_horizon;
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
    q_horizon.resize(ndof*N);

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
    q_horizon = pandaPx*pandaState + pandaPu*optimalSolution;
    pandaArm.setJointHorizon(q_horizon);
    q_horizon = pandaArm.getqEnlarged();

    pandaArm.computeJacobianHorz(q_horizon);
    jacobianHorizon = pandaArm.getJacobianHorz();



    // define mpc constraints
    Eigen::VectorXd ddqMin, ddqMax, ddqLb, ddqUb, dqMin, dqMax, qMin, qMax, vMax, vMin;
    std::vector<constraintData> constraintVectorData;


    ddqMin.resize(N*ndof), ddqMax.resize(N*ndof);
    qMin.resize(N*ndof), qMax.resize(N*ndof);
    qMin.setConstant(-3.14);
    qMax.setConstant(3.14);

    dqMin.resize(N*ndof);
    dqMax.resize(N*ndof);
    dqMin.setConstant(-1);
    dqMax.setConstant(1);

    jntPosCst jointPosCst(ndof,N,dt, "jointPosCst",pandaPx,pandaPu);
    jointPosCst.setLimit(qMin,qMax);
    jointPosCst.setLowerBound(pandaState);
    jointPosCst.setUpperBound(pandaState);
    jointPosCst.setConstraintMatrix();
    constraintVectorData.push_back(jointPosCst.getConstraintData());

    jntVelCst jointVelCst(ndof,N,dt, "jointVelCst",pandaPxdq,pandaPudq);
    jointVelCst.setLimit(dqMin,dqMax);
    jointVelCst.setLowerBound(pandaState);
    jointVelCst.setUpperBound(pandaState);
    jointVelCst.setConstraintMatrix();
    constraintVectorData.push_back(jointVelCst.getConstraintData());


    jntAccCst jnt_acc_cst(ndof, N,dt, "jointAccCst",pandaPx,pandaPu);
    jnt_acc_cst.setLimit(ddqMin,ddqMax);


    Eigen::VectorXd cartVelMin, cartVelMax, dq, dq_horizon ;
    cartVelMin.resize(6*N);
    cartVelMax.resize(6*N);

    cartVelMin.setConstant(-0.5);
    cartVelMax.setConstant(0.5);
    for (int i(0); i << N; i ++ ){
        cartVelMin.segment((i*N+3),3) << -1,-1,-1;
        cartVelMax.segment((i*N+3),3) << 1,1,1;
    }
    dq.resize(ndof);
    dq_horizon.resize(ndof*N);
    q_horizon.resize(ndof*N);
    Eigen::MatrixXd Jacobian;
    dq_horizon = pandaPxdq*pandaState + pandaPudq*optimalSolution;
    pandaArm.setJointVelHorizon(dq_horizon);
    pandaArm.computeJacobian();
    dq_horizon = pandaArm.getdqEnlarged();
    Jacobian.resize(6,7);
    Jacobian = pandaArm.getJacobian().data;
    dq = pandaArm.getJointVel().data;


    cartVelCst cartesianVelCst(ndof,N,dt,"cartesienVelCst",pandaPxdq,pandaPudq);
    cartesianVelCst.setLimit(cartVelMin,cartVelMax);
    cartesianVelCst.setLowerBound(dq_horizon,jacobianHorizon);
    cartesianVelCst.setUpperBound(dq_horizon,jacobianHorizon);
    cartesianVelCst.setConstraintMatrix(jacobianHorizon);
//    constraintVectorData.push_back(cartesianVelCst.getConstraintData());

//    constraintData cartVelCst;
//    cartVelCst.name_ = "cartesienVelCst" ;
//    cartVelCst.lowBound_ = (cartVelMin - Jacobian.block(0,0,3,ndof)*dq)/dt;
//    cartVelCst.upBound_ = (cartVelMax - Jacobian.block(0,0,3,ndof)*dq)/dt;
//    cartVelCst.cstMatrix_ = Jacobian.block(0,0,3,ndof);
//    constraintVectorData.push_back(cartesianVelCst.getConstraintData());

     // ------------------------------------   Initialize qpOASES solver  ------------------------------------------


    // Define QP solver
    // compute the total number of constraint


    Eigen::MatrixXd H ;
    H.resize(N*ndof,N*ndof);
    Eigen::VectorXd g, lb, ub;
    g.resize(N*ndof);
    lb.resize(N*ndof);
    ub.resize(N*ndof);

    pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
    H = pandaTask.getMatrixH();
    g = pandaTask.getVectorg();


    lb.setConstant(-10);
    ub.setConstant(10);


    mpc_solve qptest(1,ndof*N,2*ndof*N);
    qptest.initData(H,g,lb,ub);
    qptest.constructProblem(constraintVectorData,H,g);
    qptest.setDefaultOptions();


    // ------------------------------- Let's start simulation --------------------
    // Test for velocity
    Eigen::VectorXd linear_vel;
    linear_vel.resize(6);

    std::ofstream myfile, myfile2, myfile3;
     myfile.open ("/home/zheng/Bureau/jointvel.txt");
     myfile2.open ("/home/zheng/Bureau/acc.txt");
     myfile3.open("/home/zheng/Bureau/cartvel.txt");
    bool is_solved;
    int ite=0;
    while(ite < 5000)
    {

//        pandaState = pandaArm.getRobotState();

        pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
        H = pandaTask.getMatrixH();
        g = pandaTask.getVectorg();


        pandaArm.computeJacobian();
        Jacobian = pandaArm.getJacobian().data;
        dq = pandaArm.getJointVel().data;

        pandaArm.computeJacobianHorz(q_horizon);
        jacobianHorizon = pandaArm.getJacobianHorz();

        jointPosCst.update(pandaState);
        jointVelCst.update(pandaState);
        cartesianVelCst.update(dq_horizon,jacobianHorizon);


//        cartVelCst.lowBound_ = (cartVelMin - Jacobian.block(0,0,3,ndof)*dq)/dt;
//        cartVelCst.upBound_ = (cartVelMax - Jacobian.block(0,0,3,ndof)*dq)/dt;
//        cartVelCst.cstMatrix_ = Jacobian.block(0,0,3,ndof);

//        std::cout << "lbA cart vel debug :\n" <<(cartVelMin - Jacobian.block(0,0,3,ndof)*dq)/dt <<std::endl;
//        std::cout << "ubA cart vel debug :\n" <<(cartVelMax - Jacobian.block(0,0,3,ndof)*dq)/dt <<std::endl;
//        std::cout << "A cart vel debug :\n" <<Jacobian.block(0,0,3,ndof) <<std::endl;

//        std::cout << "lbA cart vel  :\n" <<cartesianVelCst.getConstraintData().lowBound_<<std::endl;
//        std::cout << "ubA cart vel  :\n" <<cartesianVelCst.getConstraintData().upBound_<<std::endl;
//        std::cout << "A cart vel  :\n" <<cartesianVelCst.getConstraintData().cstMatrix_ <<std::endl;

        constraintVectorData[0] = jointPosCst.getConstraintData();
        constraintVectorData[1] = jointVelCst.getConstraintData();
//        constraintVectorData[2] = cartesianVelCst.getConstraintData();

//        std::cout <<" constraint number \n" << constraintVectorData.size() << std::endl;
//        for (int i(0); i < constraintVectorData.size(); i++) {
//                std::cout <<" constraint : " << i << std::endl;
//                constraintVectorData[i].print();
//        }

        qptest.constructProblem(constraintVectorData,H,g);
        auto start = std::chrono::high_resolution_clock::now();
        is_solved = qptest.solve();
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        if (!is_solved){
            std::cout <<" qp failed :\n" << std::endl;
            std::cout <<" iteration :\n" << ite << std::endl;
            optimalSolution.setZero();
            std::cout<<"optimal solution \n" <<optimalSolution<< std::endl;
            break;

        }else {
                optimalSolution = qptest.getSolution();
}
        std::cout<<"optimal solution \n" <<optimalSolution<< std::endl;


        linear_vel = Jacobian*pandaState.tail(ndof);

        q_horizon = pandaPx*pandaState + pandaPu*optimalSolution;
        pandaArm.setJointHorizon(q_horizon);

        dq_horizon = pandaPxdq*pandaState + pandaPudq*optimalSolution;
        pandaArm.setJointVelHorizon(dq_horizon);

        q_horizon = pandaArm.getqEnlarged();
        dq_horizon = pandaArm.getdqEnlarged();


        myfile<<pandaState.tail(7).transpose()<<'\n' ;
//        myfile2<<optimalSolution.transpose() << '\n'
//              <<"--------------------------------------------------------------------------------------------------------------" << '\n' ;
        myfile3 << linear_vel.transpose() << '\n';

        pandaState = stateA*pandaState + stateB*optimalSolution.segment(0,ndof);
        pandaArm.setState(pandaState.head(ndof),pandaState.tail(ndof));
        ite++;
    }
    myfile.close();
    myfile2.close();
    myfile3.close();
    std::cout<<"panda joint final is : \n" << pandaArm.getRobotState().segment(0,ndof) << std::endl;
    std::cout<<"panda ee position is :\n " << pandaArm.getSegmentPosition(7).p << std::endl;
    std::cout<<"Calcul donné par KDL est :\n " << panda_q_des.data << std::endl;
    return 0;
}



//int main(int argc, char **argv)
//{


//    // ------------------------------------   Initialize ------------------------------------------
//    // Initialize ros node for collision avoidance controller
//    ros::init(argc,argv,"panda_safety_controller");
//    ros::NodeHandle n;

//    double dt(0.01);
//    // load robot's kinematic dynamic description
//    const std::string& panda_urdf = "/home/zheng/robot_ws_zheng/src/franka_description/robots/panda_arm.urdf";

//    // set robot's dof and predictive step
//    const int N=1, ndof = 7;
//    KDL::JntArray panda_q_init(ndof), panda_dotq_init(ndof),panda_q_des(ndof), panda_q_des_back(ndof), panda_q, panda_dq;

//    // initialize robot's joint position and velocity according to roslaunch initialization
//    panda_q_init.data << 1.57, 0, 0, -1.8, 0, 1.57, 0;
//    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;

//    // initialize robot's kinematic class
//    arm_kinematic pandaArm(panda_urdf, ndof,"panda_link0", "panda_link8");

//    pandaArm.init(panda_q_init.data, panda_dotq_init.data, N);

//    // panda's end-effector location and set up desired frame
//    KDL::Frame des_frame, back_des_frame, panda_ee_frame, panda_des_frame, base_frame;
//    KDL::Frame forearm, elbow, base_link;
//    panda_ee_frame = pandaArm.getSegmentPosition(7);
//    base_frame = pandaArm.getSegmentPosition(0);
//    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
//    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
//    forearm = pandaArm.getSegmentPosition(3);
//    base_link = pandaArm.getSegmentPosition(1);
//    panda_des_frame.p[0] = 0.5 ;
//    panda_des_frame.p[1] = 0.2 ;
//    panda_des_frame.p[2] = 0.5 ;
////    panda_des_frame.p[0] = panda_ee_frame.p[0] ;
////    panda_des_frame.p[1] = panda_ee_frame.p[1] ;
////    panda_des_frame.p[2] = panda_ee_frame.p[2] ;
//    panda_des_frame.M = base_frame.M;
//    panda_des_frame.M.DoRotX(- pi);
//    pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);
//    std::cout <<" desired q :\n" << panda_q_des.data.transpose() << std::endl;
//    // define an cube shape as an obstacle
//    rviz_visual_tools::RvizVisualToolsPtr cubeObstacleMarkers;
//    rviz_visual_tools::colors color = rviz_visual_tools::RED;
//    rviz_visual_tools::colors color2 = rviz_visual_tools::CYAN;


//    // put vertices into ros msg to visualize them in rviz
//    geometry_msgs::PoseArray robotVerticesPose;
//    robotVerticesPose.poses.resize(1);

//    robotVerticesPose.poses[0].position.x = panda_ee_frame.p.x();
//    robotVerticesPose.poses[0].position.y = panda_ee_frame.p.y();
//    robotVerticesPose.poses[0].position.z = panda_ee_frame.p.z();


//    // ------------------------------------   Initialize Lps solvers  ------------------------------------------

//    int Nv = 5, Nc = 8;
//    Eigen::VectorXd lpSolution;
//    lpSolution.resize(5);
//    std::shared_ptr<lpSolver> lp = std::make_shared<lpSolver>(Nv,Nc);
//    lp->setCost();
//    lp->setRowConstraint();


//    // ------------------------------------   Initialize MPC  ------------------------------------------
//    Eigen::VectorXd optimalSolution;

//    // Define MPC task
//    Eigen::VectorXd pandaState, qHorizon;
//    bool panda_ok;
//    MPC_Task pandaTask(N,1,0.001,ndof,dt, "panda");
//    panda_ok = pandaTask.init();

//    // Augmented state
//    Eigen::MatrixXd pandaPx, pandaPu, pandaPxdq, pandaPudq;
//    pandaPx.resize(N*ndof,2*ndof);
//    pandaPu.resize(N*ndof,N*ndof);
//    pandaPx = pandaTask.getMatrixPx();
//    pandaPu = pandaTask.getMatrixPu();

//    pandaPxdq.resize(N*ndof,2*ndof);
//    pandaPudq.resize(N*ndof,N*ndof);
//    pandaPxdq = pandaTask.getMatrixPxdq();
//    pandaPudq = pandaTask.getMatrixPudq();

//    pandaState.resize(2*ndof);
//    pandaState = pandaArm.getRobotState();

//    optimalSolution.resize(ndof*N);
//    optimalSolution.setZero();
//    qHorizon.resize(ndof*N);

//    Eigen::MatrixXd jacobianHorizon;
//    jacobianHorizon.resize(N*6, N*ndof);

//    Eigen::VectorXd qHorizonDes;
//    qHorizonDes.resize(ndof*N);
//    for (size_t i=0; i<N; i++)
//    {
//         qHorizonDes.segment(ndof*i,ndof) = panda_q_des.data;
//    }

//    Eigen::MatrixXd  stateA, stateB;
//    stateA = pandaTask.getStateA();
//    stateB = pandaTask.getStateB();
//    pandaArm.computeqEnlarged(optimalSolution,pandaPx,pandaPu);
//    qHorizon = pandaPx*pandaState + pandaPu*optimalSolution;
//    pandaArm.computeJacobianHorz(qHorizon);
//    jacobianHorizon = pandaArm.getJacobianHorz();
//    // define mpc constraints
//    int Cst = 0;
//    Eigen::VectorXd ddqMin, ddqMax, ddqLb, ddqUb, dqMin, dqMax, qMin, qMax, vMax, vMin;

//    ddqMin.resize(N*ndof), ddqMax.resize(N*ndof);
//    ddqLb.resize(N*ndof), ddqUb.resize(N*ndof);

//    qMin.resize(N*ndof), qMax.resize(N*ndof);
//    qMin.setConstant(-3.14);
//    qMax.setConstant(3.14);

//    dqMin.resize(N*ndof);
//    dqMax.resize(N*ndof);
//    for (int i(0) ; i < N; i++){
//        dqMin.segment(ndof*i,ndof) << -pi/10,-pi/10,-pi/10,-pi/10, -pi/10, -pi/10, -pi/10;
//        dqMax.segment(ndof*i,ndof) << pi/10, pi/10, pi/10, pi/10, pi/10, pi/10, pi/10;
//    }
//    dqMin.setConstant(-0.2);
//    dqMax.setConstant(0.2);
//    KDL::Jacobian Jacobian;
//    Jacobian.resize(ndof);
//    panda_q.resize(ndof);
//    panda_dq.resize(ndof);
//    // Define joint acceleration limit
//    jntAccCst jnt_acc_cst(ndof, N,dt);
//    jnt_acc_cst.setLimit(ddqMin,ddqMax);
//    pandaArm.computeJacobian();
//    Jacobian = pandaArm.getJacobian();

//    // Define joint velocity limit


//     // ------------------------------------   Initialize qpOASES solver  ------------------------------------------


//    // Define QP solver
//    // compute the total number of constraint


//    Eigen::MatrixXd H,cstA,cstAdq,cstdq ;
//    H.resize(N*ndof,N*ndof);
//    Eigen::VectorXd g, lb, ub, lbdq, ubdq, lbState, ubState, ubVel, lbq,ubq;
//    g.resize(N*ndof);
//    lb.resize(N*ndof);
//    ub.resize(N*ndof);
//    pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
//    H = pandaTask.getMatrixH();
//    g = pandaTask.getVectorg();
//    cstA.resize(N*ndof,N*ndof);
//    cstAdq.resize(N*ndof,N*ndof);
//    cstdq.resize(N*ndof,ndof);
//    lbdq.resize(N*ndof);
//    ubdq.resize(N*ndof);
//    lb.setConstant(-10);
//    ub.setConstant(10);
//    lbq.resize(7);
//    ubq.resize(7);
//    panda_q.data = pandaState.head(ndof);
//    panda_dq = pandaArm.getJointVel();
//    lbState.resize(14);
//    ubState.resize(14);
////    pandaArm.computeJacobian(panda_q,Jacobian);

////    lbA = dqMin - pandaPxdq*pandaState ;
////    ubA = dqMax - pandaPxdq*pandaState ;
////    cstA = pandaPudq ;
//    // ======================  test of new formulation of constraints ==================================
//    cstAdq =pandaPudq ;
////    for (int i(0);i<N;i++){
////        for (int j(0); j <i+1;j++){
////            cstAdq.block(i*ndof,j*ndof,ndof,ndof).setIdentity();
////        }
////        cstdq.block(i*ndof,0,ndof,ndof).setIdentity();
////    }
//    lbdq = dqMin - pandaPxdq*pandaState;
//    ubdq = dqMax -  pandaPxdq*pandaState;

//    lbState.segment(0,7)= qMin;
//    lbState.segment(7,7) = dqMin;

//    ubState.segment(0,7) = qMax;
//    ubState.segment(7,7) = dqMax;

//    // Constraint for joint position
//    cstA = pandaPu;
//    lbq = qMin - pandaPx*pandaState;
//    ubq = qMax - pandaPx*pandaState;
//    // Define total constraint
//    Eigen::VectorXd lbA, ubA;
//    Eigen::MatrixXd A;
//    lbA.resize(14);
//    ubA.resize(14);

//    A.resize(7,7);
//    A.block(0,0,7,7) = cstA;
////    A.block(7,0,7,7) = cstAdq;

//    lbA = lbState - stateA*pandaState ;
////    lbA.segment(7,7) = lbdq;
//    ubA = ubState - stateA*pandaState ;
////    ubA.segment(7,7) = ubdq;
//    // ndof*N constraint here, because only variable associated to velocity
//    mpc_solve qptest(1,7,14);
//    qptest.initData(H,g,stateB,lb,ub,lbA,ubA);

//    // Test for velocity
//    Eigen::VectorXd linear_vel;
//    linear_vel.resize(6);

//    std::ofstream myfile, myfile2, myfile3;
//     myfile.open ("/home/zheng/Bureau/jointvel.txt");
//     myfile2.open ("/home/zheng/Bureau/acc.txt");
//     myfile3.open("/home/zheng/Bureau/cartvel.txt");
//    bool is_solved;
//    int ite=0;
//    while(ite < 5000)
//    {

////        pandaState = pandaArm.getRobotState();
//        pandaArm.computeqEnlarged(optimalSolution,pandaPx,pandaPu);
//        std::cout<<"optimal solution \n" <<optimalSolution<< std::endl;
//        qHorizon = pandaArm.getqEnlarged();

//        pandaArm.computeJacobian();
//        Jacobian = pandaArm.getJacobian();
//        panda_dq = pandaArm.getJointVel();

//        pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
//        H = pandaTask.getMatrixH();
//        g = pandaTask.getVectorg();




//        lbdq = dqMin - pandaPxdq*pandaState;
//        ubdq = dqMax -  pandaPxdq*pandaState;


//        // Constraint for joint position
//        lbq = qMin - pandaPx*pandaState;
//        ubq = qMax - pandaPx*pandaState;
//        lbA = lbState - stateA*pandaState ;
//    //    lbA.segment(7,7) = lbdq;
//        ubA = ubState - stateA*pandaState ;
//        lbA.segment(0,7) = lbq;
////        lbA.segment(7,7) = lbdq;
//        ubA.segment(0,7) = ubq;
////        ubA.segment(7,7) = ubdq;
//        // Constraint for joint position

////        lbq = 2*(qMin - pandaState.head(ndof) - pandaState.tail(ndof)*dt)/(dt*dt);
////        ubq = 2*(qMax - pandaState.head(ndof) - pandaState.tail(ndof)*dt)/(dt*dt);

////        qpSolver.initData(H,g,A,lb,ub,lbA,ubA);
//        std::cout << "A :\n" << stateB << std::endl;
//        std::cout << "lbA :\n" << lbA << std::endl;
//        std::cout << "ubA :\n" << ubA << std::endl;

//        is_solved = qptest.solve(H,g,stateB,lb,ub,lbA,ubA);

//        if (!is_solved){
//            std::cout <<" qp failed :\n" << std::endl;
//            std::cout <<" iteration :\n" << ite << std::endl;
//            optimalSolution.setZero();
//            std::cout<<"optimal solution \n" <<optimalSolution<< std::endl;

//        }else {
//                optimalSolution = qptest.getSolution();
//}

//        pandaState = stateA*pandaState + stateB*optimalSolution;

//        myfile<<pandaState.tail(7).transpose()<<'\n' ;
//        myfile2<<optimalSolution.transpose() << '\n'
//              <<"--------------------------------------------------------------------------------------------------------------" << '\n' ;
//        myfile3 << linear_vel.transpose() << '\n';
//        pandaArm.setState(pandaState.head(ndof),pandaState.tail(ndof));
//        ite++;
//    }
//    myfile.close();
//    myfile2.close();
//    myfile3.close();
//    std::cout<<"panda joint final is : \n" << pandaArm.getRobotState().segment(0,ndof) << std::endl;
//    std::cout<<"panda ee position is :\n " << pandaArm.getSegmentPosition(7).p << std::endl;
//    std::cout<<"Calcul donné par KDL est :\n " << panda_q_des.data << std::endl;
//    return 0;
//}

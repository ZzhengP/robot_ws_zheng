#include "robotmodel/RosRobotModel.h"
#include "solver/RosMpcSolver.h"
#include "solver/RosPlaneSolver.h"
#include "task/RosMpcTask.h"
#include "constraint/RosGenericCst.h"
#include "constraint/RosJointAccCst.h"
#include "constraint/RosJointVelCst.h"
#include "constraint/RosJointPosCst.h"
#include "constraint/RosCartVelCst.h"
#include "constraint/RosAvoidanceCst.h"
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
#include <math.h>
#include "nav_msgs/Path.h"
using namespace  std;
using namespace qpOASES;
using namespace  plane;
const double pi = 3.1415927;


int main(int argc, char **argv)
{


    // ------------------------------------   Initialize ------------------------------------------
    // Initialize ros node for collision avoidance controller
    ros::init(argc,argv,"panda_safety_controller");
    ros::NodeHandle n;

    double dt(0.05);
    // load robot's kinematic dynamic description
    const std::string& panda_urdf = "/home/zheng/robot_ws_zheng/src/franka_description/robots/panda_arm.urdf";

    // set robot's dof and predictive step
    const int N=5, ndof = 7;
    KDL::JntArray panda_q_init(ndof), panda_dotq_init(ndof),panda_q_des(ndof), panda_q_des_back(ndof);

    // initialize robot's joint position and velocity according to roslaunch initialization
    //panda_q_init.data << 1.57, 0, 0, -1.8, 0, 1.57, 0;
    panda_q_init.data<<0.5, 0., 0., -1.8,0.,1.57 , 0. ;

    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;

    // initialize robot's kinematic class
    arm_kinematic pandaArm(panda_urdf, ndof,"panda_link0", "panda_link8");

    pandaArm.init(panda_q_init.data, panda_dotq_init.data, N);

    // panda's end-effector location and set up desired frame
    KDL::Frame des_frame, back_des_frame, panda_ee_frame, panda_des_frame, panda_obs_frame;
    KDL::Frame forearm, elbow, base_link;
    panda_obs_frame = pandaArm.getSegmentPosition(7);
    panda_ee_frame = pandaArm.getSegmentPosition(8);
    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    panda_obs_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    forearm = pandaArm.getSegmentPosition(3);
    base_link = pandaArm.getSegmentPosition(1);


    panda_des_frame.p[0] = 0.470808;
    panda_des_frame.p[1] = -0.3 ;
    panda_des_frame.p[2] = 0.517842 ;
//    panda_des_frame.p[0] = panda_ee_frame.p[0] ;
//    panda_des_frame.p[1] = panda_ee_frame.p[1] ;
//    panda_des_frame.p[2] = panda_ee_frame.p[2] ;
    panda_des_frame.M = panda_des_frame.M;
    panda_des_frame.M.DoRotX(- pi);
    pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);

    // define an cube shape as an obstacle
    rviz_visual_tools::RvizVisualToolsPtr cubeObstacleMarkers;
    rviz_visual_tools::colors color = rviz_visual_tools::RED;
    rviz_visual_tools::colors color2 = rviz_visual_tools::CYAN;
    rviz_visual_tools::colors colorPlane1 = rviz_visual_tools::TRANSLUCENT_DARK;
    rviz_visual_tools::colors colorPlane2 = rviz_visual_tools::RED;
    rviz_visual_tools::colors colorPlane3 = rviz_visual_tools::CYAN;
    rviz_visual_tools::colors colorPlane4 = rviz_visual_tools::TRANSLUCENT_LIGHT;

    cubeObstacleMarkers.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/simpleCube"));
    // define cube location (we suppose this is only a point)
    Eigen::Vector3d cube ;
    geometry_msgs::Pose cubeLocation, forearmLocation;
    cubeLocation.position.x = 0.5;
    cubeLocation.position.y = 0.0;
//    cubeLocation.position.z = forearm.p.z();
    cubeLocation.position.z = 0.517842;
    cube << cubeLocation.position.x, cubeLocation.position.y, cubeLocation.position.z;
    // define cube's size (x,y,z)
    Eigen::Vector3d obsSize;
    obsSize << 0.05,0.05,0.05;
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
    // We choose only the end-effector position with a augmented
    std::vector<Eigen::MatrixXd> robotVertices, robotVerticesAugmented  ;
    robotVertices.resize(1);
    for (int i(0); i<robotVertices.size();i++){
        // 3 means there is two vertices to determine robot's link + the base
        robotVertices[i].resize(3,2);
    }

    for (int i(0); i < 1; i ++){

        robotVertices[0].block(0,2*i,3,1) <<  panda_ee_frame.p.x(),panda_ee_frame.p.y(),panda_ee_frame.p.z();
        robotVertices[0].block(0,2*i+1,3,1) <<  forearm.p.x(),forearm.p.y(),forearm.p.z();
//        robotVertices[0].block(0,2*i+1,3,1) <<panda_ee_frame.p.x(),panda_ee_frame.p.y(),panda_ee_frame.p.z();
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
    robotVerticesPose.poses.resize(2);

    robotVerticesPose.poses[0].position.x = panda_ee_frame.p.x();
    robotVerticesPose.poses[0].position.y = panda_ee_frame.p.y();
    robotVerticesPose.poses[0].position.z = panda_ee_frame.p.z();
    robotVerticesPose.poses[1].position.x = forearm.p.x();
    robotVerticesPose.poses[1].position.y = forearm.p.y();
    robotVerticesPose.poses[1].position.z = forearm.p.z();

    // ------------------------------------   Initialize plane  ------------------------------------------

    int nbrRobotPart = 1, nbrObsPart=1;

    Planes plane(N,nbrRobotPart,nbrObsPart,robotVertices,obsVertices);
    Eigen::VectorXd singlePlane, singlePlane2;
    singlePlane.resize(5);
    singlePlane2.resize(5);
    double x_width = 2, y_width = 2;
    std::cout <<FYEL("here  : \n") << std::endl;
    Planes::PlaneData planedata,planedataIni ;
    planedata = plane.getPlanes();


    singlePlane << planedata.planeLocation[0].block(0,0,5,1);
    planedataIni = planedata;

//    cubeObstacleMarkers->publishABCDPlane(singlePlane[0],singlePlane[1],singlePlane[2],-singlePlane[3],colorPlane,x_width,y_width);
//    cubeObstacleMarkers->trigger();

//    singlePlane2 << planedata.planeLocation[1].block(0,0,5,1);
    // ------------------------------------   Initialize Lps solvers  ------------------------------------------
    double dsafe = 0.05;
    PlaneSolver planeSolver(10,dsafe);
    Eigen::VectorXd gPlane;
    gPlane.resize(5);
    gPlane.setZero();
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
    q_horizon = pandaPx*pandaState + pandaPu*optimalSolution;

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
    dqMin.setConstant(-0.5);
    dqMax.setConstant(0.5);

    jntPosCst jointPosCst(ndof,N,dt, "jointPosCst",pandaPx,pandaPu);
    jointPosCst.setLimit(qMin,qMax);
    jointPosCst.setLowerBound(pandaState);
    jointPosCst.setUpperBound(pandaState);
    jointPosCst.setConstraintMatrix();
    jointPosCst.getConstraintData().print();
    constraintVectorData.push_back(jointPosCst.getConstraintData());

    jntVelCst jointVelCst(ndof,N,dt, "jointVelCst",pandaPxdq,pandaPudq);
    jointVelCst.setLimit(dqMin,dqMax);
    jointVelCst.setLowerBound(pandaState);
    jointVelCst.setUpperBound(pandaState);
    jointVelCst.setConstraintMatrix();
    jointVelCst.getConstraintData().print();
    constraintVectorData.push_back(jointVelCst.getConstraintData());

    // Define joint acceleration limit
    jntAccCst jnt_acc_cst(ndof, N,dt, "jointAccCst",pandaPx,pandaPu);
    jnt_acc_cst.setLimit(ddqMin,ddqMax);

    Eigen::VectorXd cartVelMin, cartVelMax, dq, dq_horizon ;
//    cartVelMin.resize(6*N);
//    cartVelMax.resize(6*N);

//    cartVelMin.setConstant(-0.2);
//    cartVelMax.setConstant(0.2);

    dq.resize(ndof);
    dq_horizon.resize(ndof*N);
    q_horizon.resize(ndof*N);
    Eigen::MatrixXd Jacobian, C;
    dq_horizon = pandaPxdq*pandaState + pandaPudq*optimalSolution;
    pandaArm.setJointVelHorizon(dq_horizon);
    pandaArm.computeJacobian();
    dq_horizon = pandaArm.getdqEnlarged();
    Jacobian.resize(6,7);
    Jacobian = pandaArm.getJacobian().data;
    dq = pandaArm.getJointVel().data;


//    cartVelCst cartesianVelCst(ndof,N,dt,"cartesienVelCst",pandaPxdq,pandaPudq);
//    cartesianVelCst.setLimit(cartVelMin,cartVelMax);
//    cartesianVelCst.setLowerBound(dq_horizon,jacobianHorizon);
//    cartesianVelCst.setUpperBound(dq_horizon,jacobianHorizon);
//    cartesianVelCst.setConstraintMatrix(jacobianHorizon);
//    constraintVectorData.push_back(cartesianVelCst.getConstraintData());

    ObsAvoidanceCSt obsAvoidanceCst(ndof,N,dt,dsafe,"ObsAvoidanceConstraint",pandaPx,pandaPu);

    obsAvoidanceCst.setLowerBound();
    obsAvoidanceCst.setUpperBoundAndConstraint(robotVerticesAugmented,planedata,pandaState,jacobianHorizon,q_horizon);
    constraintVectorData.push_back(obsAvoidanceCst.getConstraintData());

     // ------------------------------------   Initialize qpOASES solver  ------------------------------------------

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
    int constraintSize = 2*ndof*N + obsAvoidanceCst.getConstraintData().upBound_.size();
    mpc_solve qptest(1,ndof*N,constraintSize);
    qptest.initData(H,g,lb,ub);
    qptest.constructProblem(constraintVectorData,H,g);
    qptest.setDefaultOptions();

    // ------------------------------------   Initialize Ros connextion  ------------------------------------------

    ros::Rate loop_rate(1000);

    // create ros topic for sending msg
    ros::Publisher panda_joint_state_1_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint1_position_controller/command", 1000);
    ros::Publisher panda_joint_state_2_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint2_position_controller/command", 1000);
    ros::Publisher panda_joint_state_3_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint3_position_controller/command", 1000);
    ros::Publisher panda_joint_state_4_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint4_position_controller/command", 1000);
    ros::Publisher panda_joint_state_5_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint5_position_controller/command", 1000);
    ros::Publisher panda_joint_state_6_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint6_position_controller/command", 1000);
    ros::Publisher panda_joint_state_7_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint7_position_controller/command", 1000);

    rviz_visual_tools::RvizVisualToolsPtr PathPublisher;
    PathPublisher.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/mpcPath"));


    std::vector<geometry_msgs::Pose> path;
    path.resize(N);
    bool is_solved, is_plane_solved;
    int ite=0;
    std::ofstream myfile, myfile2;
    myfile.open ("/home/zheng/Bureau/jointvel.txt");
    myfile2.open ("/home/zheng/Bureau/acc.txt");


    Eigen::VectorXd solution;
    solution.resize(5);


    Eigen::VectorXd eePosHorz ;
    eePosHorz.resize(3*N);

    while(ros::ok())
    {
        pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
        H = pandaTask.getMatrixH();
        g = pandaTask.getVectorg();

        pandaArm.computeJacobian();
        Jacobian = pandaArm.getJacobian().data;
        dq = pandaArm.getJointVel().data;

        pandaArm.computeJacobianHorz(q_horizon);
        jacobianHorizon = pandaArm.getJacobianHorz();
//        for (int k(0); k < N; k ++ ){
//            jacobianHorizon.block(6*k,7*k,6,7) = Jacobian;
//        }
//        myfile2<< "jacobian horizon :\n" <<jacobianHorizon << '\n';

        jointPosCst.update(pandaState);
        jointVelCst.update(pandaState);
        obsAvoidanceCst.setUpperBoundAndConstraint(robotVerticesAugmented,planedata,pandaState,jacobianHorizon,q_horizon);


        constraintVectorData[0] = jointPosCst.getConstraintData();
        constraintVectorData[1] = jointVelCst.getConstraintData();
        constraintVectorData[2] = obsAvoidanceCst.getConstraintData();

        qptest.constructProblem(constraintVectorData,H,g);
//        qptest.print();
        obsAvoidanceCst.getConstraintData().print();


        is_solved = qptest.solve();

        if (!is_solved){
            std::cout <<" qp failed :\n" << std::endl;
            std::cout <<" iteration :\n" << ite << std::endl;
            optimalSolution.setZero();
            std::cout<<"optimal solution \n" <<optimalSolution<< std::endl;
            break;

        }else {
                optimalSolution = qptest.getSolution();
}

        q_horizon = pandaPx*pandaState + pandaPu*optimalSolution;
        pandaArm.setJointHorizon(q_horizon);

        dq_horizon = pandaPxdq*pandaState + pandaPudq*optimalSolution;
        pandaArm.setJointVelHorizon(dq_horizon);

        q_horizon = pandaArm.getqEnlarged();
        dq_horizon = pandaArm.getdqEnlarged();



        panda_ee_frame = pandaArm.getSegmentPosition(8);
        forearm = pandaArm.getSegmentPosition(3);
        base_link = pandaArm.getSegmentPosition(1);
        robotVerticesPose.poses[0].position.x = panda_ee_frame.p.x();
        robotVerticesPose.poses[0].position.y = panda_ee_frame.p.y();
        robotVerticesPose.poses[0].position.z = panda_ee_frame.p.z();
        robotVerticesPose.poses[1].position.x = forearm.p.x();
        robotVerticesPose.poses[1].position.y = forearm.p.y();
        robotVerticesPose.poses[1].position.z = forearm.p.z();
        cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[0],obsSize[0],obsSize[1],obsSize[2]);
//        cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[1],obsSize[0],obsSize[1],obsSize[2]);

//    cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[1],obsSize[0],obsSize[1],obsSize[2]);
//    cubeObstacleMarkers->publishCuboid(robotVerticesPose.poses[2],obsSize[0],obsSize[1],obsSize[2]);
        cubeObstacleMarkers->publishCuboid(cubeLocation,obsSize[0],obsSize[1],obsSize[2],color);


    // Update robot's vertices locations for separating plane problem
        for (int i(0); i < 1; i ++){

            robotVertices[0].block(0,2*i,3,1) <<  panda_ee_frame.p.x(),panda_ee_frame.p.y(),panda_ee_frame.p.z();
            robotVertices[0].block(0,2*i+1,3,1) <<  forearm.p.x(),forearm.p.y(),forearm.p.z();
//            robotVertices[0].block(0,2*i+1,3,1) <<panda_ee_frame.p.x(),panda_ee_frame.p.y(),panda_ee_frame.p.z();

        }
        pandaArm.computeCartPosHorz(q_horizon);
        eePosHorz = pandaArm.getCartPosHorz();

        for (int i(0);i<N;i++){
            robotVerticesAugmented[0].block(0,2*i,3,1) << eePosHorz.segment(3*i,3);
            robotVerticesAugmented[0].block(0,2*i+1,3,1) <<  forearm.p.x(),forearm.p.y(),forearm.p.z();
            path[i].position.x =  eePosHorz(3*i);
            path[i].position.y =  eePosHorz(3*i+1);
            path[i].position.z =  eePosHorz(3*i+2);

        }
        for (int i(0);i<nbrRobotPart; i++){
            for (int j(0);j<nbrObsPart;j++){
                for (int k(0); k < N-1; k++){

//                lp->setConstraintMatrix(robotVerticesAugmented[i].block(0,3*k,3,6),obsVerticesAugmented.block(0,0,3,2), planedataIni.planeLocation[i].block(0,k,3,1));
//                lp->setConstraintMatrix(robotVerticesAugmented[i].block(0,3*k,3,6),obsVerticesAugmented.block(0,0,3,2), planedata.planeLocation[i].block(0,k,3,1));
//                lp->setEqualityConstraintMatrix();
//                lp->loadEqualityMatrix();

//            lp->print();
//                lp->solve();
//                lpSolution = lp->getSolution();
//                planedata.planeLocation[i].block(0,k,5,1) = lp->getSolution();
//                planedata.planeLocation[i].block(0,k,3,1) = planedata.planeLocation[i].block(0,k,3,1)/planedata.planeLocation[i].block(0,k,3,1).norm();
//                planedata.planeLocation[i](3,k) = planedata.planeLocation[i](3,k)*planedata.planeLocation[i].block(0,k,3,1).norm();
//                std::cout <<" plane precedent :\n " << planedata.planeLocation[i] << std::endl;

                gPlane <<singlePlane.segment(0,4),0 ;
                planeSolver.setCost(gPlane);
                planeSolver.setCstMatrix(robotVerticesAugmented[i].block(0,2*k,3,4),obsVerticesAugmented.block(0,0,3,2),
                                         planedata.planeLocation[i].block(0,k,3,1));
                is_plane_solved = planeSolver.solve();
                planedata.planeLocation[i].block(0,k,5,1) = planeSolver.getSolution();
//                planedata.planeLocation[i].block(0,k,5,1) = planeSolver.getSolution();
                planedata.planeLocation[i].block(0,k,3,1) = planedata.planeLocation[i].block(0,k,3,1)/planedata.planeLocation[i].block(0,k,3,1).norm();
                planedata.planeLocation[i](3,k) = planedata.planeLocation[i](3,k)*planedata.planeLocation[i].block(0,k,3,1).norm();
                if (!is_plane_solved){
                    std::cout <<" plane solution failed :\n" << std::endl;
                    break;}
                }

                }
            }

        singlePlane.segment(0,5) << planedata.planeLocation[0].block(0,0,5,1);

//        std::cout <<" plane found : \n " << planedata.planeLocation[0].block(0,0,5,1) <<'\n';

        cubeObstacleMarkers->publishABCDPlane(singlePlane[0],singlePlane[1],singlePlane[2],-singlePlane[3],colorPlane1,x_width,y_width);
        cubeObstacleMarkers->publishABCDPlane(planedata.planeLocation[0](0,1),planedata.planeLocation[0](1,1),planedata.planeLocation[0](2,1)
                                              ,-planedata.planeLocation[0](3,1),colorPlane2,x_width,y_width);
        cubeObstacleMarkers->publishABCDPlane(planedata.planeLocation[0](0,2),planedata.planeLocation[0](1,2),planedata.planeLocation[0](2,2)
                                              ,-planedata.planeLocation[0](3,2),colorPlane3,x_width,y_width);
        cubeObstacleMarkers->publishABCDPlane(planedata.planeLocation[0](0,3),planedata.planeLocation[0](1,3),planedata.planeLocation[0](2,3)
                                              ,-planedata.planeLocation[0](3,3),colorPlane4,x_width,y_width);
//        cubeObstacleMarkers->publishABCDPlane(0.62,-0.62,0.478,-.61,color,x_width,y_width);
        PathPublisher->publishPath(path);
        PathPublisher->trigger();
//        PathPublisher->deleteAllMarkers();
        cubeObstacleMarkers->trigger();
        cubeObstacleMarkers->deleteAllMarkers();

        Eigen::Vector3d ee_pose ;
        ee_pose << panda_ee_frame.p.x(), panda_ee_frame.p.y(), panda_ee_frame.p.z();

        pandaState = stateA*pandaState + stateB*optimalSolution.segment(0,ndof);
        pandaArm.setState(pandaState.head(ndof),pandaState.tail(ndof));
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
        myfile<<pandaState.tail(7).transpose()<<'\n' ;
        myfile2<<optimalSolution.transpose() << '\n';
        myfile2.close();
        ite ++ ;
//        break;
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout<<"panda joint final is : \n" << pandaArm.getRobotState().segment(0,ndof) << std::endl;
    std::cout<<"panda ee position is :\n " << pandaArm.getSegmentPosition(8).p << std::endl;
    std::cout<<"Calcul donné par KDL est :\n " << panda_q_des.data << std::endl;
    return 0;
}

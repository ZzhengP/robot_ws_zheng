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

using namespace  std;
using namespace qpOASES;
using namespace  plane;



const double pi = 3.1415927;


#define MAXBUFSIZE  ((int) 1e6)

Eigen::MatrixXd readMatrix(const char *filename)
    {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    ifstream infile;
    infile.open(filename);
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        }

    infile.close();

    rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    return result;
    };



    
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



Eigen::MatrixXd msgToEigen(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  Eigen::MatrixXd eigenMatrix;
  int rows, cols, rows_stride, cols_stride ;

  rows = msg->layout.dim[0].size;
  cols = msg->layout.dim[1].size;

  rows_stride = msg->layout.dim[0].stride;
  cols_stride = msg->layout.dim[1].stride;

  eigenMatrix.resize(rows,cols);
  eigenMatrix.setZero();
  int ii =0;

 for (int i = 0; i < rows; ++i){
      for (int j = 0; j < cols; ++j){
          eigenMatrix(i,j) = msg->data[ii++];
      }
 }
	
  
  return eigenMatrix;
}

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
    const int N= 5 , ndof = 7;
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

    // initialize robot's kinematic class
    arm_kinematic pandaArm(&n,panda_urdf, ndof,"panda_link0", "panda_link8");

    pandaArm.init(panda_q_init.data, panda_dotq_init.data, N);

    // panda's end-effector location and set up desired frame
    KDL::Frame des_frame, back_des_frame, panda_ee_frame, panda_des_frame, panda_obs_frame;
    KDL::Frame forearm, wrist, base_link;
    panda_obs_frame = pandaArm.getSegmentPosition(7);
    panda_ee_frame = pandaArm.getSegmentPosition(8);
    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    panda_obs_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));
    forearm = pandaArm.getSegmentPosition(3);
    wrist = pandaArm.getSegmentPosition(6);
    base_link = pandaArm.getSegmentPosition(1);


//    panda_des_frame.p[0] = 0.470808;
//    panda_des_frame.p[1] = -0.5 ;
//    panda_des_frame.p[2] = 0.517842 ;
    panda_des_frame.p[0] = 0.7;
    panda_des_frame.p[1] = -0.2 ;
    panda_des_frame.p[2] = 0.1 ;
//    panda_des_frame.p[0] = panda_ee_frame.p[0] ;
//    panda_des_frame.p[1] = panda_ee_frame.p[1] ;
//    panda_des_frame.p[2] = panda_ee_frame.p[2] ;
    panda_des_frame.M = panda_des_frame.M;
    panda_des_frame.M.DoRotX(- pi);
    pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);


    // define an cube shape as an obstacle
    rviz_visual_tools::RvizVisualToolsPtr cubeObstacleMarkers, planeSeparation, goalPublisher;
    rviz_visual_tools::colors color = rviz_visual_tools::RED;
    rviz_visual_tools::colors color2 = rviz_visual_tools::BLACK;
    rviz_visual_tools::colors colorPlane1 = rviz_visual_tools::TRANSLUCENT_DARK;
    rviz_visual_tools::colors colorPlane2 = rviz_visual_tools::RED;
    rviz_visual_tools::colors colorPlane3 = rviz_visual_tools::CYAN;
    rviz_visual_tools::colors colorPlane4 = rviz_visual_tools::TRANSLUCENT_LIGHT;
    rviz_visual_tools::colors colorGoal = rviz_visual_tools::GREEN;
    Eigen::Vector3d Goal1, Goal2 ;
    Goal1 <<  panda_des_frame.p[0], panda_des_frame.p[1], panda_des_frame.p[2];


    Goal2 <<  panda_ee_frame.p.x(), panda_ee_frame.p.y(), panda_ee_frame.p.z();

   

    // ------------------------------------   Initialize MPC  ------------------------------------------
    Eigen::VectorXd optimalSolution;

    // Define MPC task
    Eigen::VectorXd pandaState, q_horizon, dotq_horizon;
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
    pandaArm.setMPCState(pandaTask.getMatrixPx(), pandaTask.getMatrixPu(),
                         pandaTask.getMatrixPxdq(), pandaTask.getMatrixPudq());

    q_horizon.resize(ndof*N);
    dotq_horizon.resize(ndof*N);
    dotq_horizon.setZero();
    Eigen::MatrixXd jacobianHorizon, jacobianDotHorizon;
    jacobianHorizon.resize(N*6, N*ndof);
    jacobianDotHorizon.resize(N*6,N*ndof);
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

    dotq_horizon = pandaPxdq*pandaState + pandaPudq*optimalSolution;
    pandaArm.setJointVelHorizon(dotq_horizon);

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
    dqMin.setConstant(-pi/10);
    dqMax.setConstant(pi/10);
    dqMin.tail(ndof).setZero();
    dqMax.tail(ndof).setZero();
    JntPosCst jointPosCst(ndof,N,dt, "jointPosCst",pandaPx,pandaPu);
    jointPosCst.setLimit(qMin,qMax);
    jointPosCst.setLowerBound(pandaState);
    jointPosCst.setUpperBound(pandaState);
    jointPosCst.setConstraintMatrix();
    jointPosCst.getConstraintData().print();
    constraintVectorData.push_back(jointPosCst.getConstraintData());

    JntVelCst jointVelCst(ndof,N,dt, "jointVelCst",pandaPxdq,pandaPudq);
    jointVelCst.setLimit(dqMin,dqMax);
    jointVelCst.setLowerBound(pandaState);
    jointVelCst.setUpperBound(pandaState);
    jointVelCst.setConstraintMatrix();
    jointVelCst.getConstraintData().print();
    constraintVectorData.push_back(jointVelCst.getConstraintData());

    // Define joint acceleration limit
    JntAccCst jnt_acc_cst(ndof, N,dt, "jointAccCst",pandaPx,pandaPu);
    jnt_acc_cst.setLimit(ddqMin,ddqMax);

    Eigen::VectorXd cartVelMin, cartVelMax, dq ;
    cartVelMin.resize(3*N);
    cartVelMax.resize(3*N);

    for (int k(0); k <N; k ++){
        cartVelMin.segment(3*k,3) << -0.6, -0.6, -0.6;
        cartVelMax.segment(3*k,3) << 0.6, 0.6, 0.6 ;
    }

    dq.resize(ndof);
    dotq_horizon.resize(ndof*N);
    q_horizon.resize(ndof*N);
    Eigen::MatrixXd Jacobian, C;
    dotq_horizon = pandaPxdq*pandaState + pandaPudq*optimalSolution;
    pandaArm.setJointVelHorizon(dotq_horizon);
    pandaArm.computeJacobian();
    dotq_horizon = pandaArm.getdqEnlarged();
    Jacobian.resize(6,7);
    Jacobian = pandaArm.getJacobian().data;
    dq = pandaArm.getJointVel().data;

    
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
//    int constraintSize = 2*ndof*N + obsAvoidanceCst.getConstraintData().upBound_.size()
//                        + TableAvoidanceCst.getConstraintData().upBound_.size() + + cartesianVelCst.getConstraintData().upBound_.size();
    int constraintSize = 2*ndof*N;
    mpc_solve qptest(1,ndof*N,constraintSize);
    qptest.initMPCData(lb,ub);
    qptest.constructProblem(constraintVectorData,H,g);

    qptest.setDefaultOptions();

    Eigen::Vector3d cartesian_error;
    bool task_complete ;
    task_complete = false;
    Eigen::Vector3d currentEEpose, goal, goalStart, goalEnd ;

//    goalEnd << 0.470808, -0.3, 0.517842;
    goalEnd << 0.7, -0.2, 0.1 ;
//    goalStart << panda_ee_frame.p[0], panda_ee_frame.p[1], panda_ee_frame.p[2];
    goalStart << 0.5, 0.3, 0.1 ;

    goal = goalEnd;
    // ------------------------------------   Initialize Ros connextion  ------------------------------------------
    Eigen::VectorXd jointTemp;
    jointTemp.resize(ndof);
    ros::Rate loop_rate(100);

    // create ros topic for sending msg
    ros::Publisher panda_joint_state_1_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint1_position_controller/command", 1000);
    ros::Publisher panda_joint_state_2_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint2_position_controller/command", 1000);
    ros::Publisher panda_joint_state_3_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint3_position_controller/command", 1000);
    ros::Publisher panda_joint_state_4_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint4_position_controller/command", 1000);
    ros::Publisher panda_joint_state_5_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint5_position_controller/command", 1000);
    ros::Publisher panda_joint_state_6_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint6_position_controller/command", 1000);
    ros::Publisher panda_joint_state_7_pub = n.advertise<std_msgs::Float64>("/panda/panda_joint7_position_controller/command", 1000);




    Eigen::VectorXd eePosHorz ;
    eePosHorz.resize(3*N);

    Eigen::Vector3d eeVel, eeAngleVel;
    eeAngleVel.setZero();
    eeVel.setZero();
  
    

    double simtime = 0;
    int count = 0;

     while(ros::ok())
    {

        currentEEpose << panda_ee_frame.p[0], panda_ee_frame.p[1], panda_ee_frame.p[2];
        cartesian_error = currentEEpose - goal;

        if (cartesian_error.norm() < 0.0001){

            if (goal == goalStart){
                goal = goalEnd;
            }else {
                goal = goalStart;
            }

        }

        panda_des_frame.p[0]=  goal(0);
        panda_des_frame.p[1]=  goal(1);
        panda_des_frame.p[2]=  goal(2);
        pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);
        for (size_t i=0; i<N; i++)
        {
             qHorizonDes.segment(ndof*i,ndof) = panda_q_des.data;
        }
//        pandaArm.computeJntFromCart(panda_des_frame,panda_q_des);

        pandaTask.computeHandg(jacobianHorizon,pandaState,qHorizonDes);
        H = pandaTask.getMatrixH();
        g = pandaTask.getVectorg();

        pandaArm.computeJacobian();
        Jacobian = pandaArm.getJacobian().data;
        dq = pandaArm.getJointVel().data;

        pandaArm.computeJacobianHorz(q_horizon);
        jacobianHorizon = pandaArm.getJacobianHorz();

        pandaArm.setqqdhorizon(q_horizon,dotq_horizon);
        jacobianDotHorizon = pandaArm.getJacobianDotHorz();

        // Update joint position constraint
        jointPosCst.update(pandaState);
        // Update vel position constraint
        jointVelCst.update(pandaState);
     

        constraintVectorData[0] = jointPosCst.getConstraintData();
        constraintVectorData[1] = jointVelCst.getConstraintData();

        qptest.constructProblem(constraintVectorData,H,g);
        bool is_solved;

        auto start = std::chrono::system_clock::now();
        is_solved = qptest.solve();
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        if (!is_solved){
            std::cout <<" qp failed :\n" << std::endl;
            optimalSolution.setZero();
            std::cout<<"optimal solution \n" <<optimalSolution<< std::endl;
            break;

        }else {
                optimalSolution = qptest.getSolution();
}

        // compute q enlarged
        q_horizon = pandaPx*pandaState + pandaPu*optimalSolution;
        pandaArm.setJointHorizon(q_horizon);

        // compute qdot enlarged
        dotq_horizon = pandaPxdq * pandaState + pandaPudq * optimalSolution;
        pandaArm.setJointVelHorizon(dotq_horizon);



        pandaArm.computeJntPosHorz(optimalSolution);
        pandaArm.computeCartPosHorz();
        pandaArm.verticesPublisher();


        Eigen::Vector3d ee_pose ;
        ee_pose << panda_ee_frame.p.x(), panda_ee_frame.p.y(), panda_ee_frame.p.z();
        pandaState = stateA*pandaState + stateB*optimalSolution.segment(0,ndof);
        pandaArm.setState(pandaState.head(ndof),pandaState.tail(ndof));
        panda_ee_frame = pandaArm.getSegmentPosition(8);

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

        simtime = simtime + dt;
//        break;
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout<<"panda joint final is : \n" << pandaArm.getRobotState().segment(0,ndof) << std::endl;
    std::cout<<"panda ee position is :\n " << panda_ee_frame.p << std::endl;
    std::cout<<"Calcul donné par KDL est :\n " << panda_q_des.data << std::endl;
    return 0;
}

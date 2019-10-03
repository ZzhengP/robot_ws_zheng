#include "robotmodel/robotmodel.h"
#include "solver/mpc_solver.h"
#include "task/mpc_task.h"
#include "constraint/generic_cst.h"
#include "constraint/joint_acc_cst.h"


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
#include "sensor_msgs/JointState.h"
#include <sstream>
#include "cmath"
#include "qpOASES.hpp"
using namespace  std;
using namespace qpOASES;

const double pi = 3.1415927;

sensor_msgs::JointState jnt_state;

void callback(const sensor_msgs::JointState& msg )
{

    std::cout <<" good ? \n " << std::endl;
    for (size_t i(0); i<msg.position.size() ; i ++){
        jnt_state.name[i] = msg.name[i];
        jnt_state.position[i] = msg.position[i];
        jnt_state.velocity[i] = msg.velocity[i];
    }

}


int main(int argc, char **argv)
{

    double dt(0.05);
    const std::string& urdf_name= "/home/zheng/robot_ws_zheng/src/ur_description/urdf/ur5_robot.urdf";
    const std::string& panda_urdf = "/home/zheng/catkin_ws/src/franka_ros/franka_description/robots/panda_arm.urdf";

    const int ndof=6, N=2, panda_ndof = 7;
    arm_kinematic robot_arm(urdf_name, ndof,"base_link", "wrist_3_link");
    arm_kinematic panda_arm(panda_urdf, panda_ndof,"panda_link0", "panda_link7");

    KDL::JntArray q_init(ndof), dotq_init(ndof), q_des(ndof), q_des_back(ndof), panda_q_des(panda_ndof);
    KDL::JntArray panda_q_init(panda_ndof), panda_dotq_init(panda_ndof), panda_q_des_back(panda_ndof);

    q_init.data << 0, -1.57, 1.57, -1.57, -1.57, 0 ;
    dotq_init.data << 0, 0, 0, 0, 0, 0;

    panda_q_init.data << 1.57, 0, 0, -1.8, 0, 1.57, 0;
    panda_dotq_init.data << 0, 0, 0, 0, 0, 0, 0;

    robot_arm.init(q_init.data, dotq_init.data, N);

    panda_arm.init(panda_q_init.data, panda_dotq_init.data, N);

    KDL::Frame ee_frame, des_frame, back_des_frame, panda_ee_frame, panda_des_frame;
    ee_frame = robot_arm.getSegmentPosition(5);
    panda_ee_frame = panda_arm.getSegmentPosition("panda_link7");

    Eigen::Vector3d ZYX_angle, panda_ZYX_angle;
    ee_frame.M.GetRPY(ZYX_angle(0),ZYX_angle(1),ZYX_angle(2));
    panda_ee_frame.M.GetRPY(panda_ZYX_angle(0),panda_ZYX_angle(1),panda_ZYX_angle(2));

    des_frame.p[0] = 0. ;
    des_frame.p[1] = -0.5;
    des_frame.p[2] = 0.5 ;
    des_frame.M = ee_frame.M ;
    panda_des_frame.p[0] = 0.5 ;
    panda_des_frame.p[1] = 0. ;
    panda_des_frame.p[2] = 0.5 ;
    panda_des_frame.M = panda_des_frame.M;
    panda_des_frame.M.DoRotX(- pi);
    robot_arm.computeJntFromCart(des_frame,q_des);
    panda_arm.computeJntFromCart(panda_des_frame,panda_q_des);

    // Define MPC task
    Eigen::VectorXd optimal_Solution, robot_state, q_horizon;
    Eigen::VectorXd panda_optimal_Solution, panda_robot_state, panda_q_horizon;

    bool ok;
    bool panda_ok;
    MPC_Task task(N,1,0.001,ndof,dt, "ur5");
    ok = task.init();

    MPC_Task panda_task(N,1,0.001,panda_ndof,dt, "panda");
    panda_ok = panda_task.init();

    Eigen::MatrixXd Px, Pu, Px_dq, Pu_dq;
    Eigen::MatrixXd panda_Px, panda_Pu, panda_Px_dq, panda_Pu_dq;

    Px.resize(N*ndof,2*ndof);
    Pu.resize(N*ndof,N*ndof);
    Px = task.getMatrixPx();
    Pu = task.getMatrixPu();
    panda_Px.resize(N*panda_ndof,2*panda_ndof);
    panda_Pu.resize(N*panda_ndof,N*panda_ndof);
    panda_Px = panda_task.getMatrixPx();
    panda_Pu = panda_task.getMatrixPu();

    Px_dq.resize(N*ndof,2*ndof);
    Pu_dq.resize(N*ndof,N*ndof);
    Px_dq = task.getMatrixPxdq();
    Pu_dq = task.getMatrixPudq();
    panda_Px_dq.resize(N*panda_ndof,2*panda_ndof);
    panda_Pu_dq.resize(N*panda_ndof,N*panda_ndof);
    panda_Px_dq = panda_task.getMatrixPxdq();
    panda_Pu_dq = panda_task.getMatrixPudq();

    robot_state.resize(2*ndof);
    robot_state = robot_arm.getRobotState();
    panda_robot_state.resize(2*panda_ndof);
    panda_robot_state = panda_arm.getRobotState();

    optimal_Solution.resize(ndof*N);
    optimal_Solution.setZero();
    panda_optimal_Solution.resize(panda_ndof*N);
    panda_optimal_Solution.setZero();

    q_horizon.resize(ndof*N);
    panda_q_horizon.resize( panda_ndof*N);

    //    std::cout<<"La solution optimale est \n"<< optimal_Solution << std::endl;


    Eigen::MatrixXd J_horizon, panda_J_horizon;
    J_horizon.resize(N*6, N*ndof);
    panda_J_horizon.resize(N*6, N*panda_ndof);

    Eigen::VectorXd q_horizon_des, panda_q_horizon_des;
    q_horizon_des.resize(ndof*N);
    panda_q_horizon_des.resize(panda_ndof*N);

    for (size_t i=0; i<N; i++)
    {
         q_horizon_des.segment(ndof*i,ndof) = q_des.data;
         panda_q_horizon_des.segment(panda_ndof*i,panda_ndof) = panda_q_des.data;
    }

    Eigen::MatrixXd H, panda_H;
    H.resize(N*ndof,N*ndof);
    panda_H.resize(N*panda_ndof,N*panda_ndof);

    Eigen::VectorXd g, lb, ub, panda_g, panda_lb, panda_ub;
    g.resize(N*ndof);
    lb.resize(N*ndof);
    ub.resize(N*ndof);
    panda_g.resize(N*panda_ndof);
    panda_lb.resize(N*panda_ndof);
    panda_ub.resize(N*panda_ndof);

    Eigen::MatrixXd A, B, panda_A, panda_B;
    A = task.getStateA();
    B = task.getStateB();
    panda_A = panda_task.getStateA();
    panda_B = panda_task.getStateB();


    mpc_solve qpSolver(N, ndof), panda_qpSolver(N,panda_ndof);
    qpSolver.setDefaultOptions();
    qpSolver.initData(H,g,lb,ub);
    panda_qpSolver.setDefaultOptions();
    panda_qpSolver.initData(panda_H,panda_g,panda_lb,panda_ub);

    // Etablir la communication avec ROS
    ros::init(argc,argv,"cart_controller");
    ros::NodeHandle n;
    // ur5
    ros::Publisher joint_state_1_pub = n.advertise<std_msgs::Float64>("/ur5/shoulder_pan_joint_position_controller/command", 1000);
    ros::Publisher joint_state_2_pub = n.advertise<std_msgs::Float64>("/ur5/shoulder_lift_joint_position_controller/command", 1000);
    ros::Publisher joint_state_3_pub = n.advertise<std_msgs::Float64>("/ur5/elbow_joint_position_controller/command", 1000);
    ros::Publisher joint_state_4_pub = n.advertise<std_msgs::Float64>("/ur5/wrist_1_joint_position_controller/command", 1000);
    ros::Publisher joint_state_5_pub = n.advertise<std_msgs::Float64>("/ur5/wrist_2_joint_position_controller/command", 1000);
    ros::Publisher joint_state_6_pub = n.advertise<std_msgs::Float64>("/ur5/wrist_3_joint_position_controller/command", 1000);

    // Franka Emika panda
    ros::Publisher panda_joint_state_1_pub = n.advertise<std_msgs::Float64>("/ur5/panda_joint1_position_controller/command", 1000);
    ros::Publisher panda_joint_state_2_pub = n.advertise<std_msgs::Float64>("/ur5/panda_joint2_position_controller/command", 1000);
    ros::Publisher panda_joint_state_3_pub = n.advertise<std_msgs::Float64>("/ur5/panda_joint3_position_controller/command", 1000);
    ros::Publisher panda_joint_state_4_pub = n.advertise<std_msgs::Float64>("/ur5/panda_joint4_position_controller/command", 1000);
    ros::Publisher panda_joint_state_5_pub = n.advertise<std_msgs::Float64>("/ur5/panda_joint5_position_controller/command", 1000);
    ros::Publisher panda_joint_state_6_pub = n.advertise<std_msgs::Float64>("/ur5/panda_joint6_position_controller/command", 1000);
    ros::Publisher panda_joint_state_7_pub = n.advertise<std_msgs::Float64>("/ur5/panda_joint7_position_controller/command", 1000);


    ros::Publisher joint_1_vel = n.advertise<std_msgs::Float64>("/ur5/shoulder_pan_joint_position_controller/vel", 1000);
    ros::Publisher joint_2_vel = n.advertise<std_msgs::Float64>("/ur5/shoulder_lift_joint_position_controller/vel", 1000);
    ros::Publisher joint_3_vel = n.advertise<std_msgs::Float64>("/ur5/elbow_joint_position_controller/vel", 1000);
    ros::Publisher joint_4_vel = n.advertise<std_msgs::Float64>("/ur5/wrist_1_joint_position_controller/vel", 1000);
    ros::Publisher joint_5_vel = n.advertise<std_msgs::Float64>("/ur5/wrist_2_joint_position_controller/vel", 1000);
    ros::Publisher joint_6_vel = n.advertise<std_msgs::Float64>("/ur5/wrist_3_joint_position_controller/vel", 1000);
    ros::Rate loop_rate(10);

    ros::Subscriber sub;

    robot_state = robot_arm.getRobotState();
    panda_robot_state = panda_arm.getRobotState();

    robot_arm.computeqEnlarged(optimal_Solution,Px,Pu);
    panda_arm.computeqEnlarged(panda_optimal_Solution,panda_Px,panda_Pu);

    q_horizon = Px*robot_state + Pu*optimal_Solution;
    panda_q_horizon = panda_Px*panda_robot_state + panda_Pu*panda_optimal_Solution;


    // Define constraint

    Eigen::VectorXd ddq_min, ddq_max, ddq_lb, ddq_ub;
    Eigen::MatrixXd ddq_C;
    ddq_min.resize(N*ndof);
    ddq_max.resize(N*ndof);
    ddq_lb.resize(N*ndof);
    ddq_ub.resize(N*ndof);
    ddq_C.resize(N*ndof, N*ndof);
    ddq_C.setIdentity();

    for (size_t i = 0; i < N ; i++){
        ddq_min.segment(6*i,6) << -2, -2, -2, -2, -2, -2;
        ddq_max.segment(6*i,6) << 2, 2, 2, 2, 2, 2;
      }
    joint_acc_cst jnt_acc_cst(ndof, N);
    jnt_acc_cst.setLimit(ddq_min,ddq_max);
    jnt_acc_cst.setLowerBound(robot_state, Px);
    jnt_acc_cst.setUpperBound(robot_state, Px);
    ddq_lb = jnt_acc_cst.getLowerBound();
    ddq_ub = jnt_acc_cst.getUpperBound();
    double error = 100;

//    while(ros::ok())
         for (int i(0);i<1;i++)
          {        
              robot_state = robot_arm.getRobotState();
              panda_robot_state = panda_arm.getRobotState();

              robot_arm.computeqEnlarged(optimal_Solution,Px,Pu);
              panda_arm.computeqEnlarged(panda_optimal_Solution,panda_Px,panda_Pu);

      //       std::cout <<"q horizon : \n" << q_horizon << std::endl;
      //        q_horizon = Px*robot_state + Pu*optimal_Solution;
              q_horizon = robot_arm.getqEnlarged();
              panda_q_horizon = panda_Px*panda_robot_state + panda_Pu*panda_optimal_Solution;

              robot_arm.computeJacobianHorz(q_horizon);
              panda_arm.computeJacobianHorz(panda_q_horizon);

              J_horizon = robot_arm.getJacobianHorz();
              panda_J_horizon = panda_arm.getJacobianHorz();

     //        std::cout <<"J_horizon : \n" << J_horizon << std::endl;

//              robot_arm.computeCartPosHorz(q_horizon);
//              panda_arm.computeCartPosHorz(panda_q_horizon);


      //        std::cout <<"q_horizon : \n" << q_horizon << std::endl;
      //        return 0;
              task.computeHandg(J_horizon,robot_state,q_horizon_des);
              panda_task.computeHandg(panda_J_horizon,panda_robot_state,panda_q_horizon_des);
              H = task.getMatrixH();
              panda_H = panda_task.getMatrixH();

              g = task.getVectorg();
              panda_g = panda_task.getVectorg();

              panda_lb.setConstant(-10);
              panda_ub.setConstant(10);

      //        qpSolver.initData(H.block(0,6,6,6),g.segment(6,6),lb.segment(0,6),ub.segment(0,6));

      //        qpSolver.solve(H,g,cst_C_dq,lb,ub,dq_lower,dq_upper);
              qpSolver.solve(H,g,ddq_lb,ddq_ub);
              panda_qpSolver.solve(panda_H,panda_g,panda_lb,panda_ub);

              optimal_Solution = qpSolver.getSolution();
              panda_optimal_Solution = panda_qpSolver.getSolution();

      //        optimal_Solution.segment(0,6) = qpSolver.getSolution();
//              std::cout<<"La solution optimale est : \n" << optimal_Solution << std::endl;
              robot_state = A*robot_state + B*optimal_Solution.segment(0,ndof);
              panda_robot_state = panda_A*panda_robot_state + panda_B*panda_optimal_Solution.segment(0,panda_ndof);

              std_msgs::Float64 t1,t2,t3,t4,t5,t6 ; // robot joint's positions
              std_msgs::Float64 v1,v2,v3,v4,v5,v6 ; // robot joint's velocity
              std_msgs::Float64 panda_t1,panda_t2,panda_t3,panda_t4,panda_t5,panda_t6, panda_t7 ; // robot joint's positions
              t1.data = robot_state[0];
              t2.data = robot_state[1];
              t3.data = robot_state[2];
              t4.data = robot_state[3];
              t5.data = robot_state[4];
              t6.data = robot_state[5];
              panda_t1.data = panda_robot_state[0];
              panda_t2.data = panda_robot_state[1];
              panda_t3.data = panda_robot_state[2];
              panda_t4.data = panda_robot_state[3];
              panda_t5.data = panda_robot_state[4];
              panda_t6.data = panda_robot_state[5];
              panda_t7.data = panda_robot_state[6];

              v1.data = robot_state[6];
              v2.data = robot_state[7];
              v3.data = robot_state[8];
              v4.data = robot_state[9];
              v5.data = robot_state[10];
              v6.data = robot_state[11];

              joint_state_1_pub.publish(t1);
              joint_state_2_pub.publish(t2);
              joint_state_3_pub.publish(t3);
              joint_state_4_pub.publish(t4);
              joint_state_5_pub.publish(t5);
              joint_state_6_pub.publish(t6);

              panda_joint_state_1_pub.publish(panda_t1);
              panda_joint_state_2_pub.publish(panda_t2);
              panda_joint_state_3_pub.publish(panda_t3);
              panda_joint_state_4_pub.publish(panda_t4);
              panda_joint_state_5_pub.publish(panda_t5);
              panda_joint_state_6_pub.publish(panda_t6);
              panda_joint_state_7_pub.publish(panda_t7);

              joint_1_vel.publish(v1);
              joint_2_vel.publish(v2);
              joint_3_vel.publish(v3);
              joint_4_vel.publish(v4);
              joint_5_vel.publish(v5);
              joint_6_vel.publish(v6);

      //        q_init.data = q_des.data;
              sub = n.subscribe("/ur5/joint_states",1000,&callback);
              for (size_t j(0) ; j < jnt_state.position.size(); j++ )
              {
                 std::cout << jnt_state.name[j] << std::endl;
              }
              robot_arm.setState(robot_state.head(ndof),robot_state.tail(ndof));
              panda_arm.setState(panda_robot_state.head(panda_ndof),panda_robot_state.tail(panda_ndof));
              ee_frame = robot_arm.getSegmentPosition(5);
              error = pow(des_frame.p[0]-ee_frame.p[0],2) + pow(des_frame.p[1]-ee_frame.p[1],2) + pow(des_frame.p[2]-ee_frame.p[2],2) ;
              std::cout << FRED("error of tracking :") << sqrt(error) << std::endl;
              if (sqrt(error) < 0.0001)
              {
                  break;
              }
          }
          ros::spinOnce();
          loop_rate.sleep();
          std::cout<<"Robot joint ini is: \n " << q_init.data << std::endl;
          std::cout<<"Robot joint final is : \n" << robot_arm.getRobotState().segment(0,ndof) << std::endl;
          std::cout<<"robot ee position is :\n " << robot_arm.getSegmentPosition(5).p << std::endl;
          std::cout<<"Calcul donné par KDL est :\n " << q_des.data << std::endl;

          std::cout<<"panda joint final is : \n" << panda_arm.getRobotState().segment(0,ndof) << std::endl;
          std::cout<<"panda ee position is :\n " << panda_arm.getSegmentPosition("panda_link7").p << std::endl;
          std::cout<<"Calcul donné par KDL est :\n " << panda_q_des.data << std::endl;
          return 0;
      }



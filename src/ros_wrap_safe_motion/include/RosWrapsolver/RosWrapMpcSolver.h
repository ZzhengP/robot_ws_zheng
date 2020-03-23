#ifndef RosWrapMpcSolver_H
#define RosWrapMpcSolver_H

#pragma once
#include "qpOASES.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "RosWrapcommun/color.h"
#include "RosWrapconstraint/RosWrapGenericCst.h"
#include "vector"
#include "fstream"
#include "ros/ros.h"
#include "RosWraptask/RosWrapMpcTask.h"
#include "RosWraprobotmodel/RosWrapRobotModel.h"
#include "RosWrapconstraint/RosWrapJointAccCst.h"
#include "RosWrapconstraint/RosWrapJointVelCst.h"
#include "RosWrapconstraint/RosWrapJointPosCst.h"
#include "RosWrapconstraint/RosWrapAvoidanceCst.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/InteractiveMarkerUpdate.h"

using namespace plane;

class mpc_solve
{
public:
    mpc_solve(int N_horz, int Ndof, int nbrCst);

    /**
     * @brief Ros node  versions  
     */
    mpc_solve(ros::NodeHandle* nodehandle, int N_horz, int Ndof, int nbrCst, double dt);
    ~mpc_solve(){}


    bool initMPCData(Eigen::VectorXd lb, Eigen::VectorXd ub);

    void initTaskData();
    
    void initArmData(const std::string& panda_urdf, const KDL::JntArray& panda_q_init, const KDL::JntArray& panda_dq_init);

    void initRobotConstraintData(const Eigen::VectorXd &ddq_min, const Eigen::VectorXd &ddq_max,
                                 const Eigen::VectorXd &dq_min, const Eigen::VectorXd &dq_max, 
                                 const Eigen::VectorXd &q_min, const Eigen::VectorXd &q_max,
                                 const Eigen::VectorXd &v_min, const Eigen::VectorXd &v_max);



    void update();

    /**
     * @brief  After having initiliaze all paremeter, let's simulate 
     * 
     */
    void run();


    bool solve(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub);


    bool solve();

    void constructProblem(const std::vector<constraintData> &constraintVectorData,const Eigen::MatrixXd H, const Eigen::VectorXd g);

    void constructProblem();

    void verticesPublisher();

    void planeDataCallback(const std_msgs::Float64MultiArray::ConstPtr& plane_msg);

    void iMarkerCallback(const geometry_msgs::Pose::ConstPtr& imarker_msg);

   Eigen::VectorXd getSolution()
   {
        return optimal_solution_;
   }

    void setDefaultOptions()
    {
        if(!qpoases_)
            throw std::runtime_error("qpsolver pointer is null");

        qpoases_->setOptions( options_ );
        qpoases_->setPrintLevel(qpOASES::PL_HIGH);

        qpoases_standard_->setOptions( options_ );
        qpoases_standard_->setPrintLevel(qpOASES::PL_HIGH);
    }

    void print(){

        std::cout << "H_ \n " <<  H_ << '\n';
        std::cout << "g_ \n " << g_ << '\n';
        std::cout << "A_ \n " << A_ << '\n';
        std::cout << "lbA_ \n " << lbA_ << '\n';
        std::cout << "ubA_ \n " << ubA_ << '\n';
        std::cout << "lb_ \n " << lb_ << '\n';
        std::cout << "ub_ \n " << ub_ << '\n';
    }

private:

    ros::NodeHandle nh_;
    // create ros topic for sending msg
    ros::Publisher panda_joint_state_1_pub_ ;
    ros::Publisher panda_joint_state_2_pub_ ; 
    ros::Publisher panda_joint_state_3_pub_;
    ros::Publisher panda_joint_state_4_pub_;
    ros::Publisher panda_joint_state_5_pub_;
    ros::Publisher panda_joint_state_6_pub_;
    ros::Publisher panda_joint_state_7_pub_;
    ros::Subscriber plane_data_subscriber_;
    ros::Subscriber interactive_marker_subscriber_;

    std::shared_ptr<qpOASES::QProblemB> qpoases_;
    std::shared_ptr<qpOASES::SQProblem> qpoases_standard_;
    qpOASES::Options options_;
    int N_, Ndof_;
    double dt_;
    // Need RowMajor
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    Eigen::VectorXd g_, lb_, ub_, lbA_, ubA_, optimal_solution_;


    int nV_, nC_ ;


    // ----------------------------  Ros + Task parameter ----------------------------------
    std::shared_ptr<MPC_Task> mpc_task_ ;
    Eigen::MatrixXd panda_px_, panda_pu_, panda_pxdq_, panda_pudq_;
    Eigen::Vector3d currentEEpose_, goal_, goalStart_, goalEnd_ ;
    Eigen::MatrixXd  state_A_, state_B_;
    geometry_msgs::Pose interactive_marker_pos_;    
    // ---------------------------- Robot Arm Dynamic parameter ----------------------------

    std::shared_ptr<arm_kinematic> panda_arm_;
    std::string panda_urdf_;
    KDL::JntArray panda_q_ini_,  panda_dq_init_, panda_q_des_;
    KDL::Frame panda_des_frame_, panda_ee_frame_;

    Eigen::VectorXd panda_state_;
    Eigen::VectorXd q_, dq_ ;
    Eigen::VectorXd q_horizon_, dq_horizon_, q_horizon_des_,  ee_pos_horizon_ ;

    Eigen::MatrixXd jacobian_, jacobian_horizon_;

    Eigen::VectorXd ddq_min_, ddq_max_, dq_min_, dq_max_, q_min_, q_max_, v_max_, v_min_;

 

    // ------------------------- Parameter for obstacle avoidance -----------------------------
    std::vector<Eigen::MatrixXd> robot_vertices_augmented_;
    Planes::PlaneData table_planes_;

    std::shared_ptr<JntPosCst> jnt_pos_cst_ ;
    std::shared_ptr<JntVelCst> jnt_vel_cst_ ;
    std::shared_ptr<JntAccCst> jnt_acc_cst_ ;
    std::shared_ptr<ObsAvoidanceCSt> table_avoidance_cst_;
    std::shared_ptr<ObsAvoidanceCSt> obs_avoidance_cst_;
    std::vector<constraintData> total_constraint_data_ ; 
    unsigned int constraint_number_ = 0 ;

    Planes::PlaneData plane_data_;
    Eigen::MatrixXd plane_data_eigen_;
    double dsafe_;
};

#endif
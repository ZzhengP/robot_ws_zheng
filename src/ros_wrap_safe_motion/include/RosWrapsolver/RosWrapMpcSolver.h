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
class mpc_solve
{
public:
    mpc_solve(int N_horz, int Ndof, int nbrCst);

    /**
     * @brief Ros node  versions  
     */
    mpc_solve(ros::NodeHandle* nodehandle, int N_horz, int Ndof, int nbrCst, double dt, const std::string& panda_urdf, const KDL::JntArray& panda_q_init, const KDL::JntArray& panda_dq_init);
    ~mpc_solve(){}


    bool initMPCData(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub);
    void initTaskData();
    void initArmData();
    void initRobotConstraintData();

    bool solve(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub);


    bool solve();

    void constructProblem(const std::vector<constraintData> &constraintVectorData,const Eigen::MatrixXd H, const Eigen::VectorXd g);



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
    ros::NodeHandle nh_;
    std::shared_ptr<MPC_Task> mpc_task_ ;
    Eigen::MatrixXd panda_px_, panda_pu_, panda_pxdq_, panda_pudq_;


    // ---------------------------- Robot Arm Dynamic parameter ----------------------------

    std::shared_ptr<arm_kinematic> panda_arm_;
    std::string panda_urdf_;
    KDL::JntArray panda_q_ini_,  panda_dq_init_, panda_q_des_;
    KDL::Frame panda_des_frame_;

    Eigen::VectorXd panda_state_;
    Eigen::VectorXd q_horizon_, dq_horizon_;
};

#endif

#ifndef RosWrapMpcTask_H_
#define RosWrapMpcTask_H_
#pragma once
#include "iostream"
#include "unsupported/Eigen/MatrixFunctions"
#include "RosWrapcommun/RosWrapCommunDefinition.h"
#include "RosWrapcommun/color.h"
#include "Eigen/Core"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"



/**
 * @brief The MPC_Task class define the cost (objectif) function for Model Predictive Control
 * @brief And the related enlarged state, such as Px, Pu, Pxdq, Pudq
 *
 * \f$ q_{1:N} = Px*q_{0} + Pu*U_{0:N-1}   \\
 *      dq_{1:N} = Pxdq*q_{0} + Pudq*U_{0:N-1} \f$
 */
class  MPC_Task
{
public:
    /**
     * @brief MPC_Task constructor
     * @param N : Horizon of prediction
     * @param Q : First Weight
     * @param R : Second Weight
     * @param ndof
     * @param dt : time interval
     * @param robot_name
     */
    MPC_Task(int N, double Q, double R, int ndof, double dt, std::string robot_name);


    /**
     * @brief initialize private parameters
     * @return
     */
    bool init();

    /**
     * @brief computePx: Augmented joint position state
     */
    void computePx();
    /**
     * @brief computePu: Augmented joint position state
     */
    void computePu();

    /**
     * @brief computePxDq: Augmented joint velocity state
     */
    void computePxDq();

    /**
     * @brief computePuDq: Augmented joint velocity state
     */
    void computePuDq();

    /**
     * @brief computeHandg: Joints desired positions are given
     * @param J_horizon
     * @param robot_state
     * @param q_horz_Des
     */
    void computeHandg(Eigen::MatrixXd J_horizon,Eigen::VectorXd robot_state,Eigen::VectorXd q_horz_Des);

    /**
     * @brief computeHandg: Cartesien desired positions are given
     * @param J_horizon
     * @param q_horizon
     * @param cartPos_horizon
     * @param robot_state
     * @param PosDes
     */
    void computeHandg(Eigen::MatrixXd J_horizon, Eigen::VectorXd q_horizon,Eigen::VectorXd cartPos_horizon, Eigen::VectorXd robot_state,Eigen::VectorXd PosDes);

    /**
     * @brief computeE associated to cartesien space : \f$ ||Ex + f||^{2}\f$
     * @param J_horizon
     */
    void computeE(Eigen::MatrixXd J_horizon );
    /**
     * @brief computeE associated to joint space : \f$ ||Ex + f||^{2}\f$
     */
    void computeE();

    /**
     * @brief computef ssociated to cartesien space : \f$ ||Ex + f||^{2}\f$
     * @param J_horizon
     * @param cartPos_horizon
     * @param q_horizon
     * @param robot_state
     * @param PosDes
     */
    void computef(Eigen::MatrixXd J_horizon,Eigen::VectorXd cartPos_horizon,Eigen::VectorXd q_horizon, Eigen::VectorXd robot_state, Eigen::VectorXd PosDes);

    /**
     * @brief computef  associated to joint space : \f$ ||Ex + f||^{2}\f$
     * @param robot_state
     * @param q_horz_Des
     */
    void computef(const Eigen::VectorXd& robot_state, const Eigen::VectorXd& q_horz_Des);

    void computeEdq(Eigen::MatrixXd J_horizon);
    void computefdq(Eigen::VectorXd robot_state,Eigen::MatrixXd J_horizon);

    void computeEdq();
    void computefdq(Eigen::VectorXd robot_state);


    /**
     * @brief Interactive markers position callback    
     * 
     */

    void iMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr & marker_array );

    /**
     * @brief getStateA
     * @return A_
     */

    Eigen::MatrixXd getStateA()
    {
        return A_;
    }

    /**
     * @brief getStateB
     * @return B_
     */
    Eigen::MatrixXd getStateB()
    {
        return B_;
    }

    /**
     * @brief getMatrixH
     * @return H_
     */
    Eigen::MatrixXd getMatrixH()
    {
        return Hessien_;
    }

    /**
     * @brief getVectorg
     * @return  g
     */
    Eigen::VectorXd getVectorg()
    {
        return gradient_;
    }

    /**
     * @brief getMatrixPx
     * @return  Px_
     */
    Eigen::MatrixXd getMatrixPx()
    {
        return Px_;
    }

    /**
     * @brief getMatrixPu
     * @return Pu_
     */
    Eigen::MatrixXd getMatrixPu()
    {
        return Pu_;
    }

    /**
     * @brief getMatrixPxdq
     * @return Pxdq_
     */
    Eigen::MatrixXd getMatrixPxdq()
    {
        return Px_dq_;
    }

    /**
     * @brief getMatrixPudq
     * @return Pudq_
     */
    Eigen::MatrixXd getMatrixPudq()
    {
        return Pu_dq_;
    }
private:


    /**
     * @brief Ros task node
     */
    ros::NodeHandle nh_;


    const int N_, ndof;
    double weightTask1_, weightTask2_;
    const double dt_;
    /**
     * @brief Matrix and and vector for quadratic optimal problem
     */
    Eigen::MatrixXd Hessien_, E_, E_dq_, Acc_reg_, Id_;

    Eigen::VectorXd gradient_, f_, f_dq_;

    /**
     * @brief The dynamic model for the robot
     */
    Eigen::MatrixXd A_, B_,C_q_, C_dq_;

    /**
     * @brief Matrix enlarged from the MPC problem
     */

    Eigen::MatrixXd Px_, Pu_, Px_dq_, Pu_dq_;

};
#endif
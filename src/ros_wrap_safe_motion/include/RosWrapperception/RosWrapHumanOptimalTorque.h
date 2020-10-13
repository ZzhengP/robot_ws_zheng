#ifndef ROSWRAPHUMANOPTIMALTORQUE_H
#define ROSWRAPHUMANOPTIMALTORQUE_H

#pragma once 
#include "qpOASES.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "RosWrapcommun/color.h"
#include "ros/ros.h"
#include "map"

enum optimal_objective{ hand_jerk = 0,
                              joint_jerk,
                              joint_ddq,   
                              torque_variation,
                              torque,
                              geodesis,
                              energy              
};


std::ostream& operator<<(std::ostream& os, optimal_objective c){

    switch(c)
    {
        case hand_jerk   : os << "hand_jerk";    break;
        case joint_jerk: os << "joint_jerk"; break;
        case joint_ddq : os << "joint_ddq";  break;
        case torque_variation  : os << "torque_variation";   break;
        case torque : os << "torque"; break;
        case geodesis: os << "geodesis"; break;
        case energy: os << "energy"; break;
        default    : os.setstate(std::ios_base::failbit);
    }
      return os;
}


class HumanTorque {

    public:

    /**
    * @brief Construct a new Human Torque object
    * 
    * @param dt interval of discretization 
    */
    HumanTorque(const double &dt);

    ~HumanTorque(){
    }

    inline void compute_H_g(Eigen::MatrixXd &H, Eigen::VectorXd &g,const Eigen::MatrixXd &E,const Eigen::VectorXd &f){
           H = E.transpose()*E ;
           g = E.transpose()*f ;
    };
    //-------------------------------------  Objective Compute ------------------------------------------
    /**
     * @brief Minimize hand jerk 
     * 
     * @param J (Jacobian)
     */
    void compute_hand_jerk();

    /**
     * @brief Minimize joint jerk
     * @param joint acceleration precedent
     */
    void compute_joint_jerk();

    /**
     * @brief Minimize joint acceleration 
     *          if no parameters, means data stored in map is updated 
     */
    void compute_joint_acceleration();

    /**
     * @brief Minimize the variation of the change of the torque 
     * 
     */
    void compute_torque_change();

    /**
     * @brief Minimize torque 
     * 
     */
    void compute_minimal_torque();

    /**
     * @brief Minize the geodesis distance 
     * 
     */
    void compute_geodesis();

    /**
     * @brief Minimize energy
     * 
     */
    void compute_energy();

    // --------------------------------------------------------------------------------------------------
    void update_dynamic_parameters(const Eigen::MatrixXd & J, 
                                   const Eigen::VectorXd & ddX_prec,
                                   const Eigen::VectorXd & ddq_prec,
                                   const Eigen::VectorXd & torque_prec,
                                   const Eigen::VectorXd & q_prec,
                                   const Eigen::VectorXd & dq_prec,
                                   const Eigen::MatrixXd & inertial_matrix,
                                   const Eigen::VectorXd & coriolis,
                                   const Eigen::VectorXd & gravity);

    void update_objective();

    void get_update_objective(optimal_objective c){
        
         std::cout <<" getting update objective: " << c << std::endl;
         switch(c)
        {
            case hand_jerk: return HumanTorque::compute_hand_jerk() ; break;
            case joint_jerk:return HumanTorque::compute_joint_jerk() ; break;
            case joint_ddq:return  HumanTorque::compute_joint_acceleration() ; break;
            case torque_variation: return HumanTorque::compute_torque_change(); break;
            case torque: return   HumanTorque::compute_minimal_torque() ; break;
            case geodesis:return HumanTorque::compute_geodesis() ; break;
            case energy:return HumanTorque::compute_energy() ; break;
            default    : "no objective updated ";
        }
    }

    void compute_total_objective();

    bool solve();
    private:

    int n_;
    double dt_;

    std::shared_ptr<qpOASES::QProblem> qp_human_torque_;
    qpOASES::Options options_;

    // ------------------------------- Dynamic Parameter------------------------------------------------
    Eigen::MatrixXd inertial_matrix_, M_inv_, J_;
    Eigen::VectorXd coriolis_, gravity_;
    Eigen::VectorXd optimal_torque_;
    // ------------------------------------------------------------------------------------------------
    Eigen::VectorXd ddX_prec_, ddq_prec_, torque_prec_, q_prec_, dq_prec_  ;
    Eigen::VectorXd Cref_, weight_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    Eigen::VectorXd g_, lb_, ub_, lbA_, ubA_; 
    std::map<optimal_objective, Eigen::MatrixXd> optimal_objective_H_;
    std::map<optimal_objective, Eigen::VectorXd> optimal_objective_g_;

};
#endif
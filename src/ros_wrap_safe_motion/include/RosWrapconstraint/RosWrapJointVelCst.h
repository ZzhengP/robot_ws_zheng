#ifndef RosWrapJointVelCst_H
#define RosWrapJointVelCst_H
#pragma once
#include "Eigen/Core"
#include "iostream"
#include "unsupported/Eigen/MatrixFunctions"
#include "Eigen/Dense"
#include "RosWrapconstraint/RosWrapGenericCst.h"

class  jntVelCst : public generic_cst {
public:
    jntVelCst(int ndof, int N, double dt, std::string name,Eigen::MatrixXd Px, Eigen::MatrixXd Pu)
        : generic_cst (ndof, N,dt, name, Px, Pu){
        cstData_.name_ = name;
        cstData_.upBound_.resize(ndof*N_);
        cstData_.lowBound_.resize(ndof*N_);
        cstData_.cstMatrix_.resize(ndof*N_,ndof*N_);
    }


    void setLimit(const Eigen::VectorXd &min,const Eigen::VectorXd &max);

    void setLowerBound(const Eigen::VectorXd &robot_state);
    void setUpperBound(const Eigen::VectorXd &robot_state);
    void setConstraintMatrix();
    void update(const Eigen::VectorXd &robot_state);

private:

};

#endif
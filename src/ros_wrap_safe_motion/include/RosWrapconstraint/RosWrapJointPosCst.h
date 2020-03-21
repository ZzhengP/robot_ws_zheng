#ifndef RosWrapJointPosCst_H
#define RosWrapJointPosCst_H
#pragma once
#include "RosWrapconstraint/RosWrapGenericCst.h"


/**
 * @brief Joint position constraint class
 */
class jntPosCst : public generic_cst
{
public:

    jntPosCst(int ndof, int n, double dt, std::string name,Eigen::MatrixXd Px, Eigen::MatrixXd Pu)
        : generic_cst (ndof, n,dt, name, Px, Pu) {
        cstData_.name_ = name;
        cstData_.upBound_.resize(ndof*N_);
        cstData_.lowBound_.resize(ndof*N_);
        cstData_.cstMatrix_.resize(ndof*N_,ndof*N_);
    }

    void setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max);

    void setLowerBound(const Eigen::VectorXd &robot_state);

    void setUpperBound(const Eigen::VectorXd &robot_state);

    void setConstraintMatrix();

    void update(const Eigen::VectorXd &robot_state);
protected:


};

#endif
#ifndef RosWrapJointAccCst_H
#define RosWrapJointAccCst_H
#pragma once
#include "RosWrapconstraint/RosWrapGenericCst.h"

/**
 * @brief Acceleration constraint
 */
class  JntAccCst : public GenericCst
{

public:

    JntAccCst(int ndof, int N,double dt, std::string name, Eigen::MatrixXd Px, Eigen::MatrixXd Pu)
        : GenericCst(ndof, N,dt, name, Px, Pu) {

    }

     ~JntAccCst(){}


    void setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){

         min_ = min ;
         max_ = max;
    }


protected:


};

#endif
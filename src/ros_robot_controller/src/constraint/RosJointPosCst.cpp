#include "constraint/RosJointPosCst.h"



void joint_pos_cst::setLimit(Eigen::VectorXd min, Eigen::VectorXd max){
    min_ = min;
    max_ = max;
}



void joint_pos_cst::setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px){
    lb_ = min_ - robot_state*Px;
}

void joint_pos_cst::setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px)
{
    ub_ = max_ - robot_state*Px;
}

void joint_pos_cst::setConstraintMatrix(Eigen::MatrixXd Pu){
    A_ = Pu;
}

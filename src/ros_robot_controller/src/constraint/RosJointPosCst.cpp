#include "constraint/RosJointPosCst.h"



void jntPosCst::setLimit(Eigen::VectorXd min, Eigen::VectorXd max){
    min_ = min;
    max_ = max;
}



void jntPosCst::setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px){
    lb_ = min_ - Px*robot_state;
}

void jntPosCst::setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px)
{
    ub_ = max_ - Px*robot_state;
}

void jntPosCst::setConstraintMatrix(Eigen::MatrixXd Pu){
    A_ = Pu;
}

void jntPosCst::update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu){
    setLowerBound( robot_state, Px);
    setUpperBound(robot_state, Px);
    setConstraintMatrix(Pu);
}

#include "constraint/RosJointPosCst.h"



void jntPosCst::setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){
    min_ = min;
    max_ = max;
}



void jntPosCst::setLowerBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px){
    lbA_ = min_ - Px*robot_state;
    cstData_.lowBound_ = lbA_;
}

void jntPosCst::setUpperBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px)
{
    ubA_ = max_ - Px*robot_state;
    cstData_.upBound_ = ubA_;
}

void jntPosCst::setConstraintMatrix(const Eigen::MatrixXd &Pu){
    A_ = Pu;
    cstData_.cstMatrix_ = A_;
}

void jntPosCst::update(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px, const Eigen::MatrixXd &Pu){
    setLowerBound( robot_state, Px);
    setUpperBound(robot_state, Px);
    setConstraintMatrix(Pu);
}

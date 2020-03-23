#include "RosWrapconstraint/RosWrapJointPosCst.h"



void JntPosCst::setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){
    min_ = min;
    max_ = max;
}



void JntPosCst::setLowerBound(const Eigen::VectorXd &robot_state){
    lbA_ = min_ - Px_*robot_state;
    cstData_.lowBound_ = lbA_;
}

void JntPosCst::setUpperBound(const Eigen::VectorXd &robot_state)
{
    ubA_ = max_ - Px_*robot_state;
    cstData_.upBound_ = ubA_;
}

void JntPosCst::setConstraintMatrix(){
    A_ = Pu_;
    cstData_.cstMatrix_ = A_;
}

void JntPosCst::update(const Eigen::VectorXd &robot_state){
    setLowerBound( robot_state);
    setUpperBound(robot_state);
    setConstraintMatrix();
}

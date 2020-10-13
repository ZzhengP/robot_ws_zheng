#include "RosWrapconstraint/RosWrapJointVelCst.h"


void JntVelCst::setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){
        min_ = min;
        max_ = max;

}

void JntVelCst::setLowerBound(const Eigen::VectorXd &robot_state){
    lbA_ = min_ - Px_*robot_state;
    cstData_.lowBound_ = lbA_;
}

void JntVelCst::setUpperBound(const Eigen::VectorXd &robot_state){
    ubA_ = max_ - Px_*robot_state;
    cstData_.upBound_ = ubA_;
}

void JntVelCst::setConstraintMatrix(){
    A_ = Pu_;
    cstData_.cstMatrix_ = A_;
}
void JntVelCst::update(const Eigen::VectorXd &robot_state){
     setLowerBound( robot_state);
     setUpperBound( robot_state);
     setConstraintMatrix();
}


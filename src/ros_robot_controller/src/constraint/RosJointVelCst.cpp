#include "constraint/RosJointVelCst.h"


void jntVelCst::setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){
        min_ = min;
        max_ = max;

}

void jntVelCst::setLowerBound(const Eigen::VectorXd &robot_state){
    lbA_ = min_ - Px_*robot_state;
    cstData_.lowBound_ = lbA_;
}

void jntVelCst::setUpperBound(const Eigen::VectorXd &robot_state){
    ubA_ = max_ - Px_*robot_state;
    cstData_.upBound_ = ubA_;
}

void jntVelCst::setConstraintMatrix(){
    A_ = Pu_;
    cstData_.cstMatrix_ = A_;
}
void jntVelCst::update(const Eigen::VectorXd &robot_state){
     setLowerBound( robot_state);
     setUpperBound( robot_state);
     setConstraintMatrix();
}


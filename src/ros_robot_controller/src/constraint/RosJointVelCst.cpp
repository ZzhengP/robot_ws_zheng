#include "constraint/RosJointVelCst.h"


void jntVelCst::setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){
        min_ = min;
        max_ = max;

}

void jntVelCst::setLowerBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px){
    lbA_ = min_ - Px*robot_state;
    cstData_.lowBound_ = lbA_;
}

void jntVelCst::setUpperBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px){
    ubA_ = max_ - Px*robot_state;
    cstData_.upBound_ = ubA_;
}

void jntVelCst::setConstraintMatrix(const Eigen::MatrixXd &Pu){
    A_ = Pu;
    cstData_.cstMatrix_ = A_;
}
void jntVelCst::update(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px, const Eigen::MatrixXd &Pu){
     setLowerBound( robot_state, Px);
     setUpperBound( robot_state, Px);
     setConstraintMatrix(Pu);
}


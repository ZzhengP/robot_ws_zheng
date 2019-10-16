#include "constraint/RosJointVelCst.h"


void jnt_vel_cst::setLimit(Eigen::VectorXd min, Eigen::VectorXd max){
        min_ = min;
        max_ = max;

}

void jnt_vel_cst::setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px){
    lb_ = min_ - Px*robot_state;
}

void jnt_vel_cst::setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px){
    ub_ = max_ - Px*robot_state;
}

void jnt_vel_cst::setConstraintMatrix(Eigen::MatrixXd Pu){
    A_ = Pu;
}
void jnt_vel_cst::update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu){
     setLowerBound( robot_state, Px);
     setUpperBound( robot_state, Px);
     setConstraintMatrix(Pu);
}

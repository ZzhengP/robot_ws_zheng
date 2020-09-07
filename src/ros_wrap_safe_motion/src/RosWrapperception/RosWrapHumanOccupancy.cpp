#include "RosWrapperception/RosWrapHumanOccupancy.h"

RosWrapHumanOccupancy::RosWrapHumanOccupancy(const double & dt, const int &ndof):dt_(dt),ndof_(ndof){
std::cout <<" Human Occupancy module " << std::endl;
vel_.resize(3), vel_.setZero();
hand_pos_.resize(3), hand_pos_.setZero();
q_.resize(ndof_), q_.setZero();
state_.resize(ndof_+3), state_.setZero();
J_lin_.resize(3,ndof_);
X_.resize(3,3);


}

Eigen::VectorXd RosWrapHumanOccupancy::computeDynamic(const Eigen::VectorXd  & state,const Eigen::MatrixXd &J_pseudo_inv,
                                       const Eigen::VectorXd &vel){
        int len = state_.size();                                   
        Eigen::VectorXd state_forward(len);

        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(len,len);
        Eigen::MatrixXd B(len,3);
        B.block(0,0,ndof_,3) = J_pseudo_inv.block(0,0,ndof_,3);
        B.block(ndof_,0,3,3) = Eigen::MatrixXd::Identity(3,3);      
        B = B*dt_;
        state_forward = A*state + B*vel;                                

        return state_forward;
}
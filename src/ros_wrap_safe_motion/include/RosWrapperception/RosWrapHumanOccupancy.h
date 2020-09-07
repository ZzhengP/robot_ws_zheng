#ifndef ROSWRAPHUMANOCCUPANCY_H
#define ROSWRAPHUMANOCCUPANCY_H
#pragma once 
#include <Eigen/Dense>
#include <iostream>
class RosWrapHumanOccupancy
{

public:
    RosWrapHumanOccupancy(const double & dt, const int & ndof);
    ~RosWrapHumanOccupancy(){
    };

    Eigen::VectorXd computeDynamic(const Eigen::VectorXd  & state, const Eigen::MatrixXd &J_pseudo_inv, const Eigen::VectorXd &vel);
private:

    int ndof_;
    double dt_;
    Eigen::VectorXd vel_;
    Eigen::VectorXd state_, q_, hand_pos_;
    Eigen::MatrixXd J_lin_, X_;
};


 
#endif
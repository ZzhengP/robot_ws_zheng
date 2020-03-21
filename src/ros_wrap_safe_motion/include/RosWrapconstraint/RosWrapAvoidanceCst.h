#ifndef RosWrapMpcAvoidanceCst_H
#define RosWrapMpcAvoidanceCst_H

#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <string>
#include "RosWrapsepPlane/RosWrapsepPlane.h"
#include "RosWrapconstraint/RosWrapGenericCst.h"
#include <fstream>

using namespace  std;
using namespace  plane;
class ObsAvoidanceCSt{

public:

    ObsAvoidanceCSt(int ndof, int N, double dt, double dsafe, std::string,Eigen::MatrixXd Px, Eigen::MatrixXd Pu);

    ~ObsAvoidanceCSt();

    void setLowerBound();

    void setUpperBoundAndConstraint(const std::vector<Eigen::MatrixXd> &RobotVertices,
                       const Planes::PlaneData& PlaneData,
                       const Eigen::VectorXd& robotState,
                       const Eigen::MatrixXd& JacobianHorizon,
                       const Eigen::VectorXd &qHorizonPrecedent);


    constraintData getConstraintData(){
           return cstData_;
    }

protected:
//    Eigen::VectorXd lbA_, ubA_;
    Eigen::MatrixXd  Px_, Pu_;
    Eigen::MatrixXd A_;
    Eigen::VectorXd lbA_, ubA_;
    double dt_, dsafe_;
    unsigned int  N_, nbrRobotPart_, nbrObstacle_, ndof_ ;
    std::string cst_name_ ;
    constraintData cstData_;
};

#endif
#pragma once
#include "iostream"
#include <Eigen/Dense>
#include <Eigen/Core>

class  generic_cst{
public:
     generic_cst(int ndof, int N);

     virtual ~generic_cst();

     virtual void setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);

     virtual void setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);

     virtual void setConstraintMatrix(Eigen::MatrixXd Pu) ;

     Eigen::VectorXd getLowerBound(){
            return lb_;
     }

     Eigen::VectorXd getUpperBound(){
            return ub_;
     }

     Eigen::MatrixXd getConstraintMatrix(){
            return A_;
     }


protected:
     Eigen::VectorXd lb_, ub_;
     Eigen::MatrixXd A_;

     int NrOfDeg_, N_Prediction_ ;

};

#pragma once
#include "qpOASES.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "commun/color.h"

class PlaneSolver
{
    public:

    PlaneSolver(int nbrCst, double dsafe);

    void setCost(const Eigen::VectorXd&  g);

    void setCstMatrix(const Eigen::MatrixXd& robotPartielVertices,
                      const Eigen::MatrixXd& obsPartielVertices, const Eigen::MatrixXd & dataPlanePrecedent);

    bool solve();

    void print(){

        std::cout <<" lb_ :\n" << lb_ << std::endl;
        std::cout <<" ub_ :\n" << ub_ << std::endl;
        std::cout <<" lbA_ :\n" << lbA_ << std::endl;
        std::cout <<" ubA_ :\n" << ubA_ << std::endl;
        std::cout <<" A_  :\n" <<  A_  << std::endl;

        std::cout <<" H_ :\n" << H_ << std::endl;
        std::cout <<" g_ :\n" << g_ << std::endl;
    }


    Eigen::VectorXd getSolution(){
        return data_optimal_solution_;
    }

    Eigen::MatrixXd getHessien(){
        return H_;
    }

    Eigen::MatrixXd getConstraintMatrix(){
        return A_;
    }

    Eigen::VectorXd getConstraintLowerBound(){
        return lbA_;
    }

    Eigen::VectorXd getConstraintUpperBound(){
        return ubA_;
    }

    Eigen::VectorXd getLowerBound(){
        return lb_;
    }

    Eigen::VectorXd getUpperBound(){
        return ub_;
    }

    Eigen::VectorXd getGradient(){
        return g_;
    }
private:
     int nV_ , nC_;
     double dsafe_;
     std::shared_ptr<qpOASES::QProblem> qpPlane_;
     qpOASES::Options options_;

     Eigen::VectorXd g_, lbA_, ubA_, lb_, ub_ ;
     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
     Eigen::MatrixXd E_;
     Eigen::VectorXd data_optimal_solution_ ;
     qpOASES::MessageHandling msg_;
};




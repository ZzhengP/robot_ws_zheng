#pragma once
#include "qpOASES.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "commun/color.h"

class lpSolver
{
    public:

    lpSolver(int N_horz, int Ndof, int nbrCst);

    bool initData(Eigen::MatrixXd A,Eigen::VectorXd lb, Eigen::VectorXd ub, Eigen::VectorXd lbA, Eigen::VectorXd ubA);


    void computeCost();
    // Compute the constraint to ensure plane's norm to not diverge
    void computeNonDivCst();
    // Compute the constraint to force plane's norm equal to 1
    void computeNormCst(const Eigen::MatrixXd & robotPartielVertices,
                        const Eigen::MatrixXd & obsPartielVertices, const Eigen::MatrixXd & dataPlanePrecedent,double epsilon);


    void Update(const Eigen::MatrixXd & robotPartielVertices,
                      const Eigen::MatrixXd & obsPartielVertices, const Eigen::MatrixXd & dataPlanePrecedent,double epsilon);

    void solve(){
        qpOASES::returnValue ret;
        qpOASES::int_t nWSR = 1000;
        lp_solver_->setOptions(options_);
        ret = lp_solver_->init(0,g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR,0);
        lp_solver_->getPrimalSolution(data_optimal_solution_.data());

    }

    Eigen::VectorXd getSolution()
    {
         return data_optimal_solution_;
    }
     void setDefaultOptions()
     {
         if(!lp_solver_)
             throw std::runtime_error("qpsolver pointer is null");

         options_.setToMPC(); // setToReliable() // setToDefault()
         options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
         options_.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
         lp_solver_->setOptions( options_ );
         lp_solver_->setPrintLevel(qpOASES::PL_NONE);
         lp_solver_test_->setOptions(options_);
         lp_solver_test_->setPrintLevel(qpOASES::PL_NONE);
     }

    private:
    // Define an solver for linear programming
    std::shared_ptr<qpOASES::QProblem> lp_solver_;
    std::shared_ptr<qpOASES::QProblemB> lp_solver_test_;
    qpOASES::Options options_;
    int N_prediction_, Ndof_;
    Eigen::MatrixXd H_, A_;
    Eigen::VectorXd g_, lb_, ub_, lbA_, ubA_, data_optimal_solution_;


    int nV_, nC_ ;
};



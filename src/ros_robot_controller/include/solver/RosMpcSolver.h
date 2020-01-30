#pragma once
#include "qpOASES.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "commun/color.h"
#include "constraint/RosGenericCst.h"
#include "vector"

class mpc_solve
{
public:
    mpc_solve(int N_horz, int Ndof, int nbrCst);
    ~mpc_solve(){}
    bool initData(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub);


    bool solve(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub);


    bool solve();

    void constructProblem(const std::vector<constraintData> &constraintVectorData,const Eigen::MatrixXd H, const Eigen::VectorXd g);

   Eigen::VectorXd getSolution()
   {
        return data_optimal_solution_;
   }

    void setDefaultOptions()
    {
        if(!qpoases_)
            throw std::runtime_error("qpsolver pointer is null");

        qpoases_->setOptions( options_ );
        qpoases_->setPrintLevel(qpOASES::PL_HIGH);

        qpoases_standard_->setOptions( options_ );
        qpoases_standard_->setPrintLevel(qpOASES::PL_HIGH);
    }


private:
    std::shared_ptr<qpOASES::QProblemB> qpoases_;
    std::shared_ptr<qpOASES::SQProblem> qpoases_standard_;
    qpOASES::Options options_;
    int N_prediction_, Ndof_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    Eigen::VectorXd g_, lb_, ub_, lbA_, ubA_, data_optimal_solution_;


    int nV_, nC_ ;
};

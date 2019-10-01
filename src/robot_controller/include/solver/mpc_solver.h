#include "qpOASES.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/shared_ptr.hpp>
class mpc_solve
{
public:
    mpc_solve(int N_horz, int Ndof)
    {
        N_prediction_ = N_horz;
        Ndof_ = Ndof;
        nV_ = Ndof*N_horz; // wo cao, wo ri le  !!!!!
        nC_ = 0;
        options_.enableFlippingBounds = qpOASES::BT_FALSE;
        options_.initialStatusBounds = qpOASES::ST_INACTIVE;
        options_.numRefinementSteps = 1;
        qpoases_.reset(new qpOASES::QProblemB(nV_));
        qpoases_standard_.reset(new qpOASES::QProblem());
    }

    bool initData(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub)
    {
        H_.resize(H.rows(),H.cols());
        g_.resize(g.size());
        lb_.resize(lb.size());
        ub_.resize(ub.size());
        data_optimal_solution_.resize(nV_);

        H_ = H;
        g_ = g;
        lb_ = lb;
        ub_ = ub;
        return true;
    }

    bool initData(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::MatrixXd A,Eigen::VectorXd lb, Eigen::VectorXd ub, Eigen::VectorXd lbA, Eigen::VectorXd ubA)
    {
        H_.resize(H.rows(),H.cols());
        g_.resize(g.size());
        lb_.resize(lb.size());
        ub_.resize(ub.size());
        A_.resize(A.rows(),A.cols());
        lbA_.resize(lbA.size());
        ubA_.resize(ubA.size());

        data_optimal_solution_.resize(nV_);

        H_ = H;
        g_ = g;
        lb_ = lb;
        ub_ = ub;
        A_ = A;
        lbA_ = lbA;
        ubA_ = ubA;
        return true;
    }
    void solve(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub){
         qpOASES::returnValue ret;
         qpOASES::int_t nWSR = 1000;
         qpoases_->setOptions(options_);
//         ret = qpoases_->init(H_.data(),g_.data(),lb_.data(),ub_.data(),nWSR,0);
          ret = qpoases_->init(H.data(),g.data(),lb.data(),ub.data(),nWSR,0);
         qpoases_->getPrimalSolution(data_optimal_solution_.data());
    }

    void solve(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::MatrixXd A,Eigen::VectorXd lb, Eigen::VectorXd ub, Eigen::VectorXd lbA, Eigen::VectorXd ubA){
         qpOASES::returnValue ret;
         qpOASES::int_t nWSR = 1000;
         qpoases_->setOptions(options_);
//         ret = qpoases_->init(H_.data(),g_.data(),lb_.data(),ub_.data(),nWSR,0);
          ret = qpoases_standard_->init(H.data(),g.data(),A.data(),lb.data(),ub.data(),lbA.data(),ubA.data(),nWSR,0);
         qpoases_standard_->getPrimalSolution(data_optimal_solution_.data());
    }
   Eigen::VectorXd getSolution()
   {
        return data_optimal_solution_;
   }
    void setDefaultOptions()
    {
        if(!qpoases_)
            throw std::runtime_error("qpsolver pointer is null");

        options_.setToMPC(); // setToReliable() // setToDefault()
        options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
        options_.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
        qpoases_->setOptions( options_ );
        qpoases_->setPrintLevel(qpOASES::PL_NONE);
    }


private:
    std::shared_ptr<qpOASES::QProblemB> qpoases_;
    std::shared_ptr<qpOASES::QProblem> qpoases_standard_;
    qpOASES::Options options_;
    int N_prediction_, Ndof_;
    Eigen::MatrixXd H_, A_;
    Eigen::VectorXd g_, lb_, ub_, lbA_, ubA_, data_optimal_solution_;


    int nV_, nC_;
};

#include "solver/lpsolver.h"

lpSolver::lpSolver(int N_horz, int Ndof, int nbrCst):N_prediction_(N_horz), Ndof_(Ndof),nC_(nbrCst)
{
    nV_ = 5;
    options_.enableFlippingBounds = qpOASES::BT_FALSE;
    options_.initialStatusBounds = qpOASES::ST_INACTIVE;
    options_.numRefinementSteps = 1;
    lp_solver_.reset(new qpOASES::QProblem(nV_,nC_,qpOASES::HST_ZERO));
    lp_solver_test_.reset(new qpOASES::QProblemB(nV_));
    g_.resize(5);

    lb_.resize(5);
    lb_.setConstant(-1);
    ub_.resize(5);
    ub_.setConstant(1);
    lbA_.resize(nbrCst);
    lbA_.setConstant(-1);
    ubA_.resize(nbrCst);
    ubA_.setConstant(1);
    A_.resize(nbrCst,nV_);
    A_.setZero();
    g_ << 0, 0, 0, 0, -1;
    lb_ << -1, -1, -1 ,0,0;
    ub_ << 1, 1, 1,  1, 2;
    data_optimal_solution_.resize(nV_);
    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << BOLD(FRED("lps solver is successively created: "))  <<'\n'
              << FYEL("The number of optimization variable is (n,d,r) : ") << nV_ << std::endl;

    std::cout <<"---------------------------------------------" << std::endl;
}



void lpSolver::computeCost(){
    g_ << 0,0,0,0,-1;

}

void lpSolver::computeNonDivCst(){
    lb_ << -1, -1, -1 ,0,0;
    ub_ << 1, 1, 1, 0.2,0.2;

}

void lpSolver::computeNormCst(const Eigen::MatrixXd &robotPartielVertices,
                            const Eigen::MatrixXd & obsPartielVertices, const Eigen::MatrixXd & dataPlanePrecedent, double epsilon = 0.01){


//    ubA_ << -0.1,-0.1,-0.1,-0.1,-0.1,-0.1,1;
    A_.block(0,0,1,5) << robotPartielVertices.block(0,0,3,1).transpose(),-1,1;
    A_.block(1,0,1,5) << robotPartielVertices.block(0,1,3,1).transpose(),-1,1;
//    A_.block(2,0,1,5) << robotPartielVertices.block(0,2,3,1).transpose(),-1,0;
//    A_.block(3,0,1,5) << robotPartielVertices.block(0,3,3,1).transpose(),-1,0;
//    A_.block(2,0,1,5) << -obsPartielVertices.block(0,0,3,1).transpose(),1,0;
    A_.block(2,0,1,5) << -obsPartielVertices.block(0,1,3,1).transpose(),1,0;
    A_.block(3,0,1,5) << dataPlanePrecedent.transpose(),0,0;
    lbA_[3] = 0.1;
//    ubA_ << 0.,0.,0.,0., 1;
    ubA_ << 0. ,0.,0., 1.;
    std::cout <<"lb_ \n" << lb_ << '\n' ;
    std::cout <<"ub_ \n" << ub_ << '\n' ;
    std::cout <<"A_ \n" << A_ << '\n' ;
    std::cout <<"lbA_ \n" << lbA_ << '\n' ;
    std::cout <<"ubA_ \n" << ubA_ << '\n' ;

}

void lpSolver::Update(const Eigen::MatrixXd &robotPartielVertices, const Eigen::MatrixXd &obsPartielVertices, const Eigen::MatrixXd &dataPlanePrecedent, double epsilon){
   computeCost();
   computeNonDivCst();
   computeNormCst(robotPartielVertices, obsPartielVertices, dataPlanePrecedent, epsilon);
}

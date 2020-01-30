#include "solver/RosMpcSolver.h"

mpc_solve::mpc_solve(int N_horz, int Ndof, int nbrCst):N_prediction_(N_horz), Ndof_(Ndof),nC_(nbrCst)
{
    nV_ = Ndof*N_horz; // wo cao, wo ri le  !!!!!
    options_.enableFlippingBounds = qpOASES::BT_FALSE;
    options_.enableRegularisation = qpOASES::BT_TRUE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options_.enableEqualities = qpOASES::BT_TRUE; //
    options_.numRefinementSteps = 5;
    qpoases_.reset(new qpOASES::QProblemB(nV_));
    qpoases_standard_.reset(new qpOASES::SQProblem(nV_,nbrCst,qpOASES::HST_SEMIDEF));
    data_optimal_solution_.resize(nV_);

    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << BOLD(FRED("QPs solver is successively created: "))  <<'\n'
              << FYEL("The number of optimization variable is : ") << nV_ << std::endl;
    std::cout <<"---------------------------------------------" << std::endl;
}

bool mpc_solve::initData(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub)
{
    data_optimal_solution_.resize(nV_);
    lb_ = lb;
    ub_ = ub;
    A_.resize(nC_,nV_);
    lbA_.resize(nC_);
    ubA_.resize(nC_);
//        std::cout <<"---------------------------------------------" << std::endl;
//        std::cout << BOLD(FRED("QPs standard formulation solver is successively initialized: "))  <<std::endl;
//        std::cout <<"---------------------------------------------" << std::endl;
    return true;

}

bool mpc_solve::solve(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub){
     qpOASES::returnValue ret;
     qpOASES::int_t nWSR = 1000000;
     H_ = H;
     g_ = g;
     lb_ = lb;
     ub_ = ub;

     qpoases_->setOptions(options_);
      ret = qpoases_->init(H.data(),g.data(),lb.data(),ub.data(),nWSR,0);
      if (ret == qpOASES::SUCCESSFUL_RETURN){
     qpoases_->getPrimalSolution(data_optimal_solution_.data());
             return true;}
      else{
            return false;
      }
}

bool mpc_solve::solve(){
     qpOASES::returnValue ret;
     qpOASES::int_t nWSR = 1000000;
     static bool qpoases_initialized = false;

     if(!qpoases_initialized)
           {
         // Initialise the problem, once it has found a solution, we can hotstart
         ret = qpoases_standard_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

         // Keep init if it didn't work
         if(ret == qpOASES::SUCCESSFUL_RETURN)
         {
           qpoases_initialized = true;
         }
           }
           else{
         // Otherwise let's reuse the previous solution to find a solution faster
         ret = qpoases_standard_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

         if(ret != qpOASES::SUCCESSFUL_RETURN){
           qpoases_initialized = false;
         }
           }

           // Zero grav if not found
           // TODO: find a better alternative

           if(ret == qpOASES::SUCCESSFUL_RETURN){
         // Get the solution
                qpoases_standard_->getPrimalSolution(data_optimal_solution_.data());
                std::cout << qpoases_standard_->printProperties() << '\n';
                 return true;
           }else{
               return false;
           }

}



void mpc_solve::constructProblem(const std::vector<constraintData> &constraintVectorData, const Eigen::MatrixXd H, const Eigen::VectorXd g){

    unsigned int nbr_cst = constraintVectorData.size();
    int rows, cols;
    int rows_cumul ;
    rows_cumul = 0;
    for (int i(0); i<nbr_cst ; i++) {


        rows = constraintVectorData[i].cstMatrix_.rows();
        cols = constraintVectorData[i].cstMatrix_.cols();


        A_.block(rows_cumul,0,rows,cols) = constraintVectorData[i].cstMatrix_;
        lbA_.segment(rows_cumul,constraintVectorData[i].lowBound_.size()) = constraintVectorData[i].lowBound_;
        ubA_.segment(rows_cumul,constraintVectorData[i].upBound_.size()) = constraintVectorData[i].upBound_;
        rows_cumul += rows;
    }

    H_ = H;
    g_ = g;


}

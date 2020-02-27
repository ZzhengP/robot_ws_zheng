#include "solver/RosMpcSolver.h"

using namespace qpOASES;
enum  	returnValue {
  TERMINAL_LIST_ELEMENT = -1, SUCCESSFUL_RETURN = 0, RET_DIV_BY_ZERO, RET_INDEX_OUT_OF_BOUNDS,
  RET_INVALID_ARGUMENTS, RET_ERROR_UNDEFINED, RET_WARNING_UNDEFINED, RET_INFO_UNDEFINED,
  RET_EWI_UNDEFINED, RET_AVAILABLE_WITH_LINUX_ONLY, RET_UNKNOWN_BUG, RET_PRINTLEVEL_CHANGED,
  RET_NOT_YET_IMPLEMENTED, RET_INDEXLIST_MUST_BE_REORDERD, RET_INDEXLIST_EXCEEDS_MAX_LENGTH, RET_INDEXLIST_CORRUPTED,
  RET_INDEXLIST_OUTOFBOUNDS, RET_INDEXLIST_ADD_FAILED, RET_INDEXLIST_INTERSECT_FAILED, RET_INDEX_ALREADY_OF_DESIRED_STATUS,
  RET_ADDINDEX_FAILED, RET_REMOVEINDEX_FAILED, RET_SWAPINDEX_FAILED, RET_NOTHING_TO_DO,
  RET_SETUP_BOUND_FAILED, RET_SETUP_CONSTRAINT_FAILED, RET_MOVING_BOUND_FAILED, RET_MOVING_CONSTRAINT_FAILED,
  RET_SHIFTING_FAILED, RET_ROTATING_FAILED, RET_QPOBJECT_NOT_SETUP, RET_QP_ALREADY_INITIALISED,
  RET_NO_INIT_WITH_STANDARD_SOLVER, RET_RESET_FAILED, RET_INIT_FAILED, RET_INIT_FAILED_TQ,
  RET_INIT_FAILED_CHOLESKY, RET_INIT_FAILED_HOTSTART, RET_INIT_FAILED_INFEASIBILITY, RET_INIT_FAILED_UNBOUNDEDNESS,
  RET_INIT_FAILED_REGULARISATION, RET_INIT_SUCCESSFUL, RET_OBTAINING_WORKINGSET_FAILED, RET_SETUP_WORKINGSET_FAILED,
  RET_SETUP_AUXILIARYQP_FAILED, RET_NO_CHOLESKY_WITH_INITIAL_GUESS, RET_NO_EXTERN_SOLVER, RET_QP_UNBOUNDED,
  RET_QP_INFEASIBLE, RET_QP_NOT_SOLVED, RET_QP_SOLVED, RET_UNABLE_TO_SOLVE_QP,
  RET_INITIALISATION_STARTED, RET_HOTSTART_FAILED, RET_HOTSTART_FAILED_TO_INIT, RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED,
  RET_ITERATION_STARTED, RET_SHIFT_DETERMINATION_FAILED, RET_STEPDIRECTION_DETERMINATION_FAILED, RET_STEPLENGTH_DETERMINATION_FAILED,
  RET_OPTIMAL_SOLUTION_FOUND, RET_HOMOTOPY_STEP_FAILED, RET_HOTSTART_STOPPED_INFEASIBILITY, RET_HOTSTART_STOPPED_UNBOUNDEDNESS,
  RET_WORKINGSET_UPDATE_FAILED, RET_MAX_NWSR_REACHED, RET_CONSTRAINTS_NOT_SPECIFIED, RET_INVALID_FACTORISATION_FLAG,
  RET_UNABLE_TO_SAVE_QPDATA, RET_STEPDIRECTION_FAILED_TQ, RET_STEPDIRECTION_FAILED_CHOLESKY, RET_CYCLING_DETECTED,
  RET_CYCLING_NOT_RESOLVED, RET_CYCLING_RESOLVED, RET_STEPSIZE, RET_STEPSIZE_NONPOSITIVE,
  RET_SETUPSUBJECTTOTYPE_FAILED, RET_ADDCONSTRAINT_FAILED, RET_ADDCONSTRAINT_FAILED_INFEASIBILITY, RET_ADDBOUND_FAILED,
  RET_ADDBOUND_FAILED_INFEASIBILITY, RET_REMOVECONSTRAINT_FAILED, RET_REMOVEBOUND_FAILED, RET_REMOVE_FROM_ACTIVESET,
  RET_ADD_TO_ACTIVESET, RET_REMOVE_FROM_ACTIVESET_FAILED, RET_ADD_TO_ACTIVESET_FAILED, RET_CONSTRAINT_ALREADY_ACTIVE,
  RET_ALL_CONSTRAINTS_ACTIVE, RET_LINEARLY_DEPENDENT, RET_LINEARLY_INDEPENDENT, RET_LI_RESOLVED,
  RET_ENSURELI_FAILED, RET_ENSURELI_FAILED_TQ, RET_ENSURELI_FAILED_NOINDEX, RET_ENSURELI_FAILED_CYCLING,
  RET_BOUND_ALREADY_ACTIVE, RET_ALL_BOUNDS_ACTIVE, RET_CONSTRAINT_NOT_ACTIVE, RET_BOUND_NOT_ACTIVE,
  RET_HESSIAN_NOT_SPD, RET_HESSIAN_INDEFINITE, RET_MATRIX_SHIFT_FAILED, RET_MATRIX_FACTORISATION_FAILED,
  RET_PRINT_ITERATION_FAILED, RET_NO_GLOBAL_MESSAGE_OUTPUTFILE, RET_DISABLECONSTRAINTS_FAILED, RET_ENABLECONSTRAINTS_FAILED,
  RET_ALREADY_ENABLED, RET_ALREADY_DISABLED, RET_NO_HESSIAN_SPECIFIED, RET_USING_REGULARISATION,
  RET_EPS_MUST_BE_POSITVE, RET_REGSTEPS_MUST_BE_POSITVE, RET_HESSIAN_ALREADY_REGULARISED, RET_CANNOT_REGULARISE_IDENTITY,
  RET_CANNOT_REGULARISE_SPARSE, RET_NO_REGSTEP_NWSR, RET_FEWER_REGSTEPS_NWSR, RET_CHOLESKY_OF_ZERO_HESSIAN,
  RET_ZERO_HESSIAN_ASSUMED, RET_CONSTRAINTS_ARE_NOT_SCALED, RET_INITIAL_BOUNDS_STATUS_NYI, RET_ERROR_IN_CONSTRAINTPRODUCT,
  RET_FIX_BOUNDS_FOR_LP, RET_USE_REGULARISATION_FOR_LP, RET_UPDATEMATRICES_FAILED, RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED,
  RET_UNABLE_TO_OPEN_FILE, RET_UNABLE_TO_WRITE_FILE, RET_UNABLE_TO_READ_FILE, RET_FILEDATA_INCONSISTENT,
  RET_OPTIONS_ADJUSTED, RET_UNABLE_TO_ANALYSE_QPROBLEM, RET_NWSR_SET_TO_ONE, RET_UNABLE_TO_READ_BENCHMARK,
  RET_BENCHMARK_ABORTED, RET_INITIAL_QP_SOLVED, RET_QP_SOLUTION_STARTED, RET_BENCHMARK_SUCCESSFUL,
  RET_NO_DIAGONAL_AVAILABLE, RET_DIAGONAL_NOT_INITIALISED, RET_ENSURELI_DROPPED, RET_SIMPLE_STATUS_P1,
  RET_SIMPLE_STATUS_P0, RET_SIMPLE_STATUS_M1, RET_SIMPLE_STATUS_M2, RET_SIMPLE_STATUS_M3
};
mpc_solve::mpc_solve(int N_horz, int Ndof, int nbrCst):N_prediction_(N_horz), Ndof_(Ndof),nC_(nbrCst)
{
    nV_ = Ndof*N_horz; // wo cao, wo ri le  !!!!!
    options_.enableFlippingBounds = qpOASES::BT_FALSE;
    options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options_.enableEqualities = qpOASES::BT_TRUE; //
    options_.numRefinementSteps = 10;
    qpoases_.reset(new qpOASES::QProblemB(nV_));
    qpoases_standard_.reset(new qpOASES::SQProblem(nV_,nbrCst,qpOASES::HST_SEMIDEF));
    qpoases_standard_->setPrintLevel(qpOASES::PL_DEBUG_ITER);
    data_optimal_solution_.resize(nV_);
    data_optimal_solution_.setZero();
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
     qpOASES::returnValue ret2;

     qpOASES::int_t nWSR = 1000000;
     static bool qpoases_initialized = false;
     qpoases_standard_->setPrintLevel(qpOASES::PL_HIGH);
//     if(!qpoases_initialized)
//           {
         // Initialise the problem, once it has found a solution, we can hotstart

         qpoases_standard_->setOptions(options_);

//         ret = qpoases_standard_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR,0,data_optimal_solution_.data(),0);
         ret = qpoases_standard_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

         qpoases_standard_->printProperties();

         // Keep init if it didn't work
//         if(ret == qpOASES::SUCCESSFUL_RETURN)
//         {
//           qpoases_initialized = true;
//         }
//           }
//           else{
//         // Otherwise let's reuse the previous solution to find a solution faster
//         ret = qpoases_standard_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

//         if(ret != qpOASES::SUCCESSFUL_RETURN){
//           qpoases_initialized = false;
//         }
//           }

           // Zero grav if not found
           // TODO: find a better alternative

           if(ret == qpOASES::SUCCESSFUL_RETURN){
         // Get the solution
                qpoases_standard_->getPrimalSolution(data_optimal_solution_.data());
                 return true;
           }else{

                   return false;
           }

}



void mpc_solve::constructProblem(const std::vector<constraintData> &constraintVectorData, const Eigen::MatrixXd H, const Eigen::VectorXd g){

    int nbr_cst = constraintVectorData.size();
    int rows_cumul ;
    rows_cumul = 0;
    for (int i(0); i<nbr_cst ; i++) {
        int rows, cols;
        rows =0;
        cols = 0;
        rows = constraintVectorData[i].cstMatrix_.rows();
        cols = constraintVectorData[i].cstMatrix_.cols();

//        std::cout <<"constraint name :\n " << constraintVectorData[i].name_ << '\n';
//        std::cout <<" rows :\n " << rows <<'\n';
//        std::cout <<" cols :\n " << cols <<'\n';
//        std::cout <<" lbA_ :\n  " << constraintVectorData[i].lowBound_ <<'\n' ;
//        std::cout <<" ubA_ :\n  " << constraintVectorData[i].upBound_ <<'\n' ;
//        std::cout <<" rows cumul  :\n  " << rows_cumul <<'\n' ;

        A_.block(rows_cumul,0,rows,cols) = constraintVectorData[i].cstMatrix_;
        lbA_.segment(rows_cumul,constraintVectorData[i].lowBound_.size()) = constraintVectorData[i].lowBound_;
        ubA_.segment(rows_cumul,constraintVectorData[i].upBound_.size()) = constraintVectorData[i].upBound_;
        rows_cumul += rows;
    }

    H_ = H;
    g_ = g;

    std::ofstream myfile;
    myfile.open ("/home/zheng/Bureau/PBPproblem.txt");
   qpoases_standard_->setOptions(options_);
   myfile << "H_ \n " <<  H_ << '\n';
   myfile << "g_ \n " << g_ << '\n';
   myfile << "A_ \n " << A_ << '\n';
   myfile << "lbA_ \n " << lbA_ << '\n';
   myfile << "ubA_ \n " << ubA_ << '\n';
   myfile << "lb_ \n " << lb_ << '\n';
   myfile << "ub_ \n " << ub_ << '\n';
   myfile.close();
}

#include "RosWrapsolver/RosWrapPlaneSolver.h"
#include <fstream>
PlaneSolver::PlaneSolver(int nbrCst, double dsafe):nC_(nbrCst), dsafe_(dsafe)
{
    nV_ = 5; // wo cao, wo ri le  !!!!!
//    options_.enableFlippingBounds = qpOASES::BT_FALSE;
//    options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
//    options_.enableEqualities = qpOASES::BT_TRUE; //
    options_.numRefinementSteps = 10;
    qpPlane_.reset(new qpOASES::QProblem(nV_,nbrCst,qpOASES::HST_POSDEF));
    qpPlane_->setPrintLevel(qpOASES::PL_HIGH);
    data_optimal_solution_.resize(nV_);

    lb_.resize(nV_);
    ub_.resize(nV_);

    lb_ << -1, -1, -1, -100, -1000000;
    ub_ << 1, 1, 1, 1000000, 1000000 ;



    E_.resize(5,5);
    E_.setZero();
    E_.block(0,0,4,4).setIdentity();
    H_.resize(5,5);
//    H_ = E_.transpose() * E_ ;
//    H_.setIdentity();
    g_.resize(5);
}



void PlaneSolver::setCost(const Eigen::VectorXd& f){
    Eigen::VectorXd vecI;

//    g_.setZero();
    Eigen::MatrixXd H1,H2,H3,E1;
    Eigen::VectorXd g1,g2,g3;
    H1.resize(5,5);
    H2.resize(5,5);
    H3.resize(5,5);
    g1.resize(5);
    g2.resize(5);
    g3.resize(5);

    E1.resize(4,5);
    E1.setZero();
    E1.block(0,0,4,4).setIdentity();
    H1 = E1.transpose()*E1;
    g1 = - E1.transpose()*f.segment(0,4);


    H2.setZero();
    g2 << 0,0,0,0,-1;

    H3.setZero();
    H3(4,4) = 0.2;
    g3.setZero();
    // For linearized
    H_ = 0.01*H1 + 0.1*H2 + H3;
    g_ = 0.01*g1 + 0.1*g2 + g3;
    // For non linear
//    H_ = 0.1*H1 + 0.1*H2 + H3;
//    g_ = 0.1*g1 + 0.1*g2 + g3;
}


void PlaneSolver::setCstMatrix(const Eigen::MatrixXd &robotPartielVertices,
                               const Eigen::MatrixXd &obsPartielVertices,
                               const Eigen::MatrixXd &dataPlanePrecedent){

    int rcols = robotPartielVertices.cols(), pcols = obsPartielVertices.cols();
    // A_.resize(rcols + pcols +1, 5);
    A_.resize(rcols + pcols +3, 5);

    //  Additional non vertical plane: 
    A_.setZero();
    //  min -d + alpha*d² + ||(ak,bk) - (ak,bk)^p||²
    // dsafe  << -rk.ak + bk
    // -dsafe << pk.ak - bk - dk
    // 1-e << ak^p.ak << 1
    // Plane close to robot

//    std::cout <<" rcols : \n" <<rcols <<std::endl;
//    std::cout <<" pcols : \n" <<pcols <<std::endl ;
    // lbA_.resize(rcols+pcols+1);
    // ubA_.resize(rcols+pcols+1);

    //  Additional non vertical plane: 
    lbA_.resize(rcols+pcols+3);
    ubA_.resize(rcols+pcols+3);
    for (int j(0); j < rcols ; j ++ ){
        A_.block(j,0,1,5) << -robotPartielVertices.block(0,j,3,1).transpose() , 1, -1;
    }
    for (int i(0); i < pcols; i++ ){
         A_.block(i+rcols,0,1,5) << obsPartielVertices.block(0,i,3,1).transpose(), -1, 0;
//        std::cout <<" obs partiel vertices:\n " << obsPartielVertices.block(0,i,3,1).transpose() << std::endl;
    }


    A_.block(rcols + pcols,0,1,5) <<  dataPlanePrecedent.transpose(),0,0 ;
    // Additional projection constraint
    A_.block(rcols + pcols+1,0,1,5) << dataPlanePrecedent(0),dataPlanePrecedent(1),0,0,0 ;
    A_.block(rcols + pcols+2,0,1,5) << dataPlanePrecedent(0),dataPlanePrecedent(1),0,0,0 ;

    // Additional non vertical constraint 
    // A_.block(rcols + pcols+1,0,1,5) << 0,1,0,0,0 ;

//    A_.block(rcols + pcols, 0, 1 ,5) << 0, 0, 0, 0, 0 ;
    // dsafe_ = dsafe_ /2 ;
    // dsafe_ = 0.1;
    dsafe_ = 0.05;
    lbA_.setConstant(dsafe_);
    //  lbA_.setZero();
    lbA_(rcols + pcols) = 0.9;
    lbA_.tail(2) << -0.3, -0.3;
    // lbA_ << dsafe_,dsafe_,dsafe_,dsafe_,dsafe_,dsafe_, 0.9;

//    lbA_.tail(1) <<  1-0.1;

    ubA_.setConstant(10000000);
    // ubA_(rcols + pcols +1) = 1;
    ubA_(rcols + pcols) = 1;
    ubA_.tail(2) << 0.7,0.7;
 
}


bool PlaneSolver::solve(){
    qpOASES::returnValue ret;
    qpOASES::PrintLevel printlevel;
    printlevel = qpOASES::PL_HIGH;
    qpOASES::int_t nWSR = 100000;
    std::ofstream myfile;
    myfile.open ("/home/zheng/Bureau/problem.txt");
//     if(!qpoases_initialized)
//           {
        // Initialise the problem, once it has found a solution, we can hotstart
    int cstNbr;
    cstNbr = lbA_.size();

    qpOASES::QProblem example( 5,cstNbr);
    qpOASES::Options options_;
    options_.numRefinementSteps = 10;
    options_.setToMPC();
    example.setPrintLevel(qpOASES::PL_HIGH);
//    print();

    ret = example.init( H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(), nWSR );
    example.getPrimalSolution(data_optimal_solution_.data());
//    qpPlane_.reset(new qpOASES::QProblem(nV_,cstNbr, qpOASES::HST_POSDEF));

//    options_.setToMPC();
//    qpPlane_->setOptions(options_);
//    qpPlane_->setPrintLevel( printlevel);

    myfile << "H_ \n " <<  H_ << '\n';
    myfile << "g_ \n " << g_ << '\n';
    myfile << "A_ \n " << A_ << '\n';
    myfile << "lbA_ \n " << lbA_ << '\n';
    myfile << "ubA_ \n " << ubA_ << '\n';
    myfile << "lb_ \n " << lb_ << '\n';
    myfile << "ub_ \n " << ub_ << '\n';
//    ret = qpPlane_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);


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
//               qpPlane_->getPrimalSolution(data_optimal_solution_.data());

               myfile << "data_optimal_solution_ \n " << data_optimal_solution_<< '\n';
                return true;
          }else{
              std::cout <<"Plane optimization failed :\n " <<  std::endl;
              std::cout <<"return value is :  " <<  ret  << '\n';
              std::cout <<"qpProblem status is : " << qpPlane_->getStatus()<<'\n';
              std::cout <<"qpProblem obj is : " << qpPlane_->getObjVal()<<'\n';
              std::cout <<"qpProblem cst number is : " << qpPlane_->getNC()<<'\n';
              std::cout <<"qpProblem variable number is : " << qpPlane_->getNV()<<'\n';
              std::cout <<"qpProblem is initialized ? " << qpPlane_->isInitialised()<<'\n';
              std::cout <<"qpProblem is solved ? " << qpPlane_->isSolved()<<'\n';
              std::cout <<"qpProblem is infeasible ? " << qpPlane_->isInfeasible()<<'\n';
              std::cout <<"qpProblem is Unbounded ? " << qpPlane_->isUnbounded()<<'\n';
              std::cout <<" print level : " << qpPlane_->getPrintLevel() << std::endl;

              myfile << "data_optimal_solution_ \n " << data_optimal_solution_ << '\n';

              return false;
          }

}

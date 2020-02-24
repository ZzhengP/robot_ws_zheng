#include "qpOASES.hpp"
#include "Eigen/Core"
#include "iostream"

using namespace qpOASES;

int main (){

//    real_t H[5*5] = {   1,   0,   0,   0,   0,
//                        0,   1,   0,   0,   0,
//                        0,   0,   1,   0,   0,
//                        0,   0,   0,   1,   0,
//                        0,   0,   0,   0, 0.1 };
//   printf( "\ntest = [ %e, %e]; ",H[0], H[6] );

//    real_t A[7*5] = {  -0.470814,    -0.257208,    -0.517853,            1,            0,
//                       0.022332,    0.0120235,     -0.64798,            1,            0,
//                      -0.470814,    -0.257208,    -0.517853,            1,            0,
//                       0.022332,    0.0120235,     -0.64798,            1,            0,
//                            0.5,           0 ,    0.517842,           -1,           -1,
//                            0.5,            0,     0.517842,           -1,           -1,
//                   0.112746,    -0.993624, -4.05584e-05 ,            0,            0};
//    real_t g[5] = {   -0.112746,
//                      0.993624,
//                   4.05584e-05,
//                     0.0730771,
//                             -1};
//    real_t lb[5] = {     -1,
//                         -1,
//                         -1,
//                       -100,
//                     -1e+06 };
//    real_t ub[5] = {    1,
//                        1,
//                        1,
//                    1e+06,
//                    1e+06};
//    real_t lbA[7] = {  0.1,
//                       0.1,
//                       0.1,
//                       0.1,
//                       0.1,
//                       0.1,
//                       0.9 };
//    real_t ubA[7] = { 1e+07,
//                      1e+07,
//                      1e+07,
//                      1e+07,
//                      1e+07,
//                      1e+07,
//                          1};

    
    Eigen::VectorXd g_, lbA_, ubA_, lb_, ub_ ;
    Eigen::MatrixXd E_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    H_.resize(5,5);
    H_ <<   1,    0,    0,    0,    0,
            0,    1,    0,    0,    0,
            0,    0,    1,    0,    0,
            0,    0,    0,    1,    0,
            0,    0,    0,    0,  0.1;

     g_.resize(5);
     g_ << -0.112746,
             0.993624,
          4.05584e-05,
            0.0730771,
                    -1;

     lbA_.resize(7);
     lbA_ << 0.1,
             0.1,
             0.1,
             0.1,
             0.1,
             0.1,
             0.9;
     ubA_.resize(7);

     ubA_ << 1e+07,
             1e+07,
             1e+07,
             1e+07,
             1e+07,
             1e+07,
                 1;

     lb_.resize(5);
     ub_.resize(5);
     lb_ <<  -1,
             -1,
             -1,
           -100,
         -1e+06;
     ub_ <<  1,
             1,
             1,
         1e+06,
         1e+06;
     A_.resize(7,5);
     A_ <<  -0.470814,    -0.257208,    -0.517853,            1,            0,
             0.022332,    0.0120235,     -0.64798,            1,            0,
            -0.470814,    -0.257208,    -0.517853,            1,            0,
             0.022332,    0.0120235,     -0.64798,            1,            0,
                  0.5,           0 ,    0.517842,           -1,           -1,
                  0.5,            0,     0.517842,           -1,           -1,
         0.112746,    -0.993624, -4.05584e-05 ,            0,            0;
    QProblem example( 5,7, HST_POSDEF);
    Options options_;
    qpOASES::returnValue ret;

    options_.numRefinementSteps = 10;
    options_.setToMPC();
    int nWSR = 1000000;
    example.setPrintLevel(PL_HIGH);

    Eigen::VectorXd solution;
    solution.resize(5);
    ret = example.init( H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(), nWSR );
   example.getPrimalSolution( solution.data() );

   std::cout <<" solution : \n" << solution << std::endl;


    example.setPrintLevel(PL_HIGH);
    real_t xOpt[5];

//        printf( "\nxOpt = [ %e, %e,%e, %e,%e ];  objVal = %e\n\n", xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],example.getObjVal() );
    if(ret == qpOASES::SUCCESSFUL_RETURN){
  // Get the solution
//               qpPlane_->getPrimalSolution(data_optimal_solution_.data());

         std::cout <<"ok " << '\n';
    }else{
        std::cout <<"failed " << '\n';

    }

    example.printOptions();
    return 0;
}

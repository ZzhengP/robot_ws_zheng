#include "constraint/RosGenericCst.h"


generic_cst::generic_cst(int ndof, int N):NrOfDeg_(ndof),N_Prediction_(N){

             lb_.resize(ndof*N);
             ub_.resize(ndof*N);
             A_.resize(ndof*N,ndof*N);
             min_.resize(ndof*N);
             max_.resize(ndof*N);
}

generic_cst::~generic_cst()
{

}

void generic_cst::setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px)
{

}

void generic_cst::setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px)
{

}

void generic_cst::setConstraintMatrix(Eigen::MatrixXd Pu)
{

}

void generic_cst::update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu){

}

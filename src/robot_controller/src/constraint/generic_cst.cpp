#include "constraint/generic_cst.h"


generic_cst::generic_cst(int ndof, int N):NrOfDeg_(ndof),N_Prediction_(N){

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

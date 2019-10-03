#include "constraint/generic_cst.h"



class  joint_acc_cst : public generic_cst
{

public:

    joint_acc_cst(int ndof, int N) : generic_cst(ndof, N) {}

     ~joint_acc_cst(){}

    void setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px_dq){
         lb_ = min_ ;
    }

    void setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px_dq){
         ub_ = max_ ;
    }

    void setConstraintMatrix(Eigen::MatrixXd C){
         A_ = C ;
    }

    void setLimit(Eigen::VectorXd min, Eigen::VectorXd max){
         min_ = min ;
         max_ = max;
    }

protected:

    Eigen::VectorXd min_, max_ ;

};

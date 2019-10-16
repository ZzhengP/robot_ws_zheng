#include "constraint/RosGenericCst.h"

class  jnt_acc_cst : public generic_cst
{

public:

    jnt_acc_cst(int ndof, int N) : generic_cst(ndof, N) {
    }

     ~jnt_acc_cst(){}

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
        std::cout <<" min_ size : " << min_.size() << std::endl;
        std::cout <<" max_ size : " << max_.size() << std::endl;
        std::cout <<" min size : " << min.size() << std::endl;
        std::cout <<" min size : " << max.size() << std::endl;

         min_ = min ;
         max_ = max;
    }

    void update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu){

    };
protected:


};

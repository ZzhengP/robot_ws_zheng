#include "Eigen/Core"
#include "iostream"
#include "unsupported/Eigen/MatrixFunctions"
#include "Eigen/Dense"
#include "constraint/RosGenericCst.h"

class  jnt_vel_cst : public generic_cst {
public:
    jnt_vel_cst(int ndof, int N) : generic_cst (ndof, N){}


    void setLimit(Eigen::VectorXd min, Eigen::VectorXd max);

    void setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);
    void setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);
    void setConstraintMatrix(Eigen::MatrixXd Pu);
    void update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu);

private:

};

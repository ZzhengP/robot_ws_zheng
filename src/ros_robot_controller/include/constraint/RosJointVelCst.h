#include "Eigen/Core"
#include "iostream"
#include "unsupported/Eigen/MatrixFunctions"
#include "Eigen/Dense"
#include "constraint/RosGenericCst.h"

class  jntVelCst : public generic_cst {
public:
    jntVelCst(int ndof, int N) : generic_cst (ndof, N){}


    void setLimit(Eigen::VectorXd min, Eigen::VectorXd max);

    void setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);
    void setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);
    void setConstraintMatrix(Eigen::MatrixXd Pu);
    void update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu);

private:

};

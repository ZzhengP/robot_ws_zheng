#include "Eigen/Core"
#include "iostream"
#include "Eigen/Dense"
#include "constraint/RosGenericCst.h"


class jntPosCst : public generic_cst
{
public:

    jntPosCst(int ndof, int n) : generic_cst (ndof, n) {}

    void setLimit(Eigen::VectorXd min, Eigen::VectorXd max);

    void setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);

    void setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);

    void setConstraintMatrix(Eigen::MatrixXd Pu);

    void update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu);
protected:


};

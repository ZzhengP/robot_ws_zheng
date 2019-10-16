#include "Eigen/Core"
#include "iostream"
#include "Eigen/Dense"
#include "constraint/RosGenericCst.h"


class jnt_pos_cst : public generic_cst
{
public:

    jnt_pos_cst(int ndof, int n) : generic_cst (ndof, n) {}

    void setLimit(Eigen::VectorXd min, Eigen::VectorXd max);

    void setLowerBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);

    void setUpperBound(Eigen::VectorXd robot_state, Eigen::MatrixXd Px);

    void setConstraintMatrix(Eigen::MatrixXd Pu);

    void update(Eigen::VectorXd robot_state, Eigen::MatrixXd Px, Eigen::MatrixXd Pu);
protected:


};

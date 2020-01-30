#include "Eigen/Core"
#include "iostream"
#include "unsupported/Eigen/MatrixFunctions"
#include "Eigen/Dense"
#include "constraint/RosGenericCst.h"

class  jntVelCst : public generic_cst {
public:
    jntVelCst(int ndof, int N, double dt, std::string name) : generic_cst (ndof, N,dt, name){
        cstData_.name_ = name;
        cstData_.upBound_.resize(ndof*N_);
        cstData_.lowBound_.resize(ndof*N_);
        cstData_.cstMatrix_.resize(ndof*N_,ndof*N_);
    }


    void setLimit(const Eigen::VectorXd &min,const Eigen::VectorXd &max);

    void setLowerBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px);
    void setUpperBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px);
    void setConstraintMatrix(const Eigen::MatrixXd &Pu);
    void update(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px, const Eigen::MatrixXd &Pu);

private:

};

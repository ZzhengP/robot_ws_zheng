#include "constraint/RosGenericCst.h"


class cartVelCst: public generic_cst
{
public:

    cartVelCst(int ndof, int N,double dt, std::string name): generic_cst(ndof, N, dt, name ) {
        min_.resize(3*N_);
        max_.resize(3*N_);
        lbA_.resize(3*N_);
        ubA_.resize(3*N_);
        A_.resize(3*N_,ndof*N_);
        cstData_.name_ = name;
        cstData_.upBound_.resize(3*N_);
        cstData_.lowBound_.resize(3*N_);
        cstData_.cstMatrix_.resize(3*N_,ndof*N_);
    }


    void setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){

        min_ = min;
        max_ = max;

    }

    void setLowerBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Jacobian){

       lbA_ = (min_ - Jacobian*robot_state)/dt_;
    }
    void setUpperBound(const Eigen::VectorXd &robot_state,const Eigen::MatrixXd &Jacobian){

       ubA_ = (max_ - Jacobian*robot_state)/dt_;
    }
    void setConstraintMatrix(const Eigen::MatrixXd &Jacobian){
        A_ =  Jacobian;
    }
    void update(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Px, const Eigen::MatrixXd &Jacobian){

        setLowerBound( robot_state, Jacobian);
        setUpperBound(robot_state, Jacobian);
        setConstraintMatrix(Jacobian);

    }
protected:


};


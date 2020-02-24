#include "constraint/RosGenericCst.h"
#include <fstream>

class cartVelCst: public generic_cst
{
public:

    cartVelCst(int ndof, int N,double dt, std::string name, Eigen::MatrixXd Px, Eigen::MatrixXd Pu,Eigen::MatrixXd Pxdq, Eigen::MatrixXd Pudq)
        : generic_cst(ndof, N, dt, name, Px, Pu ) {

        Pxdq_ = Pxdq;
        Pudq_ = Pudq;
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

    void setLowerBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &JacobianHorizon,
                       const Eigen::MatrixXd & JacobianDotHorizon,Eigen::VectorXd eeVelPrecedent,
                       const Eigen::VectorXd &qHorizonPrecedent,const Eigen::VectorXd &dotqHorizonPrecedent ){

        Eigen::MatrixXd JacLinearHorizon_, JacDotLinearHorizon_;
        JacLinearHorizon_.resize(3*N_,N_*7);
        JacDotLinearHorizon_.resize(3*N_,N_*7);
        JacLinearHorizon_.setZero();
        JacDotLinearHorizon_.setZero();
        for (int k(0); k<N_;k++){
            JacLinearHorizon_.block(3*k,7*k,3,7) = JacobianHorizon.block(6*k,7*k,3,7);
            JacDotLinearHorizon_.block(3*k,7*k,3,7) = JacobianDotHorizon.block(6*k,7*k,3,7);

        }
       lbA_ = min_ - eeVelPrecedent - JacDotLinearHorizon_*(Px_*robot_state - qHorizonPrecedent)
                             - JacLinearHorizon_*(Pxdq_*robot_state - dotqHorizonPrecedent);
//        lbA_ = min_ - eeVelPrecedent - JacLinearHorizon_*(Pxdq_*robot_state - dotqHorizonPrecedent);
       cstData_.lowBound_ = lbA_;

    }
    void setUpperBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &JacobianHorizon,
                       const Eigen::MatrixXd & JacobianDotHorizon,Eigen::VectorXd eeVelPrecedent,
                       const Eigen::VectorXd &qHorizonPrecedent,const Eigen::VectorXd &dotqHorizonPrecedent){
        Eigen::MatrixXd JacLinearHorizon_, JacDotLinearHorizon_;
        JacLinearHorizon_.resize(3*N_,N_*7);
        JacDotLinearHorizon_.resize(3*N_,N_*7);
        JacLinearHorizon_.setZero();
        JacDotLinearHorizon_.setZero();
        for (int k(0); k<N_;k++){
            JacLinearHorizon_.block(3*k,7*k,3,7) = JacobianHorizon.block(6*k,7*k,3,7);
            JacDotLinearHorizon_.block(3*k,7*k,3,7) = JacobianDotHorizon.block(6*k,7*k,3,7);

        }
       ubA_ = max_ - eeVelPrecedent - JacDotLinearHorizon_*(Px_*robot_state - qHorizonPrecedent)
                   - JacLinearHorizon_*(Pxdq_*robot_state - dotqHorizonPrecedent);
//        ubA_ = max_ - eeVelPrecedent - JacLinearHorizon_*(Pxdq_*robot_state - dotqHorizonPrecedent);
       cstData_.upBound_ = ubA_;

    }
    void setConstraintMatrix(const Eigen::MatrixXd &JacobianHorizon, const Eigen::MatrixXd & JacobianDotHorizon){
        Eigen::MatrixXd JacLinearHorizon_, JacDotLinearHorizon_;
        JacLinearHorizon_.resize(3*N_,N_*7);
        JacDotLinearHorizon_.resize(3*N_,N_*7);
        JacLinearHorizon_.setZero();
        JacDotLinearHorizon_.setZero();

        for (int k(0); k<N_;k++){
            JacLinearHorizon_.block(3*k,7*k,3,7) = JacobianHorizon.block(6*k,7*k,3,7);
            JacDotLinearHorizon_.block(3*k,7*k,3,7) = JacobianDotHorizon.block(6*k,7*k,3,7);

        }
        A_ =  JacDotLinearHorizon_*Pu_ + JacLinearHorizon_*Pudq_;
//        A_ =  JacLinearHorizon_*Pudq_;
        cstData_.cstMatrix_ = A_;

       std::ofstream myfile;
       myfile.open ("/home/zheng/Bureau/carteVelContraint.txt");
       myfile << "A_ \n " <<  A_ << '\n';
       myfile << "lbA_ \n " << lbA_ << '\n';
       myfile << "ubA_ \n " << ubA_ << '\n';

       myfile.close();
    }

protected:
    Eigen::MatrixXd Pxdq_, Pudq_;
};


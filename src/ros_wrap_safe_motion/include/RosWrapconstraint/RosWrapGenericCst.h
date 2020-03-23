#ifndef RosWrapsepGenericCst_H
#define RosWrapsepGenericCst_H
#pragma once
#include "iostream"
#include <Eigen/Dense>
#include <Eigen/Core>


struct constraintData{
       std::string name_;
       Eigen::VectorXd lowBound_, upBound_;
       Eigen::MatrixXd cstMatrix_;
       void print(){
           std::cout <<" constraint name :\n " << name_ << std::endl;
           std::cout <<" constraint data low bound :\n" << lowBound_ << std::endl;
           std::cout <<" constraint data upper bound :\n" << upBound_ << std::endl;
           std::cout <<" constraint data matrix :\n" << cstMatrix_ << std::endl;

       }
};

/**
 * @brief The generic constraint class
 * \f$ lbA <= Ax <= ubA   \f$
 * Which is defined by a lower bound, upper bound and constraint matrix
 */
class  GenericCst{
public:
    /**
      * @brief basic construtor
      * @param ndof
      * @param N
      * @param dt
      */
     GenericCst(int ndof, int N, double dt, std::string name, Eigen::MatrixXd Px, Eigen::MatrixXd Pu);

     virtual ~GenericCst();

     /**
      * @brief setLowerBound: lbA
      * @param robot_state
      * @param Px
      */
     virtual void setLowerBound(const Eigen::VectorXd &robot_state){};

     virtual void setLowerBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Jacobian){};

     /**
      * @brief setUpperBound
      * @param robot_state
      * @param Px
      */
     virtual void setUpperBound(const Eigen::VectorXd &robot_state){};
     virtual void setUpperBound(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Jacobian){};
     /**
      * @brief setConstraintMatrix
      * @param Pu
      */
     virtual void setConstraintMatrix() {};
     virtual void setConstraintMatrix(const Eigen::MatrixXd &Jacobian) {};
     /**
      * @brief update constraint state : lbA, ubA, A
      * @param robot_state
      * @param Px
      * @param Pu
      */
     virtual void update(const Eigen::VectorXd &robot_state){};

      virtual void update(const Eigen::VectorXd &robot_state, const Eigen::MatrixXd &Jacobian){};
     /**
      * @brief getLowerBound
      * @return lbA_
      */
     Eigen::VectorXd getLowerBound(){
            return lbA_;
     }

     /**
      * @brief getUpperBound
      * @return ubA_
      */
     Eigen::VectorXd getUpperBound(){
            return ubA_;
     }

     /**
      * @brief getConstraintMatrix
      * @return A_
      */
     Eigen::MatrixXd getConstraintMatrix(){
            return A_;
     }

     /**
      * @brief getConstraintData (name, lbA, ubA, A)
      * @return cstData_
      */
     constraintData getConstraintData(){
            return cstData_;
     }


protected:
     Eigen::VectorXd lbA_, ubA_, min_, max_;
     Eigen::MatrixXd A_, Px_, Pu_;
     double dt_;
     int n_, N_ ;
     std::string cst_name_ ;

     constraintData cstData_;
};

#endif
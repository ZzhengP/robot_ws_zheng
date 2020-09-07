#ifndef ROSWRAPHUMANMOTION_H
#define ROSWRAPHUMANMOTION_H
#pragma once 
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include "map"
#include "memory"
#include "qpOASES.hpp"

 class HumanMotion
 {

 public:
     HumanMotion(const int & ndof, const Eigen::MatrixXd & DHparam,
                 const double & lc1, const double & lc2, const double & lu, const double & lf, const double & lh,
                 const double & m1, const double & m2 );

     void print() const;

     Eigen::Matrix4d getHandTransformation() const{
        
        return handTransformMatrix_;
     }

     std::vector<Eigen::Matrix4d> getTransformationMatrixList(){
         return TransformationMatrixList_;
     }
     std::vector<Eigen::Matrix4d> getTransformationToBaseMatrixList(){
         return TransformationMatrixToBaseList_;
     }
     std::vector<Eigen::Vector3d> getStackedLinearVelocity(){
         return stacked_linear_velocity_;

     }

     std::vector<Eigen::Vector3d> getStackedAngularVelocity(){
         return stacked_angular_velocity_;
         
     }

     Eigen::MatrixXd getJacobian(){
         return Jacobian_;
     }
     Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d & angularVel){
         Eigen::Matrix3d skewMatrix;
         
         skewMatrix << 0, -angularVel(2), angularVel(1),
                       angularVel(2), 0, -angularVel(0),
                       -angularVel(1), angularVel(0), 0;

         return skewMatrix;

     }


     void computeForwardKinematic(const Eigen::VectorXd &theta){

          thetaList_.setZero();  
          thetaList_ = theta + DHparam_.col(3);

          for (int it(0); it < ndof_; it ++){

             TransformationMatrixList_[it] << cos(thetaList_(it)), -sin(thetaList_(it)), 0 , aList_(it),
				 							 sin(thetaList_(it))*cos(alphaList_(it)), cos(thetaList_(it))*cos(alphaList_(it)), -sin(alphaList_(it)), -sin(alphaList_(it))*dList_(it),
											 sin(thetaList_(it))*sin(alphaList_(it)), cos(thetaList_(it))*sin(alphaList_(it)), cos(alphaList_(it)), cos(alphaList_(it))*dList_(it),
											 0,0,0,1 ;
          } 

     }


     void computeTransformationMatrixToBaseList(){

        Eigen::Matrix4d MatrixToBase;
        MatrixToBase.setIdentity();
         for (int it(0); it < ndof_+1; it ++){
                MatrixToBase = MatrixToBase*TransformationMatrixList_[it];
 				TransformationMatrixToBaseList_[it] = MatrixToBase;
			}	

       handTransformMatrix_ = TransformationMatrixToBaseList_[ndof_];
     }
     void computeStackedVelocity(const Eigen::VectorXd& dtheta);

     void computeHandVelocity(){

            Eigen::Matrix3d Rot4h;
            Eigen::Vector3d P4h;
            Rot4h = TransformationMatrixList_[4].block(0,0,3,3);
			P4h = TransformationMatrixList_[4].block(0,3,3,1) ;
  			hand_angular_vel_ = Rot4h.transpose()*stacked_angular_velocity_[3];
			hand_linear_vel_ =  Rot4h.transpose()*(stacked_linear_velocity_[3] + skewSymmetricMatrix(stacked_angular_velocity_[3])*P4h);


     }
     
     Eigen::Vector3d getHandLinearVelocity(){
         return hand_linear_vel_;
     }
     void computeJacobian();
     Eigen::MatrixXd computeJacobian(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList);   
     void computeJacobianC1();
     void computeJacobianC2();
     Eigen::MatrixXd computeJacobianC1(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList);   
     Eigen::MatrixXd computeJacobianC2(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList);   


     Eigen::VectorXd computeForwardDynamic(const Eigen::VectorXd & dq,const Eigen::VectorXd & ddq);
     Eigen::VectorXd computeCoriolisGravity(const Eigen::VectorXd & dq);

     Eigen::MatrixXd computeInertialMass(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList, const Eigen::MatrixXd & Jacobian_c1, const Eigen::MatrixXd & jacobian_c2);
     ~HumanMotion(){
         
     }

 private:

     /**
      * @brief human dof = 4
      * 
      */
     const int ndof_;
     const double lc1_, lc2_, lu_, lf_, lh_, m1_, m2_;
     /**
      * @brief D.H parameters 
      * 
      */
     Eigen::MatrixXd DHparam_;
     Eigen::VectorXd aList_;
     Eigen::VectorXd alphaList_;
     Eigen::VectorXd dList_;
     Eigen::VectorXd thetaList_;
     /**
      * @brief Transformation matrix of hand 
      * 
      */
     Eigen::Matrix4d handTransformMatrix_, T3C1_, T4C2_;

     /**
      * @brief Dimension of ndof +1 : the n + 1 frame is the hand fixed frame 
      * 
      */
     std::vector<Eigen::Matrix4d> TransformationMatrixList_;
     std::vector<Eigen::Matrix4d> TransformationMatrixToBaseList_; 
     
     Eigen::Vector3d handPosition_;
     Eigen::Vector3d hand_linear_vel_, hand_angular_vel_;  

     /**
      * @brief linear velocity of frame i+1 expressed in coordinate i+1
      * 
      */
     std::vector<Eigen::Vector3d> stacked_linear_velocity_ ;
     std::vector<Eigen::Vector3d> stacked_angular_velocity_ ;
     std::vector<Eigen::Vector3d> stacked_linear_acc_ ;
     std::vector<Eigen::Vector3d> stacked_angular_acc_ ;

     Eigen::VectorXd q_, dq_, ddq_ ;
     Eigen::MatrixXd Jacobian_, Jacobian_C1_, Jacobian_C2_;

     Eigen::Matrix3d Ic1_, Ic2_; 
 };

#endif
#ifndef HUMANMODEL_H
#define HUMANMODEL_H
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
class humanModel
{

    public:

        humanModel(const int& ndof, const std::vector<Eigen::Matrix4d>& M_home);

        ~humanModel(){}


        // Rigid motion related fonction 
        bool nearZero(const double& val);

 
        /**
         * @brief return the skew matrix presentation of angular velocity 
         * 
         * @param angular velocity
         * @return skew matrix 
         */
        Eigen::MatrixXd VecToso3(const Eigen::Vector3d& omega);

        /**
         * @brief return the vector presentation of angular velocity skew matrix 
         * 
         * @param omega_matrix 
         * @return angular velocity
         */
        Eigen::Vector3d so3ToVec(const Eigen::Matrix3d& omega_matrix);

        /**
         * @brief Translates an exponential rotation into it's individual components
         * 
         * @param Exponential rotation (rotation matrix in terms of a rotation axis
	     *		  and the angle of rotation)
         * @return The axis and angle of rotation as [x, y, z, theta]
         */
        Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3);


        /**
         * @brief Translates an exponential rotation into a rotation matrix
         * 
         * @param  exponenential representation of a rotation 
         * @return Rotation matrix
         */
        Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat);

        /**
         * @brief Computes the matrix logarithm of a rotation matrix
         * 
         * @param Rotation matrix 
         * @return matrix logarithm of a rotation
         */
        Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R);

        /**
         * @brief Combines a rotation matrix and position vector into a single
	     * 				Special Euclidian Group (SE3) homogeneous transformation matrix
         
         * @param R 
         * @param p 
         * @return  Matrix of T = [ [R, p],
	     *						    [0, 1] ]
         */
        Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);


        std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T);
        
        /**
         * @brief Translates a spatial velocity vector into a transformation matrix
         * 
         * @param spatial  velocity 
         * @return spatial  velocity  transformation matrix 
         */
        Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V);


        /**
         * @brief Translates a spatial velocity transformation matrix  into a spatial velocity vector
         * 
         * @param spatial velocity transformation matrix 
         * @return [angular velocity, linear velocity ]
         */
        Eigen::VectorXd se3ToVec(const Eigen::MatrixXd& T);

        /**
         * @brief adjoint representation of a transformation matrix
	     *	 Used to change the frame of reference for spatial velocity vectors
         * 
         * @param 4x4 Transformation matrix SE(3)
         * @return 6x6 Adjoint Representation of the matrix
         */
        Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T);


        Eigen::MatrixXd TransInv(const Eigen::MatrixXd& transform);

        Eigen::MatrixXd RotInv(const Eigen::MatrixXd& rotMatrix) {
		    return rotMatrix.transpose();
	    }
        /**
         * @brief Rotation expanded for screw axis
         * 
         * @param se3 matrix representation of exponential coordinates (transformation matrix) 
         * @return 6x6 Matrix representing the rotation
         */
        Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat);

        /**
         * @brief 
         * 
         * @param T 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T) ;


        /**
         * @brief Compute end effector frame (used for current spatial position calculation)
         * 
         * @param  Home configuration (position and orientation) of end-effector
	     *		   The joint screw axes in the space frame when the manipulator
	     *             is at the home position
	     * 		   A list of joint coordinates.
   
         * @return Transfomation matrix representing the end-effector frame when the joints are
	     *				at the specified coordinates
         */
        Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList);

        Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList);



        Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList);

        Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList);

        Eigen::MatrixXd getTransformation(const Eigen::MatrixXd& M, const Eigen::VectorXd& thetaList);
        Eigen::MatrixXd getRelativeTransformation(const Eigen::MatrixXd& Ti, const Eigen::MatrixXd& Tj);

        Eigen::VectorXd computeVelocity(const Eigen::VectorXd& V, const Eigen::MatrixXd& T);


        bool IKinSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T, Eigen::VectorXd& thetalist, double eomg, double ev);
        std::vector<Eigen::VectorXd> computeOccupancy(const Eigen::VectorXd& q_init, const Eigen::VectorXd& dq_init,const Eigen::VectorXd& ddq_init, const int & N, const double & Vmax);

    private:

        Eigen::MatrixXd  S_list_, B_list_  ;
        Eigen::Matrix4d M_tool_;
        std::vector<Eigen::Matrix4d> M_ ;        
        int ndof_;


};

#endif
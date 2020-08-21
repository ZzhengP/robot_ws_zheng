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
 private:
     /* data */
 public:
     HumanMotion(const int& ndof,const std::map<std::string, Eigen::Matrix4d> & M_joint , 
                 const std::map<std::string, Eigen::Matrix4d> & M_CoM,
                 const Eigen::MatrixXd &S_list );
     ~HumanMotion(){
         
     }



    // Rigid motion related fonction 
    bool nearZero(const double& val);
    /**
     * @brief return the skew matrix presentation of angular velocity 
     * 
     * @param angular velocity
     * @return skew matrix 
     */
    Eigen::MatrixXd VecToso3(const Eigen::Vector3d& omega);

    /*
    * Function: Calculate the 6x6 matrix [adV] of the given 6-vector
    * Input: Eigen::VectorXd (6x1)
    * Output: Eigen::MatrixXd (6x6)
    * Note: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2
    */
    Eigen::MatrixXd ad(Eigen::VectorXd);


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

        

    /*
    * Function: Computes inverse kinematics in the space frame for an open chain robot
    * Inputs:
    *	Slist: The joint screw axes in the space frame when the
    *         manipulator is at the home position, in the format of a
    *         matrix with axes as the columns
    *	M: The home configuration of the end-effector
    *	T: The desired end-effector configuration Tsd
    *	thetalist[in][out]: An initial guess and result output of joint angles that are close to
    *         satisfying Tsd
    *	emog: A small positive tolerance on the end-effector orientation
    *        error. The returned joint angles must give an end-effector
    *        orientation error less than eomg
    *	ev: A small positive tolerance on the end-effector linear position
    *      error. The returned joint angles must give an end-effector
    *      position error less than ev
    * Outputs:
    *	success: A logical value where TRUE means that the function found
    *           a solution and FALSE means that it ran through the set
    *           number of maximum iterations without finding a solution
    *           within the tolerances eomg and ev.
    *	thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
    */
        bool IKinSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T, Eigen::VectorXd& thetalist, double eomg, double ev);
   
        /**
         * @brief Compute inverse kinematic for given constraint
         * 
         * @param Slist 
         * @param M 
         * @param T 
         * @param thetalist 
         * @return true 
         * @return false 
         */
        bool IKNumericalSolver(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd & M, const Eigen::Vector3d& xd, Eigen::Vector4d & thetalist);
        //------------------------------------- Dynamic -------------------------------------------------------------

        /* 
        * Function: This function uses forward-backward Newton-Euler iterations to solve the 
        * equation:
        * taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
         *                               + g(thetalist) + Jtr(thetalist) * Ftip
        * Inputs:
        *  thetalist: n-vector of joint variables
        *  dthetalist: n-vector of joint rates
        *  ddthetalist: n-vector of joint accelerations
        *  g: Gravity vector g
        *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
        *  Mlist: List of link frames {i} relative to {i-1} at the home position
        *  Glist: Spatial inertia matrices Gi of the links
        *  Slist: Screw axes Si of the joints in a space frame, in the format
        *         of a matrix with the screw axes as the columns.
        * 
        * Outputs:
        *  taulist: The n-vector of required joint forces/torques
        * 
        */
        Eigen::VectorXd InverseDynamics(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, 
                                         const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, 
                                         const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        /* 
        * Function: This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and 
        *   ddthetalist = 0. The purpose is to calculate one important term in the dynamics equation       
        * Inputs:
        *  thetalist: n-vector of joint variables
        *  g: Gravity vector g
        *  Mlist: List of link frames {i} relative to {i-1} at the home position
        *  Glist: Spatial inertia matrices Gi of the links
        *  Slist: Screw axes Si of the joints in a space frame, in the format
        *         of a matrix with the screw axes as the columns.
        * 
        * Outputs:
        *  grav: The 3-vector showing the effect force of gravity to the dynamics
        * 
        */
        Eigen::VectorXd GravityForces(const Eigen::VectorXd&, const Eigen::VectorXd&,
                                      const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);


        /* 
        * Function: This function calls InverseDynamics n times, each time passing a 
        * ddthetalist vector with a single element equal to one and all other 
        * inputs set to zero. Each call of InverseDynamics generates a single 
        * column, and these columns are assembled to create the inertia matrix.       
        *
        * Inputs:
        *  thetalist: n-vector of joint variables
        *  Mlist: List of link frames {i} relative to {i-1} at the home position
        *  Glist: Spatial inertia matrices Gi of the links
        *  Slist: Screw axes Si of the joints in a space frame, in the format
        *         of a matrix with the screw axes as the columns.
        * 
        * Outputs:
        *  M: The numerical inertia matrix M(thetalist) of an n-joint serial
        *     chain at the given configuration thetalist.
        */
        Eigen::MatrixXd MassMatrix(const Eigen::VectorXd&,
                                   const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);



        /* 
        * Function: This function calls InverseDynamics with g = 0, Ftip = 0, and 
        * ddthetalist = 0.      
        *
        * Inputs:
        *  thetalist: n-vector of joint variables
        *  dthetalist: A list of joint rates
        *  Mlist: List of link frames {i} relative to {i-1} at the home position
        *  Glist: Spatial inertia matrices Gi of the links
        *  Slist: Screw axes Si of the joints in a space frame, in the format
        *         of a matrix with the screw axes as the columns.
        * 
        * Outputs:
        *  c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
        *     terms for a given thetalist and dthetalist.
        */
        Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd&, const Eigen::VectorXd&,
                                           const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        /* 
        * Function: This function calls InverseDynamics with g = 0, dthetalist = 0, and 
        * ddthetalist = 0.  
        *
        * Inputs:
        *  thetalist: n-vector of joint variables 
        *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
        *  Mlist: List of link frames {i} relative to {i-1} at the home position
        *  Glist: Spatial inertia matrices Gi of the links
        *  Slist: Screw axes Si of the joints in a space frame, in the format
        *         of a matrix with the screw axes as the columns.
        * 
        * Outputs:
        *  JTFtip: The joint forces and torques required only to create the 
        *     end-effector force Ftip.
        */
        Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd&, const Eigen::VectorXd&, 
                                              const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        /* 
        * Function: This function computes ddthetalist by solving:
        * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist) 
        *                                  - g(thetalist) - Jtr(thetalist) * Ftip
        * Inputs:
        *  thetalist: n-vector of joint variables
        *  dthetalist: n-vector of joint rates
        *  taulist: An n-vector of joint forces/torques
        *  g: Gravity vector g
        *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
        *  Mlist: List of link frames {i} relative to {i-1} at the home position
        *  Glist: Spatial inertia matrices Gi of the links
        *  Slist: Screw axes Si of the joints in a space frame, in the format
        *         of a matrix with the screw axes as the columns.
        * 
        * Outputs:
        *  ddthetalist: The resulting joint accelerations
        * 
        */
        Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, 
                                         const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, 
                                         const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);



        Eigen::VectorXd computeVelocity(const Eigen::VectorXd& V, const Eigen::MatrixXd& T);

        std::vector<Eigen::VectorXd> computeOccupancy(const Eigen::VectorXd& q_init, const Eigen::VectorXd& dq_init,const Eigen::VectorXd& ddq_init, const int & N, const double & Vmax);

    private:

        Eigen::MatrixXd  S_list_, B_list_  ;
        Eigen::Matrix4d M_hand_;
        std::map<std::string, Eigen::Matrix4d> M_joint_, M_CoM_;
        const int ndof_;
        Eigen::VectorXd q_, dq_, ddq_ ;

     
 };

#endif
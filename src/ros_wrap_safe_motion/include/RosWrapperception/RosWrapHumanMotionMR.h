#ifndef ROSWRAPHUMANMOTIONMR_H
#define ROSWRAPHUMANMOTIONMR_H
#pragma once 
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
class HumanMotionMR {

public: 
 
HumanMotionMR(const int & ndof, const Eigen::MatrixXd & M_home, const Eigen::MatrixXd & Slist, const Eigen::MatrixXd & Blist,
              const std::vector<Eigen::MatrixXd> & Mlist, const std::vector<Eigen::MatrixXd>& Glist);

/**
 * @brief return the 3x3 skew matrix of omage 
 * 
 */
Eigen::MatrixXd VecToso3(const Eigen::VectorXd & omega){
  Eigen::MatrixXd skewMatrix(3,3);
  skewMatrix << 0, -omega(2), omega(1),
                omega(2), 0, -omega(0),
                -omega(1), omega(0), 0;

   return skewMatrix;
}

/**
 * @brief calculation the associated vector of skew matrix
 * 
 * @param so3mat 
 * @return Eigen::Vector3d 
 */
Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat) {
		Eigen::Vector3d v_ret;
		v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
		return v_ret;
}

/**
 * @brief extrac the rotation axis and the rotation amount theta from teh exponential coordinates for rotation
 * 
 * @param expc3 
 * @return [x, y,z theta]
 */
Eigen::Vector4d AxisAng3(const Eigen::Vector3d & expc3);

/**
 * @brief compute the rotation matrix R from the exponential coordinate 
 * 
 * @param so3mat 
 * @return R
 */
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d & so3mat);

/**
 * @brief Compute the matrix logarithm so(3) from rotation matrix
 * 
 * @param R 
 * @return Eigen::Matrix3d 
 */
Eigen::MatrixXd MatrixLog3(const Eigen::MatrixXd & R);
/**
 * @brief Build the homogenous transformation matrix T from R et p
 * 
 * @param R 
 * @param p 
 * @return homogenous transformation T
 */
Eigen::MatrixXd RpToTrans(const Eigen::MatrixXd & R, const Eigen::VectorXd& p){
    Eigen::MatrixXd T(4,4);
    T.block(0,0,3,3) = R ;
    T.block(0,3,3,1) = p ;
    return T; 
}

/**
 * @brief Extract Rotation and Translation from homogeneous transformation
 * 
 * @param T 
 * @return Rotation and translation
 */
std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T)
{
    std::vector<Eigen::MatrixXd> Rp ;
    Rp.resize(2);
    Rp[0].resize(3,3);
    Rp[1].resize(3,1);
    Rp[0] = T.block(0,0,3,3);
    Rp[1] = T.block(0,3,3,1);
    return Rp;
}

/**
 * @brief Compute the inverse of homogeneous transformation
 * 
 * @param T 
 * @return T inverse 
 */
Eigen::MatrixXd TransInv(const Eigen::MatrixXd &T){
    auto Rp = TransToRp(T);
    Eigen::MatrixXd T_inv(4,4);
    T_inv.setZero();
    T_inv.block(0,0,3,3) = Rp[0].transpose();
    T_inv.block(0,3,3,1) = -Rp[0].transpose()*Rp[1];

    return T_inv;
}

/**
 * @brief the se(3) matrix corresponding to a 6 dim twist
 * 
 * @param V 
 * @return Eigen::Matrix4d 
 */
Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V){

    Eigen::MatrixXd se3mat ;
    se3mat.resize(4,4);
    se3mat.setZero();
    se3mat.block(0,0,3,3) = VecToso3(V.segment(0,3));
    se3mat.block(0,3,3,1) = V.tail(3);

    return se3mat;
} 

/**
 * @brief Extract the 6 dim twist from a se(3) matrix
 * 
 * @param se3mat 
 * @return Eigen::VectorXd 
 */
Eigen::VectorXd se3ToVec(const Eigen::MatrixXd & se3mat){
    Eigen::VectorXd V;
    V.resize(6);
    V.segment(0,3) = so3ToVec(se3mat.block(0,0,3,3));
    V.tail(3) = se3mat.block(0,3,3,1);

}

/**
 * @brief adjoint representation of the homogeneous transformation matrix T 
 * 
 * @param T 
 * @return  6x6 matrix
 */
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &T){
    Eigen::MatrixXd AdT(6,6);
    AdT.setZero();

    AdT.block(0,0,3,3) = T.block(0,0,3,3);
    AdT.block(3,3,3,3) = T.block(0,0,3,3);
    AdT.block(3,0,3,3) = VecToso3(T.block(0,3,3,1))*T.block(0,0,3,3);

    return AdT;

}

/**
 * @brief returns a normalized screw axis represtation S
 * 
 * @param q 
 * @param s 
 * @param h 
 * @return Eigen::VectorXd 
 */
Eigen::VectorXd ScrewToAxis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, const  double & h){
    Eigen::VectorXd axis(6);
    axis.segment(0,3) = s;
    axis.segment(3,3) = q.cross(s) + (h*s);

    return axis;
}
/**
 * @brief 
 * 
 * @param expc6 
 * @return Eigen::VectorXd 
 */
Eigen::VectorXd AxisAng6(const Eigen::VectorXd& expc6);

/**
 * @brief Computes the homogeneous transformation matrix T 
 * 
 * @param se3mat 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat);

/**
 * @brief 
 * 
 * @param T 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T);

/**
 * @brief Compute the product of exponential method fo forward kinematic
 * 
 * @param M 
 * @param Blist 
 * @param thetalist 
 * @return Transformation matrix bewteen hand and base 
 */
Eigen::MatrixXd FKinBody(const Eigen::MatrixXd &M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd & thetalist);

/**
 * @brief update private data 
 * 
 */
void ComputeFKinBody(){
    Eigen::MatrixXd Th(4,4);
    Th.setIdentity();
    Th = FKinBody(M_home_,Blist_,q_);
    
    Th_ = Th;
};

Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd &M, const Eigen::MatrixXd& Slist_, const Eigen::VectorXd & thetalist);

void ComputeFKinSpace(){
    Eigen::MatrixXd Th(4,4);
    Th.setIdentity();
    Th = FKinSpace(M_home_,Slist_,q_);
    
    Th_ = Th;
};

/**
 * @brief Compute Jacobian in space frame using Screw list described in space frame 
 * 
 * @param Slist 
 * @param thetalist 
 * @return 6*ndof Body Jacobian 
 */
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd & Slist, const Eigen::VectorXd & thetalist);
void JacobianSpace(const Eigen::VectorXd & thetalist){
    Jacobian_space_ = JacobianSpace(Slist_, thetalist);
}

Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd & Blist, const Eigen::VectorXd & thetalist);
void JacobianBody(const Eigen::VectorXd & thetalist){
    Jacobian_body_ = JacobianBody(Blist_, thetalist);
}

Eigen::MatrixXd JsToJb(const Eigen::MatrixXd& Js, const Eigen::MatrixXd& Tsb){
    Eigen::MatrixXd Jb(6,ndof_);
    Jb = Adjoint(Tsb)*Js;

    std::cout <<" Jb :\n " << Jb << std::endl;
    return Jb;
}

Eigen::VectorXd TwistBodyToBase(const Eigen::VectorXd & Vb, const Eigen::MatrixXd &Rot ){

    Eigen::VectorXd Vs(6);
    Vs.segment(0,3) = Rot*Vb.segment(0,3);
    Vs.segment(3,3) = Rot*Vb.segment(3,3);

    return Vs;
}

/**
 * @brief compute ad
 * 
 * @param V 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd ad(const Eigen::MatrixXd & V){

		Eigen::Matrix3d omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

		Eigen::MatrixXd result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
		result.bottomLeftCorner<3, 3>() = VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
		return result;
}

/**
 * @brief This function compute the inverse dynamic: taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
 *                                                              + g(thetalist) + Jtr(thetalist) * Ftip
 * @param thetalis: n-vector of joint variables 
 * @param dthetalist: n-vector of joint rates
 * @param ddthetalist: n-vector of joint accelerations 
 * @param g: Gravity vector g 
 * @param Ftip: Spatial force applied by the end-effector expressed in frame {n+1} 
 * @param Mlist: List of link frames {i} relative to {i-1} at the home position 
 * @param Glist: Spatial inertia matrices Gi of the links 
 * @param Slist: Screw axes Si of the joints in a space frame, in the format  of a matrix with the screw axes as the columns 
 * @return torque::The n-vector of required joint forces/torques
 */
Eigen::VectorXd InverseDynamic(const Eigen::VectorXd &thetalist, const Eigen::VectorXd &dthetalist, const Eigen::VectorXd & ddthetalist, const Eigen::VectorXd &g,
                               const Eigen::VectorXd &Ftip, const std::vector<Eigen::MatrixXd> &Mlist, const std::vector<Eigen::MatrixXd>& Glist,
                               const Eigen::MatrixXd & Slist);
/**
 * @brief Compute Gravity forces acting in the chain This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and
 *   ddthetalist = 0. 
 * @param thetalist 
 * @param g 
 * @param Mlist 
 * @param Glist 
 * @param Slist 
 * @return Eigen::VectorXd 
 */
Eigen::VectorXd GravityForce(const Eigen::VectorXd & thetalist, const Eigen::VectorXd& g,
                             const std::vector<Eigen::MatrixXd> & Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd Slist );

Eigen::MatrixXd MassMatrix(const Eigen::VectorXd & thetalist, 
                const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist);

Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist,
                     const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist);

private:
const int ndof_;

Eigen::MatrixXd M_home_;
std::vector<Eigen::MatrixXd> Mlist_, Glist_;
Eigen::MatrixXd Blist_, Slist_;
Eigen::VectorXd q_, tau_; 
Eigen::MatrixXd Th_;
Eigen::MatrixXd Jacobian_body_, Jacobian_space_ ;
};

#endif
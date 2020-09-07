#include "RosWrapperception/RosWrapHumanMotionMR.h"

const double pi=3.14159265358979323846;
bool nearZero(const double val){
    return std::abs(val) < 0.00001 ;
}

Eigen::MatrixXd Normalize(Eigen::MatrixXd V) {
		V.normalize();
		return V;
	}

HumanMotionMR::HumanMotionMR(const int & ndof,const Eigen::MatrixXd & M_home,
			   const Eigen::MatrixXd & Slist, const Eigen::MatrixXd & Blist,const std::vector<Eigen::MatrixXd> &Mlist,
			   const std::vector<Eigen::MatrixXd> &Glist )
			   :ndof_(ndof), M_home_(M_home), Slist_(Slist), Blist_(Blist), Mlist_(Mlist), Glist_(Glist){

         
			  tau_.resize(ndof_);
		



              std::cout <<" human motion class created " << std::endl;
			}

Eigen::Vector4d HumanMotionMR::AxisAng3(const Eigen::Vector3d& expc3) {
		Eigen::Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}

Eigen::Matrix3d HumanMotionMR::MatrixExp3(const Eigen::Matrix3d & so3mat){

    Eigen::MatrixXd rot(3,3) ;
	rot.setIdentity();
    Eigen::VectorXd omgtheta(3) ;
	omgtheta = so3ToVec(so3mat);
    if (nearZero(omgtheta.norm())){
        return rot;
    }else
    {
        const double theta = AxisAng3(omgtheta)(3);
        Eigen::Matrix3d omgmat = so3mat/theta;
        return rot + std::sin(theta)*omgmat + (1-std::cos(theta))*omgmat*omgmat;
    }
    
}

Eigen::MatrixXd HumanMotionMR::MatrixLog3(const Eigen::MatrixXd & R){
    
		double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!nearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!nearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
}

Eigen::VectorXd HumanMotionMR::AxisAng6(const Eigen::VectorXd& expc6){
		Eigen::VectorXd v_ret(7);
		double theta = Eigen::Vector3d(expc6(0), expc6(1), expc6(2)).norm();
		if (nearZero(theta))
			theta = Eigen::Vector3d(expc6(3), expc6(4), expc6(5)).norm();
		v_ret << expc6 / theta, theta;
		return v_ret;
}

	Eigen::MatrixXd HumanMotionMR::MatrixExp6(const Eigen::MatrixXd& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
		Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

		Eigen::MatrixXd m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (nearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				0, 0, 0, 1;
			return m_ret;
		}

	}

    Eigen::MatrixXd HumanMotionMR::MatrixLog6(const Eigen::MatrixXd& T) {
		Eigen::MatrixXd m_ret(4, 4);
		auto rp = TransToRp(T);
		Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
		Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
		if (nearZero(omgmat.norm())) {
			m_ret << zeros3d, rp.at(1),
				0, 0, 0, 0;
		}
		else {
			double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
			Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
			Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*rp.at(1),
				0, 0, 0, 0;
		}
		return m_ret;
	}

	Eigen::MatrixXd HumanMotionMR::FKinBody(const Eigen::MatrixXd &M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd & thetalist){
		Eigen::MatrixXd T(4,4);
		T = M;
	    
		const int dim = thetalist.size();

		for (int i(0); i<dim ; i++){

			T = T*MatrixExp6(VecTose3(Blist.col(i)*thetalist(i)));
		}
	 	
		return T;
		

	}

	Eigen::MatrixXd HumanMotionMR::FKinSpace(const Eigen::MatrixXd &M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd & thetalist){
		Eigen::MatrixXd T;
		T = M;
		int dim = thetalist.size();

		for (int i(dim-1); i>-1 ; i--){
			
			 T = MatrixExp6(VecTose3(Slist.col(i)*thetalist(i)))*T;
		}
	 	
		return T;
		

	}


Eigen::MatrixXd HumanMotionMR::JacobianSpace(const Eigen::MatrixXd & Slist, const Eigen::VectorXd & thetalist){

	Eigen::MatrixXd Js(6,ndof_);
	Js = Slist;
	Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
	Eigen::VectorXd Screw_temp(6);

	for (int i(1); i < ndof_; i++){
		Screw_temp = Slist.col(i-1)*thetalist(i-1);
		T = T * MatrixExp6(VecTose3(Screw_temp));
		Js.col(i) = Adjoint(T)*Slist.col(i);

	}
	return Js;
}

Eigen::MatrixXd HumanMotionMR::JacobianBody(const Eigen::MatrixXd & Blist, const Eigen::VectorXd & thetalist){

	Eigen::MatrixXd Jb(6,ndof_);
	Jb = Blist;
	Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
	Eigen::VectorXd Screw_temp(6);

	for (int i(ndof_-2); i >= 0 ; i--){
		Screw_temp = -Blist.col(i+1)*thetalist(i+1);
		T = T * MatrixExp6(VecTose3(Screw_temp));
		Jb.col(i) = Adjoint(T)*Blist.col(i);

	}

	return Jb;
}


Eigen::VectorXd HumanMotionMR::InverseDynamic(const Eigen::VectorXd &thetalist, const Eigen::VectorXd &dthetalist, const Eigen::VectorXd & ddthetalist, const Eigen::VectorXd &g,
                               const Eigen::VectorXd &Ftip, const std::vector<Eigen::MatrixXd> &Mlist, const std::vector<Eigen::MatrixXd>& Glist,
                               const Eigen::MatrixXd & Slist){
								   

	Eigen::MatrixXd Mi = Eigen::MatrixXd::Identity(4,4);	
	Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6,ndof_);
	std::vector<Eigen::MatrixXd> AdTi;
	for (int i = 0; i < ndof_+1; i++) {
			AdTi.push_back(Eigen::MatrixXd::Zero(6,6));
	}						   
	Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6,ndof_+1);    // velocity
	Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6,ndof_+1);   // acceleration

	Vdi.block(3, 0, 3, 1) = - g;
	AdTi[ndof_] = Adjoint(TransInv(Mlist[ndof_]));
	Eigen::VectorXd Fi = Ftip;

	Eigen::VectorXd taulist = Eigen::VectorXd::Zero(ndof_);

	// forward pass
	for (int i = 0; i < ndof_; i++) {
			Mi = Mi * Mlist[i];
			Ai.col(i) = Adjoint(TransInv(Mi))*Slist.col(i);

			AdTi[i] = Adjoint(MatrixExp6(VecTose3(Ai.col(i)*-thetalist(i)))
			          * TransInv(Mlist[i]));

			Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
			Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
						   + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
			std::cout <<" Vi : " << i << '\n' << Vi.col(i+1) <<std::endl;
			std::cout <<" Vdi : " << i << '\n' << Vdi.col(i+1) <<std::endl;

		}
	// backward pass
		for (int i = ndof_-1; i >= 0; i--) {
			Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
			     - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
			taulist(i) = Fi.transpose() * Ai.col(i);
			std::cout <<" Ai :" <<i <<'\n' <<Ai.col(i) << std::endl;
			std::cout <<" Fi : " << i << '\n' << Fi <<std::endl;
			std::cout <<" taui : " << i << '\n' << taulist(i) <<std::endl;
		}
	return taulist;
}

Eigen::VectorXd HumanMotionMR::GravityForce(const Eigen::VectorXd & thetalist, const Eigen::VectorXd& g,
                             const std::vector<Eigen::MatrixXd> & Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd Slist ){

		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd grav = InverseDynamic(thetalist, dummylist, dummylist, g,
                                                dummyForce, Mlist, Glist, Slist);
		return grav;							 

	}


Eigen::MatrixXd HumanMotionMR::MassMatrix(const Eigen::VectorXd & thetalist, 
                const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist){
					

		int n = thetalist.size();
		Eigen::VectorXd dummydtheta = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n,n);
		for (int i = 0; i < n; i++) {
			Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
			ddthetalist(i) = 1;
			M.col(i) = InverseDynamic(thetalist, dummydtheta, ddthetalist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		}
		return M;			
}


Eigen::VectorXd HumanMotionMR::VelQuadraticForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist,
                                const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist){

        int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd c = InverseDynamic(thetalist, dthetalist, dummylist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		return c;

}
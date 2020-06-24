#include "RosWrapperception/RosWrapHumanMotion.h"



HumanMotion::HumanMotion(const int& ndof,const std::map<std::string, Eigen::Matrix4d> & M_joint,
                         const std::map<std::string, Eigen::Matrix4d> & M_CoM,const Eigen::MatrixXd &S_list):ndof_(ndof){

            for (auto ite = M_joint.begin(); ite != M_joint.end(); ite++ ){

                M_joint_[ite->first] = ite->second;
            }

            for (auto ite = M_CoM.begin(); ite != M_CoM.end(); ite++ ){

                M_CoM_[ite->first] = ite->second;
            }

            S_list_.resize(6,ndof_);
            B_list_.resize(6,ndof_);

            S_list_ = S_list ;

};

bool HumanMotion::nearZero(const double& val){
    return (std::abs(val) < 0.000001 );
}



Eigen::MatrixXd HumanMotion::VecToso3(const Eigen::Vector3d& omega){

    Eigen::Matrix3d m ;

    m << 0, -omega(2), omega(1),
        omega(2), 0, -omega(0),
        -omega(1), omega(0), 0 ;
    
    return m;
}


Eigen::MatrixXd HumanMotion::ad(Eigen::VectorXd V) {
		Eigen::Matrix3d omgmat = HumanMotion::VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

		Eigen::MatrixXd result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
		result.bottomLeftCorner<3, 3>() = HumanMotion::VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
		return result;
}

Eigen::Vector3d HumanMotion::so3ToVec(const Eigen::Matrix3d& omega_matrix){

    Eigen::Vector3d omega;
    omega << omega_matrix(2,1), omega_matrix(0,2), omega_matrix(1,0);

    return omega;
}


Eigen::Vector4d HumanMotion::AxisAng3(const Eigen::Vector3d& expc3){

    Eigen::Vector4d v_ret; 

    v_ret << expc3.normalized(), expc3.norm();
    return v_ret;    
}

Eigen::Matrix3d HumanMotion::MatrixExp3(const Eigen::Matrix3d& so3mat){
    
    Eigen::Vector3d omgtheta = HumanMotion::so3ToVec(so3mat);

		Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
		if (HumanMotion::nearZero(so3mat.norm())) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
}


Eigen::Matrix3d HumanMotion::MatrixLog3(const Eigen::Matrix3d& R){
    
     double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!HumanMotion::nearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!HumanMotion::nearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = HumanMotion::VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}  
}

Eigen::MatrixXd HumanMotion::RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p){

    Eigen::MatrixXd m_ret(4,4);
    
    m_ret << R, p,
             0,0,0,1;
    return m_ret;
 }


std::vector<Eigen::MatrixXd> HumanMotion::TransToRp(const Eigen::MatrixXd& T){
    
    std::vector<Eigen::MatrixXd> Rp_ret;
    Eigen::Matrix3d R_ret;
	
    // Get top left 3x3 corner
	R_ret = T.block<3, 3>(0, 0);

	Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

	Rp_ret.push_back(R_ret);
	Rp_ret.push_back(p_ret);

	return Rp_ret;
}

Eigen::MatrixXd HumanMotion::VecTose3(const Eigen::VectorXd& V){
    // Separate angular (exponential representation) and linear velocities
    Eigen::Vector3d exp(V(0), V(1), V(2));
	Eigen::Vector3d linear(V(3), V(4), V(5));

	// Fill in values to the appropriate parts of the transformation matrix
	Eigen::MatrixXd m_ret(4, 4);
	m_ret << HumanMotion::VecToso3(exp), linear,
			0, 0, 0, 0;

	return m_ret;
}

Eigen::VectorXd HumanMotion::se3ToVec(const Eigen::MatrixXd& T){

    Eigen::VectorXd m_ret(6);
	
    m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

	return m_ret;
}

Eigen::MatrixXd HumanMotion::TransInv(const Eigen::MatrixXd& transform){
    
    auto rp = HumanMotion::TransToRp(transform);
	auto Rt = rp.at(0).transpose();
	auto t = -(Rt * rp.at(1));
	Eigen::MatrixXd inv(4, 4);
	inv = Eigen::MatrixXd::Zero(4,4);
	inv.block(0, 0, 3, 3) = Rt;
	inv.block(0, 3, 3, 1) = t;
	inv(3, 3) = 1;
	return inv;

}

Eigen::MatrixXd HumanMotion::Adjoint(const Eigen::MatrixXd& T){

    	std::vector<Eigen::MatrixXd> R = HumanMotion::TransToRp(T);
		Eigen::MatrixXd ad_ret(6, 6);
		ad_ret = Eigen::MatrixXd::Zero(6, 6);
		Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
		ad_ret << R[0], zeroes,
			   HumanMotion::VecToso3(R[1]) * R[0], R[0];
		return ad_ret;


}


Eigen::MatrixXd HumanMotion::MatrixExp6(const Eigen::MatrixXd& se3mat){
        // Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
		Eigen::Vector3d omgtheta =  HumanMotion::so3ToVec(se3mat_cut);

		Eigen::MatrixXd m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if ( HumanMotion::nearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (HumanMotion::AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				     0, 0, 0, 1;
			return m_ret;
		}


}



Eigen::MatrixXd HumanMotion::MatrixLog6(const Eigen::MatrixXd& T) {
		Eigen::MatrixXd m_ret(4, 4);
		auto rp = HumanMotion::TransToRp(T);
		Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
		Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
		if (HumanMotion::nearZero(omgmat.norm())) {
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


Eigen::MatrixXd HumanMotion::FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList){
    
    	Eigen::MatrixXd T;
		T.resize(4,4) ;
		T =  M;

		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}

		return T;
}


Eigen::MatrixXd HumanMotion::FKinBody(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList){
    
    	Eigen::MatrixXd T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = T*MatrixExp6(VecTose3(Blist.col(i)*thetaList(i))) ;
		}
		return T;
}


Eigen::MatrixXd HumanMotion::JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList){

        Eigen::MatrixXd Js = Slist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd sListTemp(Slist.col(0).size());
		for (int i = 1; i < thetaList.size(); i++) {
			sListTemp << Slist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}

		return Js;
}


Eigen::MatrixXd HumanMotion::JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList){
    
    	Eigen::MatrixXd Jb = Blist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd bListTemp(Blist.col(0).size());
		for (int i = thetaList.size() - 2; i >= 0; i--){
            
            bListTemp << Blist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
}

Eigen::MatrixXd HumanMotion::getRelativeTransformation(const Eigen::MatrixXd& Ti, const Eigen::MatrixXd& Tj){

    Eigen::MatrixXd rel_trans(4,4);

    rel_trans = HumanMotion::TransInv(Ti)*Tj;

}

Eigen::VectorXd HumanMotion::computeVelocity(const Eigen::VectorXd& V, const Eigen::MatrixXd& T){

    Eigen::VectorXd Velocity ;
    Velocity.resize(6);
	Velocity.setZero();

    auto Rp = HumanMotion::TransToRp(T);
    Eigen::Vector3d p = Rp[1];
    Eigen::Vector3d omg , lin_vel;
    omg = V.head(3);
	lin_vel = V.tail(3) +  omg.cross(p);
	Velocity.head(3) << omg;
    Velocity.tail(3) << lin_vel;

    return Velocity;
}

bool HumanMotion::IKinSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T,
		Eigen::VectorXd& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		Eigen::MatrixXd Tfk = FKinSpace(M, Slist, thetalist);
		Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
		Eigen::VectorXd Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
		Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Eigen::MatrixXd Js;
		while (err && i < maxiterations) {
			Js = JacobianSpace(Slist, thetalist);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
			linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}

Eigen::VectorXd HumanMotion::InverseDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist,
									const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
									const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
	    // the size of the lists
		int n = thetalist.size();

		Eigen::MatrixXd Mi = Eigen::MatrixXd::Identity(4, 4);
		Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6,n);
		std::vector<Eigen::MatrixXd> AdTi;
		for (int i = 0; i < n+1; i++) {
			AdTi.push_back(Eigen::MatrixXd::Zero(6,6));
		}
		Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6,n+1);    // velocity
		Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6,n+1);   // acceleration

		Vdi.block(3, 0, 3, 1) = - g;
		AdTi[n] = HumanMotion::Adjoint(HumanMotion::TransInv(Mlist[n]));
		Eigen::VectorXd Fi = Ftip;

		Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);

		// forward pass
		for (int i = 0; i < n; i++) {
			Mi = Mi * Mlist[i];
			Ai.col(i) = HumanMotion::Adjoint(HumanMotion::TransInv(Mi))*Slist.col(i);

			AdTi[i] = HumanMotion::Adjoint(HumanMotion::MatrixExp6(HumanMotion::VecTose3(Ai.col(i)*-thetalist(i)))
			          * HumanMotion::TransInv(Mlist[i]));

			Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
			Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
						   + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
		}

		// backward pass
		for (int i = n-1; i >= 0; i--) {
			Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
			     - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
			taulist(i) = Fi.transpose() * Ai.col(i);
		}
		return taulist;
	}


Eigen::VectorXd  HumanMotion::GravityForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& g,
											const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
	    int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd grav =  HumanMotion::InverseDynamics(thetalist, dummylist, dummylist, g,
                                                dummyForce, Mlist, Glist, Slist);
		return grav;
	}

Eigen::MatrixXd HumanMotion::MassMatrix(const Eigen::VectorXd& thetalist,
                        	const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n,n);
		for (int i = 0; i < n; i++) {
			Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
			ddthetalist(i) = 1;
			M.col(i) =  HumanMotion::InverseDynamics(thetalist, dummylist, ddthetalist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		}
		return M;
	}


Eigen::VectorXd HumanMotion::VelQuadraticForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist,
                                const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd c = HumanMotion::InverseDynamics(thetalist, dthetalist, dummylist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		return c;
}


Eigen::VectorXd HumanMotion::EndEffectorForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& Ftip,
								const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);

		Eigen::VectorXd JTFtip = HumanMotion::InverseDynamics(thetalist, dummylist, dummylist,
                             dummyg, Ftip, Mlist, Glist, Slist);
		return JTFtip;
}

Eigen::VectorXd HumanMotion::ForwardDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& taulist,
									const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
									const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {

		Eigen::VectorXd totalForce = taulist - HumanMotion::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
                 							 - HumanMotion::GravityForces(thetalist, g, Mlist, Glist, Slist)
                                             - HumanMotion::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

		Eigen::MatrixXd M = HumanMotion::MassMatrix(thetalist, Mlist, Glist, Slist);

		// Use LDLT since M is positive definite
        Eigen::VectorXd ddthetalist = M.ldlt().solve(totalForce);

		return ddthetalist;
}

	
std::vector<Eigen::VectorXd> HumanMotion::computeOccupancy(const Eigen::VectorXd& q_init, const Eigen::VectorXd& dq_init,const Eigen::VectorXd& ddq_init, const int & N, const double & Vmax){

	double dt = 0.05,  L1 = 1., L2 = 1.;
	Eigen::VectorXd q, dq, ddq ;

	q = q_init;
	dq = dq_init;
	ddq = ddq_init;
	
    Eigen::Matrix4d M_hand ;

    M_hand << 1,0,0,L1 + L2,
              0,1,0,0,
              0,0,1,0,
              0,0,0,1;

    std::vector<Eigen::Matrix4d> M_list ;
    M_list.resize(3);
    
    M_list[0] << 1,0,0,0,
                 0,1,0,0,
                 0,0,1,0,
                 0,0,0,1;

    M_list[1] << 1,0,0,L1,
                 0,1,0,0,
                 0,0,1,0,
                 0,0,0,1;

    M_list[2] = M_hand;

    Eigen::MatrixXd S_list;

    S_list.resize(6,2) ;
    S_list.block(0,0,6,1) << 0, 0, 1, 0, 0, 0;
    S_list.block(0,1,6,1) << 0, 0, 1, 0, -1, 0;
	
	Eigen::MatrixXd J_tool_space ;
    J_tool_space.resize(6,2);

	Eigen::VectorXd ee_twist_space, ee_vel; 
    ee_twist_space.resize(6);
	ee_vel.resize(6);
	Eigen::Matrix4d ee_trans_base;
	ee_trans_base = FKinSpace(M_hand, S_list, q);
	
	auto Rp = TransToRp(ee_trans_base);
	Eigen::Vector3d p = Rp[1];
	std::vector<Eigen::VectorXd> hand_prediction_data;

	hand_prediction_data.resize(N-1);

	for (int i(0); i < N-1 ; i++){

		q = q + dq*dt + ddq*dt*dt/2;
	
    	J_tool_space = JacobianSpace(S_list,q);
    	ee_twist_space = J_tool_space*dq;
        ee_trans_base = FKinSpace(M_hand, S_list, q);
		auto Rp = TransToRp(ee_trans_base);
	    p = Rp[1];

        ee_vel = computeVelocity(ee_twist_space,ee_trans_base);
		Eigen::Vector3d ee_vel_normalized ;
		ee_vel_normalized = ee_vel.tail(3).normalized();
	    p = p + ee_vel_normalized*Vmax*dt ;

		ee_trans_base.block(0,3,3,1) << p;
	
		auto Ik_solved = IKinSpace(S_list, M_hand, ee_trans_base, q, 0.001, 0.001);
		std::cout <<" q  after max velocity\n" << q <<'\n';


		hand_prediction_data[i].resize(2);
		hand_prediction_data[i] =  p;
	}

	return hand_prediction_data;
}
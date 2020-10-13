#include "humanmodel/humanmodel.h"




humanModel::humanModel(const int& ndof,const std::vector<Eigen::Matrix4d> & M_home):ndof_(ndof){

    M_.resize(ndof_);
    M_tool_ = M_home[ndof_-1];

    for( int i(0); i< ndof_; i++){
        M_[i] = M_home[i];
    }


};

bool humanModel::nearZero(const double& val){
    return (std::abs(val) < 0.000001 );
}



Eigen::MatrixXd humanModel::VecToso3(const Eigen::Vector3d& omega){

    Eigen::Matrix3d m ;

    m << 0, -omega(2), omega(1),
        omega(2), 0, -omega(0),
        -omega(1), omega(0), 0 ;
    
    return m;
}

Eigen::Vector3d humanModel::so3ToVec(const Eigen::Matrix3d& omega_matrix){

    Eigen::Vector3d omega;
    omega << omega_matrix(2,1), omega_matrix(0,2), omega_matrix(1,0);

    return omega;
}


Eigen::Vector4d humanModel::AxisAng3(const Eigen::Vector3d& expc3){

    Eigen::Vector4d v_ret; 

    v_ret << expc3.normalized(), expc3.norm();
    return v_ret;    
}

Eigen::Matrix3d humanModel::MatrixExp3(const Eigen::Matrix3d& so3mat){
    
    Eigen::Vector3d omgtheta = humanModel::so3ToVec(so3mat);

		Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
		if (humanModel::nearZero(so3mat.norm())) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
}


Eigen::Matrix3d humanModel::MatrixLog3(const Eigen::Matrix3d& R){
    
     double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!humanModel::nearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!humanModel::nearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = humanModel::VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}  
}

Eigen::MatrixXd humanModel::RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p){

    Eigen::MatrixXd m_ret(4,4);
    
    m_ret << R, p,
             0,0,0,1;
    return m_ret;
 }


std::vector<Eigen::MatrixXd> humanModel::TransToRp(const Eigen::MatrixXd& T){
    
    std::vector<Eigen::MatrixXd> Rp_ret;
    Eigen::Matrix3d R_ret;
	
    // Get top left 3x3 corner
	R_ret = T.block<3, 3>(0, 0);

	Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

	Rp_ret.push_back(R_ret);
	Rp_ret.push_back(p_ret);

	return Rp_ret;
}

Eigen::MatrixXd humanModel::VecTose3(const Eigen::VectorXd& V){
    // Separate angular (exponential representation) and linear velocities
    Eigen::Vector3d exp(V(0), V(1), V(2));
	Eigen::Vector3d linear(V(3), V(4), V(5));

	// Fill in values to the appropriate parts of the transformation matrix
	Eigen::MatrixXd m_ret(4, 4);
	m_ret << humanModel::VecToso3(exp), linear,
			0, 0, 0, 0;

	return m_ret;
}

Eigen::VectorXd humanModel::se3ToVec(const Eigen::MatrixXd& T){

    Eigen::VectorXd m_ret(6);
	
    m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

	return m_ret;
}

Eigen::MatrixXd humanModel::TransInv(const Eigen::MatrixXd& transform){
    
    auto rp = humanModel::TransToRp(transform);
	auto Rt = rp.at(0).transpose();
	auto t = -(Rt * rp.at(1));
	Eigen::MatrixXd inv(4, 4);
	inv = Eigen::MatrixXd::Zero(4,4);
	inv.block(0, 0, 3, 3) = Rt;
	inv.block(0, 3, 3, 1) = t;
	inv(3, 3) = 1;
	return inv;

}

Eigen::MatrixXd humanModel::Adjoint(const Eigen::MatrixXd& T){

    	std::vector<Eigen::MatrixXd> R = humanModel::TransToRp(T);
		Eigen::MatrixXd ad_ret(6, 6);
		ad_ret = Eigen::MatrixXd::Zero(6, 6);
		Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
		ad_ret << R[0], zeroes,
			   humanModel::VecToso3(R[1]) * R[0], R[0];
		return ad_ret;


}


Eigen::MatrixXd humanModel::MatrixExp6(const Eigen::MatrixXd& se3mat){
        // Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
		Eigen::Vector3d omgtheta =  humanModel::so3ToVec(se3mat_cut);

		Eigen::MatrixXd m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if ( humanModel::nearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (humanModel::AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				     0, 0, 0, 1;
			return m_ret;
		}


}



Eigen::MatrixXd humanModel::MatrixLog6(const Eigen::MatrixXd& T) {
		Eigen::MatrixXd m_ret(4, 4);
		auto rp = humanModel::TransToRp(T);
		Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
		Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
		if (humanModel::nearZero(omgmat.norm())) {
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


Eigen::MatrixXd humanModel::FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList){
    
    	Eigen::MatrixXd T;
		T.resize(4,4) ;
		T =  M;

		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}

		return T;
}


Eigen::MatrixXd humanModel::FKinBody(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList){
    
    	Eigen::MatrixXd T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = T*MatrixExp6(VecTose3(Blist.col(i)*thetaList(i))) ;
		}
		return T;
}


Eigen::MatrixXd humanModel::JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList){

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


Eigen::MatrixXd humanModel::JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList){
    
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

Eigen::MatrixXd humanModel::getRelativeTransformation(const Eigen::MatrixXd& Ti, const Eigen::MatrixXd& Tj){

    Eigen::MatrixXd rel_trans(4,4);

    rel_trans = humanModel::TransInv(Ti)*Tj;

}

Eigen::VectorXd humanModel::computeVelocity(const Eigen::VectorXd& V, const Eigen::MatrixXd& T){

    Eigen::VectorXd Velocity ;
    Velocity.resize(6);
	Velocity.setZero();

    auto Rp = humanModel::TransToRp(T);
    Eigen::Vector3d p = Rp[1];
    Eigen::Vector3d omg , lin_vel;
    omg = V.head(3);
	lin_vel = V.tail(3) +  omg.cross(p);
	Velocity.head(3) << omg;
    Velocity.tail(3) << lin_vel;

    return Velocity;
}

bool humanModel::IKinSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T,
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


std::vector<Eigen::VectorXd> humanModel::computeOccupancy(const Eigen::VectorXd& q_init, const Eigen::VectorXd& dq_init,const Eigen::VectorXd& ddq_init, const int & N, const double & Vmax){

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
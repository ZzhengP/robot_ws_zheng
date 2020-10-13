#include "RosWrapperception/RosWrapHumanMotion.h"

const double pi = 3.1415927;

using namespace std ;

HumanMotion::HumanMotion(const int & ndof, const Eigen::MatrixXd & DHparam, const double & lc1, const double & lc2, const double & lu,
					     const double & lf,const double & lh, const double & m1, const double & m2 )
						:ndof_(ndof), DHparam_(DHparam),lc1_(lc1), lc2_(lc2),lu_(lu), lf_(lf), lh_(lh), m1_(m1), m2_(m2){
			
			std::cout <<" create human motion class" << std::endl;
			handTransformMatrix_.setIdentity();
			handPosition_.setZero();

			int rows = DHparam_.rows();
			TransformationMatrixList_.resize(rows + 1);
			TransformationMatrixToBaseList_.resize(rows+1);

			// Linear and angular velocity 
			stacked_linear_velocity_.resize(ndof_); 
			stacked_angular_velocity_.resize(ndof_); 
	
			
		    // Linear and angular acceleration 
			stacked_linear_acc_.resize(ndof_); 
			stacked_angular_acc_.resize(ndof_); 
	

			q_.resize(ndof_);
			dq_.resize(ndof_);
			ddq_.resize(ndof_);
			q_.setZero();
			dq_.setZero();
			ddq_.setZero();
			aList_.resize(ndof_);
			alphaList_.resize(ndof_);
			dList_.resize(ndof_);
			thetaList_.resize(ndof_);
			aList_ = DHparam_.col(0);
			alphaList_ = DHparam_.col(1);
			dList_ = DHparam_.col(2);
			thetaList_ = DHparam_.col(3);


			for (int it(0); it < ndof_; it ++){

				 TransformationMatrixList_[it] << cos(thetaList_(it)), -sin(thetaList_(it)), 0 , aList_(it),
				 								  sin(thetaList_(it))*cos(alphaList_(it)), cos(thetaList_(it))*cos(alphaList_(it)), -sin(alphaList_(it)), -sin(alphaList_(it))*dList_(it),
												  sin(thetaList_(it))*sin(alphaList_(it)), cos(thetaList_(it))*sin(alphaList_(it)), cos(alphaList_(it)), cos(alphaList_(it))*dList_(it),
												  0,0,0,1 	 ; 	
				 handTransformMatrix_ = handTransformMatrix_*TransformationMatrixList_[it];
				 TransformationMatrixToBaseList_[it] = handTransformMatrix_;

				 stacked_linear_velocity_[it].setZero();
				 stacked_angular_velocity_[it].setZero();

				 stacked_linear_acc_[it].setZero();
				 stacked_angular_acc_[it].setZero();
			}				

			TransformationMatrixList_[ndof_] << 1, 0, 0, lf_,
						 						0, 1, 0, 0, 
												0, 0, 1, 0, 
												0, 0, 0, 1;
			handTransformMatrix_ = handTransformMatrix_*TransformationMatrixList_[ndof_];
			TransformationMatrixToBaseList_[ndof_] = handTransformMatrix_; 	

	 		T3C1_ << 1, 0, 0, lc1_,
				 	0, 1, 0, 0, 
			 		0, 0, 1, 0, 
				 	0, 0, 0, 1;

			T4C2_ << 1, 0, 0, lc2_,
				 	0, 1, 0, 0, 
			 		0, 0, 1, 0, 
				 	0, 0, 0, 1;
			Jacobian_.resize(6,ndof_);
			Jacobian_.setZero();
			
			Jacobian_C1_.resize(6,ndof_);
			Jacobian_C1_.setZero();
			Jacobian_C2_.resize(6,ndof_);
			Jacobian_C2_.setZero();

			Ic1_.setIdentity();
			Ic2_.setIdentity();

			std::cout <<" human motion created" << std::endl;

}


void HumanMotion::print() const{

	int i = 0 , j =0;
	for (auto it = TransformationMatrixList_.cbegin(); it != TransformationMatrixList_.cend(); it++) {

		std::cout << "relative transformation for " << i+1 << " to " << i << " is :\n "<<
				     (*it) << std::endl;
		i++; 
	}

	for (auto it = TransformationMatrixToBaseList_.cbegin(); it != TransformationMatrixToBaseList_.cend(); it++) {

		std::cout << " transformation for " << j+1  << " to " << 0 << " is :\n "<<
				     (*it) << std::endl;
		j++; 
	}

}

void HumanMotion::computeStackedVelocity(const Eigen::VectorXd& dtheta){

	Eigen::Vector3d vo, wo, z, pi_j ;
	Eigen::Matrix3d Roti_j; // j > i 
	vo.setZero(); 
	wo.setZero();
	z << 0, 0, 1;

	for (int i(0); i<ndof_; i++){
		stacked_linear_velocity_[i].setZero();
		stacked_angular_velocity_[i].setZero();
	}
	for (int i(0); i<ndof_; i++){

		Roti_j = TransformationMatrixList_[i].block(0,0,3,3);
		pi_j = TransformationMatrixList_[i].block(0,3,3,1) ;

		if (i == 0){
			stacked_angular_velocity_[i] =  wo + dtheta(i)* z;
			stacked_linear_velocity_[i] =Roti_j.transpose()*(vo + skewSymmetricMatrix(wo)*TransformationMatrixList_[i].block(0,3,3,1));
		}else
		{
		
  			stacked_angular_velocity_[i] = Roti_j.transpose()*stacked_angular_velocity_[i-1] + dtheta[i]* z;
			stacked_linear_velocity_[i] =  Roti_j.transpose()*(stacked_linear_velocity_[i-1] + skewSymmetricMatrix(stacked_angular_velocity_[i-1])*pi_j);


		}
		
	}
	// Roti_j = TransformationMatrixList_[ndof_].block(0,0,3,3);
	// pi_j = TransformationMatrixList[ndof_].block(0,3,3,1) ;
	// stacked_angular_velocity_[ndof_].setZero();
	// stacked_linear_velocity_[ndof_] =  Roti_j.transpose()*(stacked_linear_velocity_[ndof_-1] + skewSymmetricMatrix(stacked_angular_velocity_[ndof_-1])*pi_j);

}


void HumanMotion::computeJacobian(){

	 Eigen::Matrix3d Rot;
	 Eigen::Matrix4d Ton;
	 Eigen::Vector4d Ptilde, Pe, Pi;	
	 Eigen::Vector3d zo;
	 zo << 0, 0, 1; 
	 Ptilde << 0, 0, 0, 1;
	 Ton = TransformationMatrixToBaseList_[ndof_];
	 for (int i(0); i < ndof_; i++){
		 // Linear velocity part 
		 Rot = TransformationMatrixToBaseList_[i].block(0,0,3,3);
		 Pe = Ton*Ptilde;
		 Pi = TransformationMatrixToBaseList_[i]*Ptilde;
		 Jacobian_.block(0,i,3,1) =  skewSymmetricMatrix(Rot*zo)*(Pe.segment(0,3)- Pi.segment(0,3));
		 // Angular velocity part 
		 Jacobian_.block(3,i,3,1) =  Rot*zo;
	 }
}

Eigen::MatrixXd HumanMotion::computeJacobian(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList){

	 Eigen::Matrix3d Rot;
	 Eigen::Matrix4d Ton;
	 Eigen::Vector4d Ptilde, Pe, Pi;	
	 Eigen::Vector3d zo;
	 zo << 0, 0, 1; 
	 Ptilde << 0, 0, 0, 1;
	 Ton = TransformationMatrixToBaseList[ndof_];

	 Eigen::MatrixXd Jacobian;
	 Jacobian.resize(6,ndof_);
	 Jacobian.setZero();
	 for (int i(0); i < ndof_; i++){
		 // Linear velocity part 
		 Rot = TransformationMatrixToBaseList[i].block(0,0,3,3);
		 Pe = Ton*Ptilde;
		 Pi = TransformationMatrixToBaseList[i]*Ptilde;
		 Jacobian.block(0,i,3,1) =  skewSymmetricMatrix(Rot*zo)*(Pe.segment(0,3)- Pi.segment(0,3));
		 // Angular velocity part 
		 Jacobian.block(3,i,3,1) =  Rot*zo;
	 }

	 return Jacobian;
}

void HumanMotion::computeJacobianC1(){

	 Eigen::Matrix3d Rot;
	 Eigen::Matrix4d ToC1;
	 Eigen::Vector4d Ptilde, PC1, Pi;	
	 Eigen::Vector3d zo;
	 zo << 0, 0, 1; 
	 Ptilde << 0, 0, 0, 1;

	 ToC1 = TransformationMatrixToBaseList_[2]*T3C1_;
	 for (int i(0); i < 3; i++){
		 // Linear velocity part 
		 Rot = TransformationMatrixToBaseList_[i].block(0,0,3,3);
		 PC1 = ToC1*Ptilde;
		 Pi = TransformationMatrixToBaseList_[i]*Ptilde;
		 Jacobian_C1_.block(0,i,3,1) =  skewSymmetricMatrix(Rot*zo)*(PC1.segment(0,3)- Pi.segment(0,3));
		 // Angular velocity part 
		 Jacobian_C1_.block(3,i,3,1) =  Rot*zo;
	 }
}
Eigen::MatrixXd HumanMotion::computeJacobianC1(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList){

	 Eigen::Matrix3d Rot;
	 Eigen::Matrix4d ToC1;
	 Eigen::Vector4d Ptilde, PC1, Pi;	
	 Eigen::Vector3d zo;
	 zo << 0, 0, 1; 
	 Ptilde << 0, 0, 0, 1;

	 Eigen::MatrixXd Jacobian_C1;
	 Jacobian_C1.resize(6,ndof_);
	 Jacobian_C1.setZero();

	 ToC1 = TransformationMatrixToBaseList[2]*T3C1_;
	 for (int i(0); i < 3; i++){
		 // Linear velocity part 
		 Rot = TransformationMatrixToBaseList[i].block(0,0,3,3);
		 PC1 = ToC1*Ptilde;
		 Pi = TransformationMatrixToBaseList[i]*Ptilde;
		 Jacobian_C1.block(0,i,3,1) =  skewSymmetricMatrix(Rot*zo)*(PC1.segment(0,3)- Pi.segment(0,3));
		 // Angular velocity part 
		 Jacobian_C1.block(3,i,3,1) =  Rot*zo;
	 }

	 return Jacobian_C1;
}

void HumanMotion::computeJacobianC2(){

	 Eigen::Matrix3d Rot;
	 Eigen::Matrix4d ToC2;
	 Eigen::Vector4d Ptilde, PC2, Pi;	
	 Eigen::Vector3d zo;
	 zo << 0, 0, 1; 
	 Ptilde << 0, 0, 0, 1;

	 ToC2 = TransformationMatrixToBaseList_[3]*T4C2_;
	 for (int i(0); i < ndof_; i++){
		 // Linear velocity part 
		 Rot = TransformationMatrixToBaseList_[i].block(0,0,3,3);
		 PC2 = ToC2*Ptilde;
		 Pi = TransformationMatrixToBaseList_[i]*Ptilde;
		 Jacobian_C2_.block(0,i,3,1) =  skewSymmetricMatrix(Rot*zo)*(PC2.segment(0,3)- Pi.segment(0,3));
		 // Angular velocity part 
		 Jacobian_C2_.block(3,i,3,1) =  Rot*zo;
	 }
}

Eigen::MatrixXd HumanMotion::computeJacobianC2(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList){

	 Eigen::Matrix3d Rot;
	 Eigen::Matrix4d ToC2;
	 Eigen::Vector4d Ptilde, PC2, Pi;	
	 Eigen::Vector3d zo;
	 zo << 0, 0, 1; 
	 Ptilde << 0, 0, 0, 1;

	 Eigen::MatrixXd Jacobian_C2;
	 Jacobian_C2.resize(6,ndof_);
	 Jacobian_C2.setZero();
	 ToC2 = TransformationMatrixToBaseList_[3]*T4C2_;
	 for (int i(0); i < ndof_; i++){
		 // Linear velocity part 
		 Rot = TransformationMatrixToBaseList_[i].block(0,0,3,3);
		 PC2 = ToC2*Ptilde;
		 Pi = TransformationMatrixToBaseList_[i]*Ptilde;
		 Jacobian_C2.block(0,i,3,1) =  skewSymmetricMatrix(Rot*zo)*(PC2.segment(0,3)- Pi.segment(0,3));
		 // Angular velocity part 
		 Jacobian_C2.block(3,i,3,1) =  Rot*zo;
	 }

	 return Jacobian_C2;
}
Eigen::MatrixXd HumanMotion::computeInertialMass(const std::vector<Eigen::Matrix4d> &TransformationMatrixToBaseList,
												 const Eigen::MatrixXd & Jacobian_c1, const Eigen::MatrixXd & Jacobian_c2){


	Eigen::MatrixXd inertialMass, M1, M2;
    Eigen::Matrix4d ToC1, ToC2;

	inertialMass.resize(ndof_,ndof_);
	M1.resize(6,6);
	M1.setIdentity();
	M2.resize(6,6);
	M2.setIdentity();

	M1.block(0,0,3,3) = M1.block(0,0,3,3)*m1_;
	M2.block(0,0,3,3) = M2.block(0,0,3,3)*m2_;

	ToC1 = TransformationMatrixToBaseList[2]*T3C1_;
	ToC2 = TransformationMatrixToBaseList[3]*T4C2_;

	M1.block(3,3,3,3) = ToC1.block(0,0,3,3)*Ic1_*ToC1.block(0,0,3,3).transpose();
	M2.block(3,3,3,3) = ToC2.block(0,0,3,3)*Ic2_*ToC2.block(0,0,3,3).transpose();

	inertialMass = Jacobian_c1.transpose()*M1*Jacobian_c1 + Jacobian_c2.transpose()*M2*Jacobian_c2;

	return inertialMass;
}

Eigen::VectorXd HumanMotion::computeForwardDynamic(const Eigen::VectorXd & dq,const Eigen::VectorXd & ddq){

	std::vector<Eigen::Matrix3d> stackedInertialMatrix;
	std::vector<Eigen::Vector3d> F, N, f, n;
	stackedInertialMatrix.resize(ndof_);
	F.resize(ndof_), N.resize(ndof_), f.resize(ndof_), n.resize(ndof_);
	stackedInertialMatrix[0].setZero();
	stackedInertialMatrix[1].setZero();
	stackedInertialMatrix[2] = Ic1_;
	stackedInertialMatrix[3] = Ic2_;

	std::vector<Eigen::Vector3d> Pcom, Vcom ;
	Pcom.resize(ndof_);
	Pcom[0] << 0, 0, 0;
	Pcom[1] << 0, 0, 0;
	Pcom[2] << lc1_, 0, 0;
	Pcom[3] << lc2_, 0, 0;

    Vcom.resize(ndof_);

	Eigen::Matrix3d Roti_j;
	Eigen::Vector3d pi_j, z;
	z << 0, 0, 1;
	Eigen::VectorXd m, torque ;
	m.resize(4), torque.resize(4);
	m << 0, 0, m1_, m2_;
	torque << 0,0,0,0;
	for(int i(0); i < ndof_; i++){
		
		Roti_j = TransformationMatrixList_[i].block(0,0,3,3);
		pi_j = TransformationMatrixList_[i].block(0,3,3,1) ;
		if (i == 0){
			stacked_angular_acc_[i] = ddq[i]*z  ;
			stacked_linear_acc_[i] << 0, -9.81 , 0;
			Vcom[i] = skewSymmetricMatrix(stacked_angular_acc_[i])*Pcom[0]  ;
			F[i] = m[i]*Vcom[i];
			N[i] = stackedInertialMatrix[0]*stacked_angular_acc_[i] 
				   + skewSymmetricMatrix(stacked_angular_velocity_[i])*stackedInertialMatrix[0]*stacked_angular_velocity_[i];
		}else{
			stacked_angular_acc_[i] = Roti_j.transpose()*stacked_angular_acc_[i-1] 
									  + skewSymmetricMatrix(Roti_j.transpose()*stacked_angular_velocity_[i-1])*dq[i]*z + ddq[i]*z ;
										
			stacked_linear_acc_[i] = Roti_j.transpose()*(skewSymmetricMatrix(stacked_angular_acc_[i-1])*pi_j + 
									 skewSymmetricMatrix(stacked_angular_acc_[i-1])*skewSymmetricMatrix(stacked_angular_acc_[i-1])*pi_j + stacked_linear_acc_[i-1]);
			
			
			Vcom[i] = skewSymmetricMatrix(stacked_angular_acc_[i])*Pcom[i] +
					 skewSymmetricMatrix(stacked_angular_acc_[i])*skewSymmetricMatrix(stacked_angular_acc_[i])*Pcom[i] +stacked_linear_acc_[i];			 ;
			F[i] = m[i]*Vcom[i] ;
			N[i] = stackedInertialMatrix[0]*stacked_angular_acc_[i]+ skewSymmetricMatrix(stacked_angular_acc_[i])*stackedInertialMatrix[i]*stacked_angular_acc_[i];
		}
	}

	for (int i(3); i>-1; i-- ){
		Roti_j = TransformationMatrixList_[i].block(0,0,3,3);
		pi_j = TransformationMatrixList_[i].block(0,3,3,1) ;
		if (i==3){
			f[i] = F[i];
			n[i] = N[i] +  skewSymmetricMatrix(Pcom[i])*F[i];
			 torque[i] = n[i].transpose()*z; 
		}else
		{
			f[i] = Roti_j*f[i+1] + F[i];
			n[i] = N[i] + Roti_j*n[i+1] + skewSymmetricMatrix(Pcom[i])*F[i]
				  + skewSymmetricMatrix(pi_j)*Roti_j*f[i+1] ;
			torque[i] = n[i].transpose()*z; 	
		}
		
	}
	return torque;
}
Eigen::VectorXd HumanMotion::computeCoriolisGravity(const Eigen::VectorXd & dq){

	std::vector<Eigen::Matrix3d> stackedInertialMatrix;
	std::vector<Eigen::Vector3d> F, N, f, n;
	stackedInertialMatrix.resize(ndof_);
	F.resize(ndof_), N.resize(ndof_), f.resize(ndof_), n.resize(ndof_);
	stackedInertialMatrix[0].setZero();
	stackedInertialMatrix[1].setZero();
	stackedInertialMatrix[2] = Ic1_;
	stackedInertialMatrix[3] = Ic2_;

	std::vector<Eigen::Vector3d> Pcom, Vcom ;
	Pcom.resize(ndof_);
	Pcom[0] << 0, 0, 0;
	Pcom[1] << 0, 0, 0;
	Pcom[2] << lc1_, 0, 0;
	Pcom[3] << lc2_, 0, 0;

    Vcom.resize(ndof_);

	Eigen::Matrix3d Roti_j;
	Eigen::Vector3d pi_j, z;
	z << 0, 0, 1;
	Eigen::VectorXd m, ddq, H ;
	m.resize(4), ddq.resize(4);
	m << 0, 0, m1_, m2_;
	ddq << 0,0,0,0;
	H.resize(4);
	H << 0,0, 0,0;
	for(int i(0); i < ndof_; i++){
		
		Roti_j = TransformationMatrixList_[i].block(0,0,3,3);
		pi_j = TransformationMatrixList_[i].block(0,3,3,1) ;
		if (i == 0){
			stacked_angular_acc_[i] = ddq[i]*z  ;
			stacked_linear_acc_[i] << 0, -9.81 , 0;
			Vcom[i] = skewSymmetricMatrix(stacked_angular_acc_[i])*Pcom[0]  ;
			F[i] = m[i]*Vcom[i];
			N[i] = stackedInertialMatrix[0]*stacked_angular_acc_[i] 
				   + skewSymmetricMatrix(stacked_angular_velocity_[i])*stackedInertialMatrix[0]*stacked_angular_velocity_[i];
		}else{
			stacked_angular_acc_[i] = Roti_j.transpose()*stacked_angular_acc_[i-1] 
									  + skewSymmetricMatrix(Roti_j.transpose()*stacked_angular_velocity_[i-1])*dq[i]*z + ddq[i]*z ;
										
			stacked_linear_acc_[i] = Roti_j.transpose()*(skewSymmetricMatrix(stacked_angular_acc_[i-1])*pi_j + 
									 skewSymmetricMatrix(stacked_angular_acc_[i-1])*skewSymmetricMatrix(stacked_angular_acc_[i-1])*pi_j + stacked_linear_acc_[i-1]);
			
			
			Vcom[i] = skewSymmetricMatrix(stacked_angular_acc_[i])*Pcom[i] +
					 skewSymmetricMatrix(stacked_angular_acc_[i])*skewSymmetricMatrix(stacked_angular_acc_[i])*Pcom[i] +stacked_linear_acc_[i];			 ;
			F[i] = m[i]*Vcom[i] ;
			N[i] = stackedInertialMatrix[0]*stacked_angular_acc_[i]+ skewSymmetricMatrix(stacked_angular_acc_[i])*stackedInertialMatrix[i]*stacked_angular_acc_[i];
		}
	}

	for (int i(3); i>-1; i-- ){
		Roti_j = TransformationMatrixList_[i].block(0,0,3,3);
		pi_j = TransformationMatrixList_[i].block(0,3,3,1) ;
		if (i==3){
			f[i] = F[i];
			n[i] = N[i] +  skewSymmetricMatrix(Pcom[i])*F[i];
			H[i] = n[i].transpose()*z; 
		}else
		{
			f[i] = Roti_j*f[i+1] + F[i];
			n[i] = N[i] + Roti_j*n[i+1] + skewSymmetricMatrix(Pcom[i])*F[i]
				  + skewSymmetricMatrix(pi_j)*Roti_j*f[i+1] ;
			H[i] = n[i].transpose()*z; 	
		}
		
	}
	return H;
}
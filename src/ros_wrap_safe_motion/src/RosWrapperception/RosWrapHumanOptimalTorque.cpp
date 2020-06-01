#include "RosWrapperception/RosWrapHumanOptimalTorque.h"


HumanTorque::HumanTorque(const double & dt): dt_(dt){

    n_ = 4;

    inertial_matrix_.resize(n_,n_), inertial_matrix_.setIdentity();
    M_inv_.resize(n_,n_), M_inv_ = inertial_matrix_.inverse();
    coriolis_.resize(n_), coriolis_.setZero();
    gravity_.resize(n_), gravity_.setZero();
    optimal_torque_.resize(n_), optimal_torque_.setZero();
    J_.resize(3,n_), J_.setIdentity();
    ddX_prec_.resize(3), ddX_prec_.setZero();
    ddq_prec_.resize(n_), ddq_prec_.setZero();
    torque_prec_.resize(n_),torque_prec_.setZero();
    q_prec_.resize(n_), q_prec_.setZero();
    dq_prec_.resize(n_), dq_prec_.setZero();

    for (int opt_obj_ite = hand_jerk; opt_obj_ite!=energy+1; opt_obj_ite++){

         optimal_objective_H_[(static_cast<optimal_objective>(opt_obj_ite))];
         optimal_objective_g_[(static_cast<optimal_objective>(opt_obj_ite))];
    }
 
    Cref_.resize(7), Cref_ << 206.2, 2134.8, 40.2, 43, 31, 0.2, 39 ;
    weight_.resize(7), weight_ << 0.3, 1.0, 0.1, 0., 0., 5.2, 5.8 ;
    H_.resize(n_,n_), H_.setIdentity();
    g_.resize(n_), g_.setZero();
    lb_.resize(n_), lb_.setConstant(-1000);
    ub_.resize(n_), ub_.setConstant(1000);
    lbA_.resize(n_), lbA_.setConstant(-1000);
    ubA_.resize(n_), ubA_.setConstant(1000);
    A_.resize(n_,n_), A_.setZero();

    options_.enableFlippingBounds = qpOASES::BT_FALSE;
    options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options_.enableEqualities = qpOASES::BT_TRUE; //
    options_.numRefinementSteps = 10;
    qp_human_torque_.reset(new qpOASES::SQProblem(n_,n_,qpOASES::HST_SEMIDEF));
    qp_human_torque_->setPrintLevel(qpOASES::PL_DEBUG_ITER);
    
}

void HumanTorque::update_dynamic_parameters(const Eigen::MatrixXd & J, 
                                   const Eigen::VectorXd & ddX_prec,
                                   const Eigen::VectorXd & ddq_prec,
                                   const Eigen::VectorXd & torque_prec,
                                   const Eigen::VectorXd & q_prec,
                                   const Eigen::VectorXd & dq_prec,
                                   const Eigen::MatrixXd & inertial_matrix,
                                   const Eigen::VectorXd & coriolis,
                                   const Eigen::VectorXd & gravity){


    inertial_matrix_ =  inertial_matrix;
    coriolis_ = coriolis;
    gravity_ = gravity;                 
    J_ = J;
    ddX_prec_ = ddX_prec ;
    ddq_prec_ = ddq_prec;
    torque_prec_ = torque_prec;
    q_prec_ = q_prec;
    dq_prec_ = dq_prec;

}


void HumanTorque::compute_hand_jerk(){
    
    std::cout <<"Computing hand jerk objective " <<std::endl;
    Eigen::MatrixXd E, H;
    Eigen::VectorXd f, g;

    E = J_*M_inv_/dt_ ;
    f = -(J_*coriolis_ + J_*gravity_ + ddX_prec_)/dt_;
    compute_H_g(H, g,E,f);

    optimal_objective_H_[hand_jerk] = H;
    optimal_objective_g_[hand_jerk] = g;


}

void HumanTorque::compute_joint_jerk(){
    std::cout <<"Computing joint jerk objective " <<std::endl;

    Eigen::MatrixXd E, H;
    Eigen::VectorXd f, g;
    E.resize(n_,n_), H.resize(n_,n_), f.resize(n_), g.resize(n_);

    E = M_inv_/dt_;
    f = -(coriolis_ + gravity_ + ddq_prec_)/dt_;
    compute_H_g(H, g,E,f);

    optimal_objective_H_[joint_jerk] = H;
    optimal_objective_g_[joint_jerk] = g;
}

void HumanTorque::compute_joint_acceleration(){
    
    std::cout <<"Computing joint acceleration objective " <<std::endl;

    Eigen::MatrixXd E, H;
    Eigen::VectorXd f, g;
    E.resize(n_,n_), H.resize(n_,n_), f.resize(n_), g.resize(n_);

    E = M_inv_;
    f = -(coriolis_ + gravity_);
    compute_H_g(H, g,E,f);

    optimal_objective_H_[joint_ddq] = H;
    optimal_objective_g_[joint_ddq] = g;
}

void HumanTorque::compute_torque_change(){
    std::cout <<"Computing torque change  objective " <<std::endl;

    Eigen::MatrixXd E, H;
    Eigen::VectorXd f, g;
    E.resize(n_,n_), H.resize(n_,n_), f.resize(n_), g.resize(n_);

    E.setIdentity();
    f = -torque_prec_;
    compute_H_g(H, g,E,f);
    optimal_objective_H_[torque_variation] = H;
    optimal_objective_g_[torque_variation] = g;    
}

void HumanTorque::compute_minimal_torque(){
    std::cout <<"Computing minimal torque objective " <<std::endl;

    Eigen::MatrixXd E, H;
    Eigen::VectorXd f, g;
    E.resize(n_,n_), H.resize(n_,n_), f.resize(n_), g.resize(n_);

    E.setIdentity();
    f.setZero();
    compute_H_g(H, g,E,f);

    optimal_objective_H_[torque] = H;
    optimal_objective_g_[torque] = g; 

}

void HumanTorque::compute_geodesis(){

        std::cout <<"Computing geodesis objective " <<std::endl;

    Eigen::MatrixXd E, H, a;
    Eigen::VectorXd f, g, b;

    a.resize(n_,n_),b.resize(n_);
    E.resize(n_,n_), H.resize(n_,n_), f.resize(n_), g.resize(n_);

    a = 0.5*dt_*dt_*M_inv_;
    b = q_prec_ + dq_prec_ - 0.5*dt_*dt_*(coriolis_ + gravity_);

    E = a;
    f = b;
    
    H = E.transpose()*inertial_matrix_*E;
    g = E.transpose()*inertial_matrix_*f;

    optimal_objective_H_[geodesis] = H;
    optimal_objective_g_[geodesis] = g;
             
}

void HumanTorque::compute_energy(){

    std::cout <<"Computing energy objective " <<std::endl;

    Eigen::MatrixXd  H;
    Eigen::VectorXd  g;
    H.resize(n_,n_), g.resize(n_);

    H = dq_prec_ * dq_prec_.transpose();
    g.setZero();

    optimal_objective_H_[energy] = H;
    optimal_objective_g_[energy] = g;

}

void HumanTorque::update_objective(){
    std::cout <<" ===================Updating Objective===================" << std::endl;

     for (auto it_obj = optimal_objective_H_.begin() ; it_obj != optimal_objective_H_.end(); it_obj++){
         std::cout <<" updating " << it_obj->first << '\n';

         HumanTorque::get_update_objective(it_obj->first);

         std::cout << it_obj->first << " updated " << '\n';
     }    

    std::cout <<" =================== All objective updated ===================" << std::endl;

}

void HumanTorque::compute_total_objective(){
    std::cout <<"=================== Computing total objective ======================" << std::endl;
    int i=0;
    for (auto it_obj = optimal_objective_H_.begin(); it_obj != optimal_objective_H_.end(); it_obj++){

         H_ += weight_[i] * (it_obj -> second) / Cref_[i]; 
    }

    for (auto it_obj = optimal_objective_g_.begin(); it_obj != optimal_objective_g_.end(); it_obj++){

         g_ += weight_[i] * (it_obj -> second) / Cref_[i]; 
         i++;

    }
    
    std::cout <<"=================== total objective computed ======================" << std::endl;

}   

bool HumanTorque::solve(){

    qpOASES::returnValue ret;
    qpOASES::int_t nWSR = 1000000;
    static bool qpoases_initialized = false;
    qp_human_torque_->setPrintLevel(qpOASES::PL_HIGH);
    qp_human_torque_->setOptions(options_);

    std::cout << "H_ :\n " << H_ <<  std::endl; 
    std::cout << "g_ :\n " << g_ << std::endl; 
    std::cout << "A_ :\n " << A_ << std::endl; 
    std::cout << "lb_ :\n " << lb_ << std::endl; 
    std::cout << "ub_ :\n " << ub_ << std::endl; 
    std::cout << "lbA_ :\n " << lbA_ << std::endl; 
    std::cout << "ubA_ :\n " << ubA_ << std::endl; 

    ret = qp_human_torque_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

    qp_human_torque_->printProperties();

    if(ret == qpOASES::SUCCESSFUL_RETURN){
         // Get the solution
       qp_human_torque_->getPrimalSolution(optimal_torque_.data());
       return true;
    }else{
       ROS_INFO(" qpOASES failed to find a solution ");
       return false;
    }
}
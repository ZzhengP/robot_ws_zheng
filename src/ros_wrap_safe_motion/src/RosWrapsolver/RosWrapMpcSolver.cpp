#include "RosWrapsolver/RosWrapMpcSolver.h"

using namespace qpOASES;
enum  	returnValue {
  TERMINAL_LIST_ELEMENT = -1, SUCCESSFUL_RETURN = 0, RET_DIV_BY_ZERO, RET_INDEX_OUT_OF_BOUNDS,
  RET_INVALID_ARGUMENTS, RET_ERROR_UNDEFINED, RET_WARNING_UNDEFINED, RET_INFO_UNDEFINED,
  RET_EWI_UNDEFINED, RET_AVAILABLE_WITH_LINUX_ONLY, RET_UNKNOWN_BUG, RET_PRINTLEVEL_CHANGED,
  RET_NOT_YET_IMPLEMENTED, RET_INDEXLIST_MUST_BE_REORDERD, RET_INDEXLIST_EXCEEDS_MAX_LENGTH, RET_INDEXLIST_CORRUPTED,
  RET_INDEXLIST_OUTOFBOUNDS, RET_INDEXLIST_ADD_FAILED, RET_INDEXLIST_INTERSECT_FAILED, RET_INDEX_ALREADY_OF_DESIRED_STATUS,
  RET_ADDINDEX_FAILED, RET_REMOVEINDEX_FAILED, RET_SWAPINDEX_FAILED, RET_NOTHING_TO_DO,
  RET_SETUP_BOUND_FAILED, RET_SETUP_CONSTRAINT_FAILED, RET_MOVING_BOUND_FAILED, RET_MOVING_CONSTRAINT_FAILED,
  RET_SHIFTING_FAILED, RET_ROTATING_FAILED, RET_QPOBJECT_NOT_SETUP, RET_QP_ALREADY_INITIALISED,
  RET_NO_INIT_WITH_STANDARD_SOLVER, RET_RESET_FAILED, RET_INIT_FAILED, RET_INIT_FAILED_TQ,
  RET_INIT_FAILED_CHOLESKY, RET_INIT_FAILED_HOTSTART, RET_INIT_FAILED_INFEASIBILITY, RET_INIT_FAILED_UNBOUNDEDNESS,
  RET_INIT_FAILED_REGULARISATION, RET_INIT_SUCCESSFUL, RET_OBTAINING_WORKINGSET_FAILED, RET_SETUP_WORKINGSET_FAILED,
  RET_SETUP_AUXILIARYQP_FAILED, RET_NO_CHOLESKY_WITH_INITIAL_GUESS, RET_NO_EXTERN_SOLVER, RET_QP_UNBOUNDED,
  RET_QP_INFEASIBLE, RET_QP_NOT_SOLVED, RET_QP_SOLVED, RET_UNABLE_TO_SOLVE_QP,
  RET_INITIALISATION_STARTED, RET_HOTSTART_FAILED, RET_HOTSTART_FAILED_TO_INIT, RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED,
  RET_ITERATION_STARTED, RET_SHIFT_DETERMINATION_FAILED, RET_STEPDIRECTION_DETERMINATION_FAILED, RET_STEPLENGTH_DETERMINATION_FAILED,
  RET_OPTIMAL_SOLUTION_FOUND, RET_HOMOTOPY_STEP_FAILED, RET_HOTSTART_STOPPED_INFEASIBILITY, RET_HOTSTART_STOPPED_UNBOUNDEDNESS,
  RET_WORKINGSET_UPDATE_FAILED, RET_MAX_NWSR_REACHED, RET_CONSTRAINTS_NOT_SPECIFIED, RET_INVALID_FACTORISATION_FLAG,
  RET_UNABLE_TO_SAVE_QPDATA, RET_STEPDIRECTION_FAILED_TQ, RET_STEPDIRECTION_FAILED_CHOLESKY, RET_CYCLING_DETECTED,
  RET_CYCLING_NOT_RESOLVED, RET_CYCLING_RESOLVED, RET_STEPSIZE, RET_STEPSIZE_NONPOSITIVE,
  RET_SETUPSUBJECTTOTYPE_FAILED, RET_ADDCONSTRAINT_FAILED, RET_ADDCONSTRAINT_FAILED_INFEASIBILITY, RET_ADDBOUND_FAILED,
  RET_ADDBOUND_FAILED_INFEASIBILITY, RET_REMOVECONSTRAINT_FAILED, RET_REMOVEBOUND_FAILED, RET_REMOVE_FROM_ACTIVESET,
  RET_ADD_TO_ACTIVESET, RET_REMOVE_FROM_ACTIVESET_FAILED, RET_ADD_TO_ACTIVESET_FAILED, RET_CONSTRAINT_ALREADY_ACTIVE,
  RET_ALL_CONSTRAINTS_ACTIVE, RET_LINEARLY_DEPENDENT, RET_LINEARLY_INDEPENDENT, RET_LI_RESOLVED,
  RET_ENSURELI_FAILED, RET_ENSURELI_FAILED_TQ, RET_ENSURELI_FAILED_NOINDEX, RET_ENSURELI_FAILED_CYCLING,
  RET_BOUND_ALREADY_ACTIVE, RET_ALL_BOUNDS_ACTIVE, RET_CONSTRAINT_NOT_ACTIVE, RET_BOUND_NOT_ACTIVE,
  RET_HESSIAN_NOT_SPD, RET_HESSIAN_INDEFINITE, RET_MATRIX_SHIFT_FAILED, RET_MATRIX_FACTORISATION_FAILED,
  RET_PRINT_ITERATION_FAILED, RET_NO_GLOBAL_MESSAGE_OUTPUTFILE, RET_DISABLECONSTRAINTS_FAILED, RET_ENABLECONSTRAINTS_FAILED,
  RET_ALREADY_ENABLED, RET_ALREADY_DISABLED, RET_NO_HESSIAN_SPECIFIED, RET_USING_REGULARISATION,
  RET_EPS_MUST_BE_POSITVE, RET_REGSTEPS_MUST_BE_POSITVE, RET_HESSIAN_ALREADY_REGULARISED, RET_CANNOT_REGULARISE_IDENTITY,
  RET_CANNOT_REGULARISE_SPARSE, RET_NO_REGSTEP_NWSR, RET_FEWER_REGSTEPS_NWSR, RET_CHOLESKY_OF_ZERO_HESSIAN,
  RET_ZERO_HESSIAN_ASSUMED, RET_CONSTRAINTS_ARE_NOT_SCALED, RET_INITIAL_BOUNDS_STATUS_NYI, RET_ERROR_IN_CONSTRAINTPRODUCT,
  RET_FIX_BOUNDS_FOR_LP, RET_USE_REGULARISATION_FOR_LP, RET_UPDATEMATRICES_FAILED, RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED,
  RET_UNABLE_TO_OPEN_FILE, RET_UNABLE_TO_WRITE_FILE, RET_UNABLE_TO_READ_FILE, RET_FILEDATA_INCONSISTENT,
  RET_OPTIONS_ADJUSTED, RET_UNABLE_TO_ANALYSE_QPROBLEM, RET_NWSR_SET_TO_ONE, RET_UNABLE_TO_READ_BENCHMARK,
  RET_BENCHMARK_ABORTED, RET_INITIAL_QP_SOLVED, RET_QP_SOLUTION_STARTED, RET_BENCHMARK_SUCCESSFUL,
  RET_NO_DIAGONAL_AVAILABLE, RET_DIAGONAL_NOT_INITIALISED, RET_ENSURELI_DROPPED, RET_SIMPLE_STATUS_P1,
  RET_SIMPLE_STATUS_P0, RET_SIMPLE_STATUS_M1, RET_SIMPLE_STATUS_M2, RET_SIMPLE_STATUS_M3
};



Eigen::MatrixXd msgPlaneToEigen(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  Eigen::MatrixXd eigenMatrix;
  int rows, cols, rows_stride, cols_stride ;

  rows = msg->layout.dim[0].size;
  cols = msg->layout.dim[1].size;

  rows_stride = msg->layout.dim[0].stride;
  cols_stride = msg->layout.dim[1].stride;

  eigenMatrix.resize(rows,cols);
  eigenMatrix.setZero();
  int ii =0;

 for (int i = 0; i < rows; ++i){
      for (int j = 0; j < cols; ++j){
          eigenMatrix(i,j) = msg->data[ii++];
      }
 }
	
  
  return eigenMatrix;
}


mpc_solve::mpc_solve(int N_horz, int Ndof, int nbrCst):N_(N_horz), Ndof_(Ndof),nC_(nbrCst)
{
    nV_ = Ndof*N_horz; // wo cao, wo ri le  !!!!!
    options_.enableFlippingBounds = qpOASES::BT_FALSE;
    options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options_.enableEqualities = qpOASES::BT_TRUE; //
    options_.numRefinementSteps = 10;
    qpoases_.reset(new qpOASES::QProblemB(nV_));
    qpoases_standard_.reset(new qpOASES::SQProblem(nV_,nbrCst,qpOASES::HST_SEMIDEF));
    qpoases_standard_->setPrintLevel(qpOASES::PL_DEBUG_ITER);
    optimal_solution_.resize(nV_);
    optimal_solution_.setZero();
    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << BOLD(FRED("QPs solver is successively created: "))  <<'\n'
              << FYEL("The number of optimization variable is : ") << nV_ << std::endl;
    std::cout <<"---------------------------------------------" << std::endl;
}

mpc_solve::mpc_solve(ros::NodeHandle* nodehandle, int N_horz, int Ndof, int nbrCst, double dt)
                     :nh_(*nodehandle),N_(N_horz), Ndof_(Ndof),nC_(nbrCst), dt_(dt)
{
    nV_ = Ndof*N_horz; // wo cao, wo ri le  !!!!!
    options_.enableFlippingBounds = qpOASES::BT_FALSE;
    options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options_.enableEqualities = qpOASES::BT_TRUE; //
    options_.numRefinementSteps = 10;
    qpoases_.reset(new qpOASES::QProblemB(nV_));
    qpoases_standard_.reset(new qpOASES::SQProblem(nV_,nbrCst,qpOASES::HST_SEMIDEF));
    qpoases_standard_->setPrintLevel(qpOASES::PL_DEBUG_ITER);
    optimal_solution_.resize(nV_);
    optimal_solution_.setZero();

    panda_joint_state_1_pub_ = nh_.advertise<std_msgs::Float64>("/panda/panda_joint1_position_controller/command", 1000);
    panda_joint_state_2_pub_ = nh_.advertise<std_msgs::Float64>("/panda/panda_joint2_position_controller/command", 1000);
    panda_joint_state_3_pub_ = nh_.advertise<std_msgs::Float64>("/panda/panda_joint3_position_controller/command", 1000);
    panda_joint_state_4_pub_ = nh_.advertise<std_msgs::Float64>("/panda/panda_joint4_position_controller/command", 1000);
    panda_joint_state_5_pub_ = nh_.advertise<std_msgs::Float64>("/panda/panda_joint5_position_controller/command", 1000);
    panda_joint_state_6_pub_ = nh_.advertise<std_msgs::Float64>("/panda/panda_joint6_position_controller/command", 1000);
    panda_joint_state_7_pub_ = nh_.advertise<std_msgs::Float64>("/panda/panda_joint7_position_controller/command", 1000);
    plane_data_subscriber_  = nh_.subscribe("plane_data",100, &mpc_solve::planeDataCallback,this );
    interactive_marker_subscriber_   = nh_.subscribe("/interactivePose",100, &mpc_solve::iMarkerCallback,this);
    goalPublisher.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/MPCgoal"));
    PredictedPathPublisher_.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/mpcPath"));        
    mpcpath_.resize(N_);      


}

void mpc_solve::planeDataCallback(const std_msgs::Float64MultiArray::ConstPtr& plane_msg){
        // Transform received plane data into Eigen matrix 
        plane_data_eigen_ = msgPlaneToEigen(plane_msg);
        plane_data_.planeLocation[0] = plane_data_eigen_;
 }


void mpc_solve::iMarkerCallback(const geometry_msgs::Pose::ConstPtr& imarker_msg){
      
        interactive_marker_pos_.position =  imarker_msg->position;

        
}


bool mpc_solve::initMPCData(Eigen::VectorXd lb, Eigen::VectorXd ub)
{
    ROS_INFO("INITIALIZE MPC SOLVER DATA ");
    optimal_solution_.resize(nV_);
    lb_ = lb;
    ub_ = ub;
    A_.resize(nC_,nV_);
    lbA_.resize(nC_);
    ubA_.resize(nC_);
    A_.setIdentity();
    lbA_.setConstant(-10000);
    ubA_.setConstant(10000);

    return true;

}

void mpc_solve::initTaskData(){

    ROS_INFO("INITIALIZE MPC TASK DATA ");

    mpc_task_ = std::make_shared<MPC_Task>(N_,1,0.001,Ndof_,dt_,"pandaTask");
    bool task_ok;
    task_ok = mpc_task_ -> init();
    
    // ------------------------------------   Initialize task parameter --------------------------------------
    state_A_ = mpc_task_ -> getStateA();
    state_B_ = mpc_task_ -> getStateB();

    panda_px_.resize(N_*Ndof_,2*Ndof_);
    panda_pu_.resize(N_*Ndof_,N_*Ndof_);
    panda_px_ = mpc_task_->getMatrixPx();
    panda_pu_ = mpc_task_ ->getMatrixPu();

    panda_pxdq_.resize(N_*Ndof_, 2*Ndof_);
    panda_pudq_.resize(N_*Ndof_,N_*Ndof_);
    panda_pxdq_ = mpc_task_->getMatrixPxdq();
    panda_pudq_ = mpc_task_->getMatrixPudq();

    panda_q_des_.resize(Ndof_);
    panda_des_frame_.p[0] = 0.7;
    panda_des_frame_.p[1] = -0.2 ;
    panda_des_frame_.p[2] = 0.1 ;

    panda_des_frame_.M = panda_des_frame_.M;
    panda_des_frame_.M.DoRotX(- 3.14);


    goalEnd_ << 0.7, -0.2, 0.1 ;
    goalStart_ << 0.3, 0.4, 0.1 ;
    goal_ = goalEnd_;

    interactive_marker_pos_.position.x = 0.7;
    interactive_marker_pos_.position.y = -0.2;
    interactive_marker_pos_.position.z = 0.1;


    // goal_ << interactive_marker_pos_.position.x, interactive_marker_pos_.position.y, interactive_marker_pos_.position.z;
    ROS_INFO("MPC TASK IS INITIALIZED ");


}

void mpc_solve::initArmData(const std::string& panda_urdf, const KDL::JntArray& panda_q_init, const KDL::JntArray& panda_dq_init){
        // Initiliaze this function after having initialize MPC task because of pandaPx, pandaPu

    ROS_INFO("INITIALIZE ARM DATA ");

    panda_urdf_ = panda_urdf;
    panda_q_ini_ = panda_q_init ;
    panda_dq_init_ = panda_dq_init;
    panda_arm_ = std::make_shared<arm_kinematic>(&nh_,panda_urdf_, Ndof_, "panda_link0","panda_link8");

    panda_arm_->init(panda_q_ini_.data, panda_dq_init_.data, N_);
    panda_state_.resize(2*Ndof_);
    panda_state_ = panda_arm_->getRobotState();

    std::cout <<" panda_sate_ :\n "<< panda_state_ << std::endl; 
    jacobian_.resize(6,Ndof_);
    panda_arm_ -> computeJacobian();
    jacobian_ = panda_arm_ -> getJacobian().data;

    dq_.resize(Ndof_);
    dq_ = panda_arm_ ->getJointVel().data;
    q_.resize(Ndof_);
    q_ = panda_arm_ -> getJointPos().data;

    q_horizon_.resize(N_*Ndof_);
    q_horizon_ = panda_px_*panda_state_ + panda_pu_*optimal_solution_;
    panda_arm_ -> setJointHorizon(q_horizon_);

    dq_horizon_.resize(N_*Ndof_);
    dq_horizon_ = panda_pxdq_*panda_state_ + panda_pudq_*optimal_solution_;
    panda_arm_ -> setJointVelHorizon(dq_horizon_);

    jacobian_horizon_.resize(N_*6,N_*Ndof_);
    panda_arm_ -> computeJacobianHorz(q_horizon_);
    jacobian_horizon_ = panda_arm_ -> getJacobianHorz();

    ee_pos_horizon_.resize(3*N_);
    panda_arm_ -> computeCartPosHorz();
    ee_pos_horizon_ = panda_arm_ -> getCartPosHorz();

    panda_arm_ -> computeJntFromCart(panda_des_frame_,panda_q_des_);

    q_horizon_des_.resize(N_*Ndof_);
    for (size_t i=0; i<N_; i++)
    {
        q_horizon_des_.segment(Ndof_*i,Ndof_) = panda_q_des_.data;
    }

    mpc_task_->computeHandg(jacobian_horizon_,panda_state_,q_horizon_des_);
    H_ = mpc_task_->getMatrixH();
    g_ = mpc_task_ ->getVectorg();
    
    panda_ee_frame_ = panda_arm_ -> getSegmentPosition(8);
    ROS_INFO("ARM DATA IS INITIALIZED ");

}   


void mpc_solve::initRobotConstraintData(const Eigen::VectorXd &ddq_min, const Eigen::VectorXd &ddq_max,
                                        const Eigen::VectorXd &dq_min, const Eigen::VectorXd &dq_max, 
                                        const Eigen::VectorXd &q_min, const Eigen::VectorXd &q_max,
                                        const Eigen::VectorXd &v_min, const Eigen::VectorXd &v_max){

    ROS_INFO("INITIALIZE MPC CONSTRAINT DATA ");

    ddq_min_.resize(N_*Ndof_), ddq_max_.resize(N_*Ndof_) ;
    dq_min_.resize(N_*Ndof_), dq_max_.resize(N_*Ndof_) ;
    q_min_.resize(N_*Ndof_), q_max_.resize(N_*Ndof_) ;

    ddq_min_ = ddq_min, ddq_max_ = ddq_max;
    dq_min_ = dq_min, dq_max_ = dq_max;
    dq_min_.tail(N_).setZero(), dq_max_.tail(N_).setZero();
    q_min_ = q_min, q_max_ = q_max ;
    
    dsafe_ = 0.2/2;
    // ----------------------------------------------   Initialize robot's intrinsec constraints ---------------------------------------------------------
        // Initiliaze this function after having initialize MPC task because of pandaPx, pandaPu
    jnt_pos_cst_ = std::make_shared<JntPosCst>(Ndof_, N_, dt_, "JointPositionConstraint",panda_px_, panda_pu_);
    jnt_vel_cst_ = std::make_shared<JntVelCst>(Ndof_, N_, dt_, "JointVelocityConstraint",panda_pxdq_, panda_pudq_);
    jnt_acc_cst_ = std::make_shared<JntAccCst>(Ndof_, N_, dt_, "JointAccelerationConstraint",panda_px_, panda_pu_);
    obs_avoidance_cst_ = std::make_shared<ObsAvoidanceCSt>(Ndof_,N_,dt_,dsafe_,"CollisionFreeConstraint",panda_px_,panda_pu_);
    // 1. initialize joint position constraint 
    jnt_pos_cst_ -> setLimit(q_min_,q_max_);
    jnt_pos_cst_ -> setLowerBound(panda_state_);
    jnt_pos_cst_ -> setUpperBound(panda_state_);
    jnt_pos_cst_ -> setConstraintMatrix();
    constraint_number_++ ;
    // 2. initialize joint velocity constraint 
    jnt_vel_cst_ -> setLimit(dq_min_, dq_max_);
    jnt_vel_cst_ -> setLowerBound(panda_state_);
    jnt_vel_cst_ -> setUpperBound(panda_state_);
    jnt_vel_cst_ -> setConstraintMatrix();
    constraint_number_++ ;
    // 3. initialize joint acceleration constraint 
    jnt_acc_cst_ -> setLowerBound(ddq_min_, ddq_max_);

    total_constraint_data_.push_back(jnt_pos_cst_ -> getConstraintData());
    total_constraint_data_.push_back(jnt_vel_cst_ -> getConstraintData());
  

    // ----------------------------------------------   Initialize collision free trajectory constraints ---------------------------------------------------------
    table_avoidance_cst_ = std::make_shared<ObsAvoidanceCSt>(Ndof_,N_,dt_,0.1,"TableAvoidanceConstraint",panda_px_,panda_pu_);
    robot_vertices_augmented_.resize(1);
    robot_vertices_augmented_[0].resize(3, 2*N_);

    for (size_t i=0; i < N_; i++)
    {
         robot_vertices_augmented_[0].block(0,2*i,3,2) << 0.5, 0.5,
                                                          0.3, 0.3,
                                                          0.1, 0.1;
    }   
    
    table_planes_.N = N_;
    table_planes_.nbrPlane = 1 ;
    table_planes_.planeLocation.resize(1);
    table_planes_.planeLocation[0].resize(5,N_-1);

    for (int k(0); k< N_-1; k++){
        table_planes_.planeLocation[0].block(0,k,5,1) << 0,0,-1,0,0;
    }

    table_avoidance_cst_ ->setLowerBound();
    table_avoidance_cst_ ->setUpperBoundAndConstraint(robot_vertices_augmented_,table_planes_,panda_state_,jacobian_horizon_,q_horizon_);
    constraint_number_++;
    
    total_constraint_data_.push_back(table_avoidance_cst_ -> getConstraintData());

    // ------ Initialize Plane 
    int nbr_of_plane = 1;
    plane_data_.N = N_;
    plane_data_.nbrPlane = nbr_of_plane;

    plane_data_.planeLocation.resize(nbr_of_plane);

    for (int i = 0; i < nbr_of_plane; i++) {

        if (N_ == 1){
             plane_data_.planeLocation[i].resize(5,N_);
        }else {
             plane_data_.planeLocation[i].resize(5,N_-1);

        }
            for (int j(0); j < N_-1 ; j++){
                plane_data_.planeLocation[i].block(0,j,5,1)<< 1,0,0,0.8,0;
            }
    }


    obs_avoidance_cst_ -> setLowerBound();
    obs_avoidance_cst_ -> setUpperBoundAndConstraint(robot_vertices_augmented_,plane_data_,panda_state_,jacobian_horizon_,q_horizon_);
    constraint_number_++ ;
    total_constraint_data_.push_back(obs_avoidance_cst_ -> getConstraintData());



}

    

void mpc_solve::update(){

        Eigen::Vector3d cartesian_error;
        currentEEpose_ << panda_ee_frame_.p[0], panda_ee_frame_.p[1], panda_ee_frame_.p[2];
        cartesian_error = currentEEpose_ - goal_;

        if (cartesian_error.norm() < 0.01){

            if (goal_ == goalStart_){
                goal_ = goalEnd_;
            }else {
                goal_ = goalStart_;
            }

        }
        

        // goal_ << interactive_marker_pos_.position.x, interactive_marker_pos_.position.y, interactive_marker_pos_.position.z;

        panda_des_frame_.p[0] = goal_(0);
        panda_des_frame_.p[1] = goal_(1);
        panda_des_frame_.p[2] = goal_(2);

        panda_arm_ -> computeJntFromCart(panda_des_frame_,panda_q_des_);
        for (size_t i=0; i<N_ ; i++){
            q_horizon_des_.segment(Ndof_*i,Ndof_) = panda_q_des_.data;
        }

        mpc_task_ -> computeHandg(jacobian_horizon_, panda_state_, q_horizon_des_);
        H_ = mpc_task_ -> getMatrixH();
        g_ = mpc_task_ -> getVectorg();

        panda_arm_ -> computeJacobian();
        jacobian_ = panda_arm_ ->getJacobian().data;
        dq_ = panda_arm_ ->getJointVel().data;

        panda_arm_ -> computeJacobianHorz(q_horizon_);
        jacobian_horizon_ = panda_arm_ -> getJacobianHorz();

        // Update joint position constraint
        jnt_pos_cst_ -> update(panda_state_);
        // Update vel position constraint
        jnt_vel_cst_ -> update(panda_state_);
        // Update obstacle avoidance constraint
        table_avoidance_cst_ -> setUpperBoundAndConstraint(robot_vertices_augmented_,table_planes_,panda_state_,jacobian_horizon_,q_horizon_);
        // Update collision free constraint 
        obs_avoidance_cst_ -> setUpperBoundAndConstraint(robot_vertices_augmented_,plane_data_,panda_state_,jacobian_horizon_,q_horizon_);


        total_constraint_data_[0] = jnt_pos_cst_ -> getConstraintData();
        total_constraint_data_[1] = jnt_vel_cst_ -> getConstraintData();
        total_constraint_data_[2] = table_avoidance_cst_ -> getConstraintData();
        total_constraint_data_[3] = obs_avoidance_cst_ -> getConstraintData();


        constructProblem();
        bool is_solved ;
        is_solved = solve();
         if (!is_solved){
            std::cout <<" qp failed :\n" << std::endl;
            optimal_solution_.setZero();

        }else {
                 std::cout <<" qp success :\n" << std::endl;

                optimal_solution_ = getSolution();

}


        q_horizon_ = panda_px_*panda_state_ + panda_pu_*optimal_solution_;
        panda_arm_ -> setJointHorizon(q_horizon_);

        dq_horizon_ = panda_pxdq_*panda_state_ + panda_pudq_*optimal_solution_;
        panda_arm_ -> setJointVelHorizon(dq_horizon_);


        // panda_arm_ -> computeJntPosHorz(optimal_solution_);
        
        panda_arm_ -> computeCartPosHorz();
        
        ee_pos_horizon_ = panda_arm_ -> getCartPosHorz();
        panda_arm_ -> verticesPublisher();

        for (int i(0);i<N_;i++){
            robot_vertices_augmented_[0].block(0,2*i,3,1) << ee_pos_horizon_.segment(3*i,3);
            robot_vertices_augmented_[0].block(0,2*i+1,3,1) << ee_pos_horizon_.segment(3*i,3);
            mpcpath_[i].position.x =  ee_pos_horizon_(3*i);
            mpcpath_[i].position.y =  ee_pos_horizon_(3*i+1);
            mpcpath_[i].position.z =  ee_pos_horizon_(3*i+2);
        }

        panda_ee_frame_ = panda_arm_ ->getSegmentPosition(8);
        panda_state_ = state_A_ * panda_state_ + state_B_*optimal_solution_.segment(0,Ndof_);
        panda_arm_ -> setState(panda_state_.head(Ndof_),panda_state_.tail(Ndof_));

        std_msgs::Float64 panda_t1,panda_t2,panda_t3,panda_t4,panda_t5,panda_t6, panda_t7 ; // robot joint's positions

        panda_t1.data = panda_state_[0];
        panda_t2.data = panda_state_[1];
        panda_t3.data = panda_state_[2];
        panda_t4.data = panda_state_[3];
        panda_t5.data = panda_state_[4];
        panda_t6.data = panda_state_[5];
        panda_t7.data = panda_state_[6];


        panda_joint_state_1_pub_.publish(panda_t1);
        panda_joint_state_2_pub_.publish(panda_t2);
        panda_joint_state_3_pub_.publish(panda_t3);
        panda_joint_state_4_pub_.publish(panda_t4);
        panda_joint_state_5_pub_.publish(panda_t5);
        panda_joint_state_6_pub_.publish(panda_t6);
        panda_joint_state_7_pub_.publish(panda_t7);

        rviz_visual_tools::colors colorGoal = rviz_visual_tools::GREEN;
        rviz_visual_tools::colors colorPath = rviz_visual_tools::RED;

        goalPublisher->publishSphere(goalStart_,colorGoal,0.05);
        goalPublisher->publishSphere(goalEnd_,colorGoal,0.05);
        goalPublisher->trigger();   

        PredictedPathPublisher_->publishPath(mpcpath_,colorPath);
 
        PredictedPathPublisher_->trigger();
        PredictedPathPublisher_->deleteAllMarkers();
}

void mpc_solve::run(){
    ROS_INFO( "Start running");


    ros::Rate r(1000);// this node will work at 100hz  
    while (ros::ok()){

        update();
        ros::spinOnce();
        r.sleep();
    }
}

bool mpc_solve::solve(Eigen::MatrixXd H, Eigen::VectorXd g,Eigen::VectorXd lb, Eigen::VectorXd ub){
     qpOASES::returnValue ret;
     qpOASES::int_t nWSR = 1000000;
     H_ = H;
     g_ = g;
     lb_ = lb;
     ub_ = ub;

     qpoases_->setOptions(options_);
      ret = qpoases_->init(H.data(),g.data(),lb.data(),ub.data(),nWSR,0);
      if (ret == qpOASES::SUCCESSFUL_RETURN){
     qpoases_->getPrimalSolution(optimal_solution_.data());
             return true;}
      else{
            return false;
      }
}

bool mpc_solve::solve(){
     qpOASES::returnValue ret;
     qpOASES::returnValue ret2;

     qpOASES::int_t nWSR = 1000000;
     static bool qpoases_initialized = false;
     qpoases_standard_->setPrintLevel(qpOASES::PL_HIGH);
//     if(!qpoases_initialized)
//           {
         // Initialise the problem, once it has found a solution, we can hotstart

         qpoases_standard_->setOptions(options_);

//         ret = qpoases_standard_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR,0,data_optimal_solution_.data(),0);
         ret = qpoases_standard_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

         qpoases_standard_->printProperties();

         // Keep init if it didn't work
//         if(ret == qpOASES::SUCCESSFUL_RETURN)
//         {
//           qpoases_initialized = true;
//         }
//           }
//           else{
//         // Otherwise let's reuse the previous solution to find a solution faster
//         ret = qpoases_standard_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

//         if(ret != qpOASES::SUCCESSFUL_RETURN){
//           qpoases_initialized = false;
//         }
//           }

           // Zero grav if not found
           // TODO: find a better alternative

           if(ret == qpOASES::SUCCESSFUL_RETURN){
         // Get the solution
                qpoases_standard_->getPrimalSolution(optimal_solution_.data());
                return true;
           }else{
                ROS_INFO(" qpOASES failed to find a solution ");
              return false;
           }

}



void mpc_solve::constructProblem(const std::vector<constraintData> &constraintVectorData, const Eigen::MatrixXd H, const Eigen::VectorXd g){

    int nbr_cst = constraintVectorData.size();
    int rows_cumul ;
    rows_cumul = 0;


    for (int i(0); i<nbr_cst ; i++) {
        int rows, cols;
        rows =0;
        cols = 0;
        rows = constraintVectorData[i].cstMatrix_.rows();
        cols = constraintVectorData[i].cstMatrix_.cols();

//        std::cout <<"constraint name :\n " << constraintVectorData[i].name_ << '\n';
//        std::cout <<" rows :\n " << rows <<'\n';
//        std::cout <<" cols :\n " << cols <<'\n';
//        std::cout <<" lbA_ :\n  " << constraintVectorData[i].lowBound_ <<'\n' ;
//        std::cout <<" ubA_ :\n  " << constraintVectorData[i].upBound_ <<'\n' ;
//        std::cout <<" rows cumul  :\n  " << rows_cumul <<'\n' ;

        A_.block(rows_cumul,0,rows,cols) = constraintVectorData[i].cstMatrix_;
        lbA_.segment(rows_cumul,constraintVectorData[i].lowBound_.size()) = constraintVectorData[i].lowBound_;
        ubA_.segment(rows_cumul,constraintVectorData[i].upBound_.size()) = constraintVectorData[i].upBound_;
        rows_cumul += rows;
    }

    H_ = H;
    g_ = g;

    std::ofstream myfile;
    myfile.open ("/home/zheng/Bureau/PBPproblem.txt");
   qpoases_standard_->setOptions(options_);
   myfile << "H_ \n " <<  H_ << '\n';
   myfile << "g_ \n " << g_ << '\n';
   myfile << "A_ \n " << A_ << '\n';
   myfile << "lbA_ \n " << lbA_ << '\n';
   myfile << "ubA_ \n " << ubA_ << '\n';
   myfile << "lb_ \n " << lb_ << '\n';
   myfile << "ub_ \n " << ub_ << '\n';
   myfile.close();
}



void mpc_solve::constructProblem(){

    int nbr_cst = total_constraint_data_.size();
    int rows_cumul ;
    rows_cumul = 0;


    int total_size_of_constraint = 0;
    for (int i(0); i<nbr_cst ; i++){

         total_size_of_constraint = total_size_of_constraint + total_constraint_data_[i].cstMatrix_.rows();
    }

    // for (std::map<std::string, constraintData>::iterator map_ite = constraint_data_map_.begin(); map_ite != constraint_data_map_.end() ; map_ite ++  ){

    //     total_size_of_constraint = total_size_of_constraint + map_ite ->second.cstMatrix_.rows();
    // }

    lbA_.resize(total_size_of_constraint);
    ubA_.resize(total_size_of_constraint);
    A_.resize(total_size_of_constraint,nV_);

    if (nC_ != total_size_of_constraint ){
        qpoases_standard_ -> reset();
        qpoases_standard_.reset(new qpOASES::SQProblem(nV_,total_size_of_constraint,qpOASES::HST_SEMIDEF));
        nC_ = total_size_of_constraint;
    }


    for (int i(0); i<nbr_cst ; i++) {
        int rows, cols;
        rows =0;
        cols = 0;
        rows = total_constraint_data_[i].cstMatrix_.rows();
        cols = total_constraint_data_[i].cstMatrix_.cols();

//        std::cout <<"constraint name :\n " << constraintVectorData[i].name_ << '\n';
//        std::cout <<" rows :\n " << rows <<'\n';
//        std::cout <<" cols :\n " << cols <<'\n';
//        std::cout <<" lbA_ :\n  " << constraintVectorData[i].lowBound_ <<'\n' ;
//        std::cout <<" ubA_ :\n  " << constraintVectorData[i].upBound_ <<'\n' ;
//        std::cout <<" rows cumul  :\n  " << rows_cumul <<'\n' ;

        A_.block(rows_cumul,0,rows,cols) = total_constraint_data_[i].cstMatrix_;
        lbA_.segment(rows_cumul,total_constraint_data_[i].lowBound_.size()) = total_constraint_data_[i].lowBound_;
        ubA_.segment(rows_cumul,total_constraint_data_[i].upBound_.size()) = total_constraint_data_[i].upBound_;
        rows_cumul += rows;
    }



    std::ofstream myfile;
    myfile.open ("/home/zheng/Bureau/PBPproblem.txt");
   qpoases_standard_->setOptions(options_);
   myfile << "H_ \n " <<  H_ << '\n';
   myfile << "g_ \n " << g_ << '\n';
   myfile << "A_ \n " << A_ << '\n';
   myfile << "lbA_ \n " << lbA_ << '\n';
   myfile << "ubA_ \n " << ubA_ << '\n';
   myfile << "lb_ \n " << lb_ << '\n';
   myfile << "ub_ \n " << ub_ << '\n';
   myfile.close();
}

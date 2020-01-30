#include "robotmodel/RosRobotModel.h"



arm_kinematic::arm_kinematic(ros::NodeHandle* nodehandle,const std::string& urdf_name, unsigned int NrOfDeg,const std::string& root_name, const std::string& end_name)
   :nh_(*nodehandle),NrOfDeg_(NrOfDeg){
   kdl_parser::treeFromFile(urdf_name, arm_tree_);

   arm_tree_.getChain(root_name,end_name,arm_chain_);
   q_.resize(NrOfDeg_);
   dotq_.resize(NrOfDeg_);
   robot_state_.resize(2*NrOfDeg_);
   qqd_.resize(NrOfDeg_);
   jacobian_.resize(NrOfDeg_);
   joints_name_.resize(NrOfDeg_);
   L(0)=1;L(1)=1;L(2)=1;
   L(3)=0.01;L(4)=0.01;L(5)=0.01;
   fksolver_.reset(new KDL::ChainFkSolverPos_recursive(arm_chain_));
   iksolver_.reset(new KDL::ChainIkSolverPos_LMA(arm_chain_, L));
   chainjacsolver_.reset(new KDL::ChainJntToJacSolver(arm_chain_));
   double timeout_in_secs=0.005;
   double error=1e-5;


   for (int i(0); i<NrOfDeg_ ;i++)
   {
       joints_name_[i] = arm_chain_.getSegment(i).getJoint().getName();
   }

   std::cout <<"---------------------------------------------" << std::endl;

//   std::cout << BOLD(FRED("Robot's model is successively parsed into KDL ")) << std::endl;v
   ROS_INFO("Robot's model is successively parsed into KDL");
   std::cout << FYEL("load joint : \n") <<  std::endl;
   for (int i(0); i < NrOfDeg_; i++){
      std::cout<<FCYN("Joint " ) << i << " name "<< arm_chain_.getSegment(i).getJoint().getName() <<std::endl;
   }
   std::cout <<"\n" << std::endl;
   std::cout <<FYEL("load link : \n") << std::endl;

   for (int i(0); i <arm_chain_.getNrOfSegments();i++)
   {
       std::cout<<FCYN("Link " ) << i << " name "<< arm_chain_.getSegment(i).getName() <<std::endl;
       seg_names_idx_.add(arm_chain_.getSegment(i).getName(),i);

   }
   std::cout <<"---------------------------------------------" << std::endl;
   initializeSubscriber();

}



arm_kinematic::arm_kinematic(const std::string& urdf_name, unsigned int NrOfDeg,const std::string& root_name, const std::string& end_name)
   :NrOfDeg_(NrOfDeg){
   kdl_parser::treeFromFile(urdf_name, arm_tree_);

   arm_tree_.getChain(root_name,end_name,arm_chain_);

   q_.resize(NrOfDeg_);
   dotq_.resize(NrOfDeg_);
   robot_state_.resize(2*NrOfDeg_);
   qqd_.resize(NrOfDeg_);
   jacobian_.resize(NrOfDeg_);
   joints_name_.resize(NrOfDeg_);
   L(0)=1;L(1)=1;L(2)=1;
   L(3)=0.01;L(4)=0.01;L(5)=0.01;
   fksolver_.reset(new KDL::ChainFkSolverPos_recursive(arm_chain_));
   iksolver_.reset(new KDL::ChainIkSolverPos_LMA(arm_chain_, L));
   chainjacsolver_.reset(new KDL::ChainJntToJacSolver(arm_chain_));
   for (int i(0); i<NrOfDeg_ ;i++)
   {
       joints_name_[i] = arm_chain_.getSegment(i).getName();
   }

   std::cout <<"---------------------------------------------" << std::endl;
//   std::cout << BOLD(FRED("Robot's model is successively parsed into KDL ")) << std::endl;
   ROS_INFO("Robot's model is successively parsed into KDL");
   std::cout << FYEL("load joint : \n") <<  std::endl;
   for (int i(0); i < NrOfDeg_; i++){
      std::cout<<FCYN("Joint " ) << i << " name "<< arm_chain_.getSegment(i).getJoint().getName() <<std::endl;
   }
   std::cout <<"\n" << std::endl;
   std::cout <<FYEL("load link : \n") << std::endl;

   for (int i(0); i <arm_chain_.getNrOfSegments();i++)
   {
       std::cout<<FCYN("Link " ) << i << " name "<< arm_chain_.getSegment(i).getName() <<std::endl;
       seg_names_idx_.add(arm_chain_.getSegment(i).getName(),i);

   }
   std::cout <<"---------------------------------------------" << std::endl;

}

bool arm_kinematic::init(Eigen::VectorXd q_init, Eigen::VectorXd dotq_init,unsigned int N)
{
    N_Prediciton_ = N;
    setState(q_init,dotq_init);
    robot_state_.head(NrOfDeg_) = q_init;
    robot_state_.tail(NrOfDeg_) = dotq_init ;
    q_horizon_.resize(N_Prediciton_*NrOfDeg_);
    q_horizon_.setZero();

    dotq_horizon_.resize(N_Prediciton_*NrOfDeg_);
    dotq_horizon_.setZero();

    jacobian_horizon_.resize(N_Prediciton_*6,N_Prediciton_*NrOfDeg_);
    jacobian_horizon_.setZero();

    ee_pos_horion_.resize(N_Prediciton_*3);
    ee_pos_horion_.setZero();
    ee_pos_ = getSegmentPosition(7);
    ee_pos_.M.GetRPY(RPY_angle_(0),RPY_angle_(1),RPY_angle_(2));

    for (int i(0); i<N_Prediciton_; i ++)
    {
        q_horizon_.segment(NrOfDeg_*i, NrOfDeg_) = q_init;
        dotq_horizon_.segment(NrOfDeg_*i,NrOfDeg_) = dotq_init;
        ee_pos_horion_.segment(3*i,3) << ee_pos_.p.x(),ee_pos_.p.y(),ee_pos_.p.z();
    }

//    printRobotInformation();
    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << BOLD(FRED("Robot is initalized: "))  <<'\n'
              << FYEL("Joint position: \n ") << robot_state_.head(NrOfDeg_).transpose().format(CleanFmt) << '\n'
              << FYEL("Joint velocity: \n ") << robot_state_.tail(NrOfDeg_).transpose().format(CleanFmt) << '\n'
              << FYEL("end-effector cartesien position: ")<< '\n'
              << FYEL("x : ") << ee_pos_.p.x() << '\n'
              << FYEL("y : ") << ee_pos_.p.y() << '\n'
              << FYEL("z : ") << ee_pos_.p.z() << std::endl;
    std::cout <<"---------------------------------------------" << std::endl;
    return true;
}



void arm_kinematic::setJointPositions(const Eigen::VectorXd& q)
{
     for(unsigned int i=0; i<NrOfDeg_ ; i++){
           q_.data(i)= q(i);
       }
}

void arm_kinematic::setJointVelocities(const Eigen::VectorXd& qd)
{
    for(unsigned int i=0; i<NrOfDeg_ ; i++){
         dotq_.data(i) = qd(i);
     }
}

void arm_kinematic::setState(const Eigen::VectorXd& q,const Eigen::VectorXd& dq)
{
    setJointPositions(q);
    setJointVelocities(dq);
    robot_state_.head(NrOfDeg_) = q;
    robot_state_.tail(NrOfDeg_) = dq;
    ee_pos_ = getSegmentPosition(7) ;
}

void arm_kinematic::computeJacobianHorz(const Eigen::VectorXd &q_horizon)
{
    for (size_t i(0); i<N_Prediciton_; i++)
    {
      KDL::JntArray q_horz_temp;
      KDL::Jacobian jac_horz_temp;

      q_horz_temp.resize(NrOfDeg_);
      jac_horz_temp.resize(NrOfDeg_);
      q_horz_temp.data = q_horizon.segment(NrOfDeg_*i,NrOfDeg_);

      computeJacobian(q_horz_temp,jac_horz_temp);
      jacobian_horizon_.block(6*i,NrOfDeg_*i,6,NrOfDeg_) = jac_horz_temp.data;
    }
}


void arm_kinematic::computeCartPosHorz(const Eigen::VectorXd &q_horizon)
{
    for (int i(0); i<N_Prediciton_; i++)
    {
      KDL::JntArray q_horz_temp;
      KDL::Frame frame_horz_temp;
      q_horz_temp.resize(NrOfDeg_);
      q_horz_temp.data = q_horizon.segment(NrOfDeg_*i,NrOfDeg_);
      fksolver_->JntToCart(q_horz_temp, frame_horz_temp,-1);
      ee_pos_horion_.segment(3*i,3) << frame_horz_temp.p(0),frame_horz_temp.p(1),frame_horz_temp.p(2);
    }

}

void arm_kinematic::initializeSubscriber(){
    ROS_INFO("Initialize Subscribers to robot joints states");
    subscriber_ = nh_.subscribe("/panda/joint_states",100,&arm_kinematic::subscriberCallback,this);
 }


void arm_kinematic::subscriberCallback(const sensor_msgs::JointState &jntState){

        robot_state_.head(NrOfDeg_) << jntState.position[0],jntState.position[1],jntState.position[2],jntState.position[3],jntState.position[4],jntState.position[5],jntState.position[6] ;
        robot_state_.tail(NrOfDeg_) <<jntState.velocity[0],jntState.velocity[1],jntState.velocity[2],jntState.velocity[3],jntState.velocity[4],jntState.velocity[5],jntState.velocity[6] ;
        q_.data <<  jntState.position[0],jntState.position[1],jntState.position[2],jntState.position[3],jntState.position[4],jntState.position[5],jntState.position[6] ;
        dotq_.data << jntState.velocity[0],jntState.velocity[1],jntState.velocity[2],jntState.velocity[3],jntState.velocity[4],jntState.velocity[5],jntState.velocity[6] ;
}



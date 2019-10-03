#include "robotmodel/robotmodel.h"



arm_kinematic::arm_kinematic(const std::string& urdf_name, unsigned int NrOfDeg,const std::string& root_name, const std::string& end_name){

   kdl_parser::treeFromFile(urdf_name, arm_tree_);
   arm_tree_.getChain(root_name,end_name,arm_chain_);
   NrOfDeg_ = NrOfDeg;
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
       seg_names_idx_.add(joints_name_[i],i);
   }

   std::cout <<"---------------------------------------------" << std::endl;
   std::cout << BOLD(FRED("Robot's model is successively parsed into KDL ")) << std::endl;
   std::cout << FYEL("load joint : \n") <<  std::endl;
   for (int i(0); i < NrOfDeg_; i++){
      std::cout<<FCYN("Joint " ) << i << " name "<< arm_chain_.getSegment(i).getJoint().getName() <<std::endl;
   }
   std::cout <<"\n" << std::endl;
   std::cout <<FYEL("load link : \n") << std::endl;

   for (int i(0); i <arm_chain_.getNrOfSegments();i++)
   {
       std::cout<<FCYN("Link " ) << i << " name "<< arm_chain_.getSegment(i).getName() <<std::endl;
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

    ee_pos_horion_.resize(N_Prediciton_*6);
    ee_pos_horion_.setZero();
    ee_pos_ = getSegmentPosition(5);
    ee_pos_.M.GetRPY(RPY_angle_(0),RPY_angle_(1),RPY_angle_(2));

    for (int i(0); i<N_Prediciton_; i ++)
    {
        q_horizon_.segment(NrOfDeg_*i, NrOfDeg_) = q_init;
        dotq_horizon_.segment(NrOfDeg_*i,NrOfDeg_) = dotq_init;
        ee_pos_horion_.segment(6*i,6) << ee_pos_.p.x(),ee_pos_.p.y(),ee_pos_.p.z(),RPY_angle_(0),RPY_angle_(1),RPY_angle_(2);
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

void arm_kinematic::setState(const Eigen::VectorXd& q,const Eigen::VectorXd& qd)
{
    setJointPositions(q);
    setJointVelocities(qd);
    robot_state_.head(NrOfDeg_) = q_.data;
    robot_state_.tail(NrOfDeg_) = dotq_.data;
    ee_pos_ = getSegmentPosition(5) ;
}

void arm_kinematic::computeJacobianHorz(Eigen::VectorXd q_horizon)
{
    for (size_t i(0); i<N_Prediciton_; i++)
    {
      KDL::JntArray q_horz_temp;
      KDL::Jacobian jac_horz_temp;

      q_horz_temp.resize(NrOfDeg_);
      q_horz_temp.data.setZero();
      jac_horz_temp.resize(NrOfDeg_);
      jac_horz_temp.data.setZero();
      q_horz_temp.data = q_horizon.segment(NrOfDeg_*i,NrOfDeg_);

      computeJacobian(q_horz_temp,jac_horz_temp);
      jacobian_horizon_.block(6*i,NrOfDeg_*i,6,NrOfDeg_) = jac_horz_temp.data;
    }
}


void arm_kinematic::computeCartPosHorz(Eigen::VectorXd q_horizon)
{
    for (int i(0); i<N_Prediciton_; i++)
    {
      KDL::JntArray q_horz_temp;
      KDL::Frame frame_horz_temp;
      Eigen::MatrixXd mat_temp;
      q_horz_temp.resize(NrOfDeg_);
      q_horz_temp.data.setZero();
      q_horz_temp.data = q_horizon.segment(NrOfDeg_*i,NrOfDeg_);
      KDL::ChainFkSolverPos_recursive fksolv(arm_chain_);
      fksolv.JntToCart(q_horz_temp, frame_horz_temp, 5+1);
      Eigen::Vector3d angle;
      frame_horz_temp.M.GetRPY(angle(0),angle(1),angle(2));
      ee_pos_horion_.segment(6*i,6) << frame_horz_temp.p(0),frame_horz_temp.p(1),frame_horz_temp.p(2),angle(0),angle(1),angle(2);
    }

}

#ifndef ROSWRAPROBOTMODEL_H
#define ROSWRAPROBOTMODEL_H


#pragma once
#include "iostream"
#include "kdl/chain.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainjnttojacdotsolver.hpp"
#include <boost/timer.hpp>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"
#include <boost/scoped_ptr.hpp>
#include <urdf/model.h>
#include "RosWrapcommun/color.h"
#include "RosWrapcommun/RosWrapCommunDefinition.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <trac_ik/trac_ik.hpp>
#include "RosWrapchainjnttojacdotsolver.h"
#include <visualization_msgs/MarkerArray.h>
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"

class SegmentIndice
{
    /* This class is used to bind segment names to the index in the chain */
    public:
    const int operator()(const std::string& segment_name)
    {
        return this->operator[](segment_name);
    }
    public:
    const int operator[](const std::string& segment_name)
    {
        if(seg_idx_names.empty())
        {
            std::cerr << "Segment idx is empty ! "<< std::endl;
            return -1;
        }
        if(seg_idx_names.count(segment_name) > 0)
        {
            return seg_idx_names[segment_name];
        }
        else
        {
            std::ostringstream ss;
            ss << "Segment name ["<<segment_name<<"] does not exists";
            throw std::runtime_error(ss.str());
        }
    }
    public: void add(const std::string& seg_name,int i)
    {
        seg_idx_names[seg_name] = i;
    }
    protected : std::map<std::string,int> seg_idx_names;
};

/**
    This class is used to compute forward and inverse kinematic with KDL

  */
class  arm_kinematic
{

public:
    /**
      Constructor with ros node
    */
    arm_kinematic(ros::NodeHandle* nodehandle, const std::string& urdf_name,  unsigned int NrOfDeg, const std::string& root_name, const std::string& end_name);

    /**
     Constructor creates robot's kinematic chain
     * @brief arm_kinematic
     * @param urdf_name
     * @param NrOfDeg
     * @param root_name
     * @param end_name
     */

    arm_kinematic(const std::string& urdf_name,  unsigned int NrOfDeg, const std::string& root_name, const std::string& end_name);

    /**
     Initialize robot's state such as joint velocity and position
     * @brief init
     * @param q_init
     * @param dotq_init
     * @param N
     * @return
     */
    bool init(Eigen::VectorXd q_init, Eigen::VectorXd dotq_init,unsigned int N);

//    bool testTracIk(KDL::JntArray q, KDL::Frame desired_end_effector_pose,KDL::JntArray& return_joints){
//        return ik_solver->CartToJnt(q,desired_end_effector_pose,return_joints) >= 0;
//    }

//    KDL::Frame testforward(KDL::JntArray q) {
//        KDL::Frame seg_pos;
//        fksolver_->JntToCart(q, seg_pos);
//        return seg_pos;
//    }
    /**
    * @brief Gives the kdl segment corresponding to the index
    * @param[in] segment Index number of the segment
    * @param[out] kdl_segment The corresponding kdl segment
    */
    const KDL::Segment & getSegment(unsigned int segment)
    {
        return arm_chain_.getSegment(segment);
    }

    /**
    * @brief Returns the segment index corresponding to its name
    * @param[in] name The segment's name
    * @return The segment's index
    */
    unsigned int getSegmentIndex(const std::string& name)
    {
        return seg_names_idx_[name];
    }
    /**
      * @brief Gives the kdl frame position corresponding to the index
      * @param[in] segment Index number of the segment
      * @param[out] kdl_frame The corresponding positon of the segment
      */
    KDL::Frame getSegmentPosition(unsigned int segment) {
        KDL::Frame seg_pos;
        fksolver_->JntToCart(q_, seg_pos, segment);
        return seg_pos;
    }
    /**
     * @brief Gives the kdl frame position corresponding to its name
     * @param[in] segment_name The segment's name
     * @param[out] kdl_frame The corresponding positon of the segment
     */

    KDL::Frame  getSegmentPosition(const std::string& segment_name)
    {
        return getSegmentPosition(getSegmentIndex(segment_name));
    }

    /**
     * @brief getqEnlarged
     * @return  q predicted
     */
    Eigen::VectorXd getqEnlarged()
    {
        return q_horizon_;
    }

    /**
     * @brief getdqEnlarged
     * @return dq predicted
     */
    Eigen::VectorXd getdqEnlarged()
    {
        return dotq_horizon_;
    }

    Eigen::VectorXd getqqdEnlarged(){
        return qqd_horizon_;
    }
    /**
     * @brief getJacobianHorz
     * @return jacobian matrix predicted
     */
    Eigen::MatrixXd getJacobianHorz()
    {
        return jacobian_horizon_;
    }

    Eigen::MatrixXd getJacobianDotHorz(){
        return jacobian_dot_horizon_;
    }
    /**
     * @brief get robot state, current joint position and joint velocity
     * @return
     */
    Eigen::VectorXd getRobotState()
    {
        return robot_state_;
    }

    /**
     * @brief getCartPosHorz
     * @return  end-effector predicted position
     */
    Eigen::VectorXd getCartPosHorz()
    {
        return ee_pos_horion_;
    }

    /**
     * @brief getJointPos
     * @return q_: joint position
     */
    KDL::JntArray getJointPos()
    {
        return q_;
    }

    /**
     * @brief getJointVel
     * @return dq_ : joint velocity
     */
    KDL::JntArray getJointVel()
    {
        return dotq_;
    }

    /**
     * @brief getChain
     * @return KDL robot arm chain
     */
    KDL::Chain getChain()
    {
        return arm_chain_ ;
    }

    /**
     * @brief getNrOfJoints
     * @return total number of joint
     */
    unsigned int getNrOfJoints()
    {
       return arm_tree_.getNrOfJoints();
    }

    /**
     * @brief getRootName
     * @return base name
     */
    std::string getRootName()
    {
        return arm_tree_.getSegments().begin()->second.segment.getName();
    }

    /**
     * @brief getNrOfSegment
     * @return the total number of robot
     */
    unsigned int getNrOfSegment()
    {
        return arm_tree_.getNrOfSegments();
    }

    /**
     * @brief printRobotInformation
     *  Joint name and link name
     */
    void printRobotInformation()
    {
        for (int i(0); i < NrOfDeg_; i++){
           std::cout<<"Joint "<< i<< " name "<< arm_chain_.getSegment(i).getJoint().getName() <<std::endl;
        }

        for (int i(0); i <arm_chain_.getNrOfSegments();i++)
        {  std::cout<<"Link "<< i << " name "<< arm_chain_.getSegment(i).getName() <<std::endl;
        }
    }

    /**
     * @brief setJointPositions
     * @param q
     */
    void setJointPositions(const Eigen::VectorXd& q);
    /**
    * @brief Sets the joint velocity of the model
    * @param[in] qd_des the joint velocity in rad.
    */
    void setJointVelocities(const Eigen::VectorXd& qd);

    /**
     * @brief setState
     * @param q
     * @param qd
     */
    void setState(const Eigen::VectorXd& q,const Eigen::VectorXd& qd);

    /**
     * @brief setJointHorizon
     * @param qHorizon
     */
    void setJointHorizon(const Eigen::VectorXd &qHorizon){
        q_horizon_ = qHorizon;

    }


    void setJointVelHorizon(const Eigen::VectorXd &dqHorizon){
        dotq_horizon_ = dqHorizon ;
    }

    void setqqdhorizon(const Eigen::VectorXd &qHorizon,const Eigen::VectorXd &dqHorizon){
        for (int k(0) ; k < N_Prediction_ ; k++){
            qqd_horizon_.segment(2*NrOfDeg_*k, NrOfDeg_) = qHorizon.segment(NrOfDeg_*k,NrOfDeg_);
            qqd_horizon_.segment(2*NrOfDeg_*k+NrOfDeg_,NrOfDeg_) = dqHorizon.segment(NrOfDeg_*k,NrOfDeg_);
        }
    }

    void setMPCState(const Eigen::MatrixXd & pandaPx,const Eigen::MatrixXd & pandaPu,
                     const Eigen::MatrixXd & pandaPxdq,const Eigen::MatrixXd & pandaPudq);
    /**
     * @brief compute inverse kinematic
     * @param desire_frame
     * @param q_sol: desired joint position
     */
    void computeJntFromCart(const KDL::Frame& desire_frame,KDL::JntArray& q_sol)
    {
        int i ;
        i = iksolver_->CartToJnt(q_,desire_frame,q_sol);
    }

    /**
     * @brief computeJacobian
     */
    void computeJacobian()
    {
//        chainjacsolver_->JntToJac(q_, jacobian_,7);
        chainjacsolver_->JntToJac(q_, jacobian_,8);
    }

    /**
      * @brief compute jacobian dot
      */

    void computeJacobianDot(){
//        chainjacdotsolver_ ->JntToJacDot(qqd_,jacobianDot_,7);
        chainjacdotsolver_ ->JntToJacDot(qqd_,jacobianDot_,8);
    }
    /**
     * @brief getJacobian
     * @return jacobian
     */
    KDL::Jacobian getJacobian()
    {
        return jacobian_;
    }

    KDL::Jacobian getJacobianDot(){
        return jacobianDot_;
    }
    /**
     * @brief computeJacobian
     * @param q_in
     * @param jacobian_out
     */
    void computeJacobian(const KDL::JntArray& q_in, KDL::Jacobian& jacobian_out){
//        chainjacsolver_->JntToJac(q_in,jacobian_out,7);
        chainjacsolver_->JntToJac(q_in,jacobian_out,8);

    }


    void computeJacobianDot(const KDL::JntArrayVel & qqd_in, KDL::Jacobian& jacobianDot_out ){

//        chainjacdotsolver_ -> JntToJacDot(qqd_in, jacobianDot_out,7);
        chainjacdotsolver_ -> JntToJacDot(qqd_in, jacobianDot_out,8);

    }

    /**
     * @brief ComputeJacobian matrix during predicted horizon
     * @param q_horizon
     */
    void computeJacobianHorz(const Eigen::VectorXd &q_horizon);

    void computeJacobianDotHorz(const Eigen::VectorXd &qqd_horizon);
    /**
     * @brief Compute end-effector' position during predicted horizon
     * @param q_horizon
     */
    void computeCartPosHorz();

    /**
     * @brief Compute joint position horizon ;
     * @param optimal_control
     */

    void computeJntPosHorz(const Eigen::VectorXd &optimal_control){
        q_horizon_ = pandaPx_*robot_state_ + pandaPu_ * optimal_control;
    }


    /**
     * @brief Initalize ros subscriber node to update robot's joint position and joint vel
     */
    void initializeSubscriber();

    /**
     * @brief subscriberCallback
     * @param Ros message joint state
     */
    void subscriberCallback(const sensor_msgs::JointState& jntState);

    /**
     * @brief Node publish robot's vertices
     * 
     */

    void verticesPublisher();

protected:

    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    rviz_visual_tools::RvizVisualToolsPtr robotVerticesPub_ ;   
    /**
     * @brief NrOfDeg_ : dof of robot
     * @brief N_prediction : Prediction step
     */
    unsigned int NrOfDeg_, N_Prediction_ ;

    /**
     * @brief The chain structure of the robot
     */
    KDL::Tree arm_tree_;
    KDL::Chain arm_chain_;

    /**
    * @brief The joints limits
    */
    Eigen::VectorXd joint_lower_limit_;
    Eigen::VectorXd joint_upper_limit_;

    /**

     * @brief the current joint positon and joint velocity, Jacobian etc ...
     * @brief robot_state = (q, dq)
     */

    KDL::JntArray q_, dotq_;
    Eigen::VectorXd robot_state_;
    KDL::Jacobian jacobian_, jacobianDot_;
    Eigen::Vector3d RPY_angle_ ;
    /**
     * @brief Enlarge data to formulate MPC problem (q, dq, ee, Jacobian)
     */
    Eigen::VectorXd q_horizon_, dotq_horizon_,qqd_horizon_, ee_pos_horion_, foream_pos_horizon_;
    Eigen::MatrixXd jacobian_horizon_;
    Eigen::MatrixXd jacobian_dot_horizon_;
    /**
     * @brief joints_name_
     */
    std::vector<std::string> joints_name_;

    Eigen::Matrix<double,6,1> L;

    /**
     * @brief the forward kinematic solver for position
     *        and inverse kinematic
     */
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> iksolver_;
    boost::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
    /**
    * @brief The jacobian solver.
    */
    boost::scoped_ptr<KDL::ChainJntToJacSolver> chainjacsolver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> chainjacdotsolver_;    /**
       * @brief The gravity induced joint torque
       * for the current joint position.
       */
    KDL::JntArray gravityTorque_;
    KDL::Jacobian seg_jacobian_,seg_jacobian_dot_,tmp_jac_;
    KDL::JntArray tmp_array_pos;
    KDL::JntArrayVel qqd_;
    KDL::Twist jdot_qdot_,seg_jdot_qdot_;
    KDL::Frame ee_pos_,seg_pos_;
    KDL::Twist ee_vel_,seg_vel_;
    KDL::FrameVel frame_vel_;
    KDL::JntArray ext_wrench_torque_,ext_add_torque_,ext_torque_all_;

    /**
    * @brief the rosparam argument names
    */
    std::string robot_description_ros_name;
    std::string root_link_ros_name;
    std::string tip_link_ros_name;
    SegmentIndice seg_names_idx_;
   
    /**
     * @brief MPC state 
     * 
     */
    
    Eigen::MatrixXd pandaPx_, pandaPu_, pandaPxdq_, pandaPudq_;


};

#endif

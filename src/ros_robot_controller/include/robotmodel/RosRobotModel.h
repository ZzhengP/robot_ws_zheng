#pragma once
#include "iostream"
#include "kdl/chain.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include <boost/timer.hpp>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"
#include <boost/scoped_ptr.hpp>
#include <urdf/model.h>
#include "commun/color.h"
#include "commun/RosCommunDefinition.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
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


class  arm_kinematic
{
public:
    arm_kinematic(ros::NodeHandle* nodehandle, const std::string& urdf_name,  unsigned int NrOfDeg, const std::string& root_name, const std::string& end_name);

    arm_kinematic(const std::string& urdf_name,  unsigned int NrOfDeg, const std::string& root_name, const std::string& end_name);

    /**
     * @brief Initialize all robot state ( Matrix or vector)
     * @return
     */
    bool init(Eigen::VectorXd q_init, Eigen::VectorXd dotq_init,unsigned int N);


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
        fksolver_->JntToCart(q_, seg_pos_, segment);
        return seg_pos_;
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

    KDL::JntArray getJointsPosition(){
        return q_;
    }

    KDL::JntArray getJointsVelocity()
    {
        return dotq_;
    }

    Eigen::VectorXd getqEnlarged()
    {
        return q_horizon_;
    }

    Eigen::VectorXd getdqEnlarged()
    {
        return dotq_horizon_;
    }

    Eigen::MatrixXd getJacobianHorz()
    {
        return jacobian_horizon_;
    }
    /**
     * @brief get robot state, current joint position and joint velocity
     * @return
     */
    Eigen::VectorXd getRobotState()
    {
        return robot_state_;
    }

    Eigen::VectorXd getCartPosHorz()
    {
        return ee_pos_horion_;
    }

    void printRobotInformation()
    {
        for (int i(0); i < NrOfDeg_; i++){
           std::cout<<"Joint "<< i<< " name "<< arm_chain_.getSegment(i).getJoint().getName() <<std::endl;
        }

        for (int i(0); i <arm_chain_.getNrOfSegments();i++)
        {  std::cout<<"Link "<< i << " name "<< arm_chain_.getSegment(i).getName() <<std::endl;
        }
    }

    void setJointPositions(const Eigen::VectorXd& q);
    /**
    * @brief Sets the joint velocity of the model
    * @param[in] qd_des the joint velocity in rad.
    */
    void setJointVelocities(const Eigen::VectorXd& qd);

    void setState(const Eigen::VectorXd& q,const Eigen::VectorXd& qd);


    std::vector<double> getJointLowerLimits();
    std::vector<double> getJointUpperLimits();


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
        chainjacsolver_->JntToJac(q_, jacobian_);
    }

    KDL::Jacobian getJacobian()
    {
        return jacobian_;
    }

    /**
     * @brief computeJacobian for MPC use
     * @param q_in
     * @param jacobian_out
     */
    void computeJacobian(const KDL::JntArray& q_in, KDL::Jacobian& jacobian_out){
        chainjacsolver_->JntToJac(q_in,jacobian_out);
    }

    void computeqEnlarged(Eigen::VectorXd optimal_Solution, Eigen::MatrixXd Px, Eigen::MatrixXd Pu)
    {
        q_horizon_ = Px*robot_state_ + Pu*optimal_Solution;
    }

    void KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix)
    {
        matrix.resize(jacobian.rows(),jacobian.columns());
        for (size_t i=0; i<jacobian.rows();i++)
            for (size_t j=0; j<jacobian.columns(); j++)
                matrix(i,j) = jacobian(i,j);
    }

    void computeJacobianHorz(Eigen::VectorXd q_horizon);

    void computeCartPosHorz(Eigen::VectorXd q_horizon);

    KDL::Chain getChain()
    {
        return arm_chain_ ;
    }

    unsigned int getNrOfJoints()
    {
       return arm_tree_.getNrOfJoints();
    }

    std::string getRootName()
    {
        return arm_tree_.getSegments().begin()->second.segment.getName();
    }

    unsigned int getNrOfSegment()
    {
        return arm_tree_.getNrOfSegments();
    }

    void initializeSubscriber();

    void subscriberCallback(const sensor_msgs::JointState& jntState);

    void initializePublisher();
protected:

    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_1_, publisher_2_, publisher_3_, publisher_4_, publisher_5_, publisher_6_;
    unsigned int NrOfDeg_, N_Prediciton_ ;
    /**
     * @brief The chain structure of the robot
     */
    KDL::Tree arm_tree_;
    KDL::Chain arm_chain_;

    /**
    * @brief The joints limits
    */
    std::vector<double> joints_lower_limit_;
    Eigen::VectorXd joint_lower_limit_;
    std::vector<double> joints_upper_limit_;
    Eigen::VectorXd joint_upper_limit_;

    /**
     * @brief the current joint positon and joint velocity, Jacobian etc ...
     */

    KDL::JntArray q_, dotq_;
    Eigen::VectorXd robot_state_;
    KDL::Jacobian jacobian_;
    Eigen::Vector3d RPY_angle_;
    /**
     * @brief Enlarge data to formulate MPC problem
     */
    Eigen::VectorXd q_horizon_, dotq_horizon_, ee_pos_horion_;
    Eigen::MatrixXd jacobian_horizon_;
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

    /**
    * @brief The jacobian solver.
    */
    boost::scoped_ptr<KDL::ChainJntToJacSolver> chainjacsolver_;

    /**
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

};

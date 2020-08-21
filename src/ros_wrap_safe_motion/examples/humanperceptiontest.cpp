#include "RosWrapperception/RosWrapHumanOptimalTorque.h"
#include "RosWrapperception/RosWrapHumanMotion.h"
#include "memory"

int main(){

    // std::shared_ptr<HumanTorque> human_torque = std::make_shared<HumanTorque>(0.1);
    // Eigen::MatrixXd  J;
    // J.resize(4,4);
    // J.setZero();
    // Eigen::VectorXd X_prec, ddX_prec;
    // X_prec.resize(4);
    // X_prec.setZero();

    // human_torque -> compute_hand_jerk();
    // human_torque -> compute_joint_jerk();
    // human_torque -> compute_joint_acceleration();
    // human_torque -> compute_torque_change();
    // human_torque -> compute_minimal_torque();
    // human_torque -> compute_geodesis();
    // human_torque -> compute_energy();

    // human_torque -> update_objective();
    // human_torque -> compute_total_objective();
    // human_torque -> solve();



    // ------------------------  Test of human kinematic model ---------------------
    const int ndof = 4;
    // const double arm_len = 0.271;
    const double arm_len = 1;
    // const double forearm_len = 0.283;
    const double forearm_len = 1;
    // const double hand_len = 0.08;
    const double hand_len = 0.0;
    // const double shoulder_len = 0.15;
    const double shoulder_len = 0.;
    const double pi = 3.1415927;
    std::map<std::string, Eigen::Matrix4d>  M_joint;
    std::map<std::string, Eigen::Matrix4d>  M_CoM;



    M_joint["M_shoulder1"] << 0,0,1,0,
                              0,1,0,-shoulder_len,
                              -1,0,0,0,
                              0,0,0,1;

    M_joint["M_shoulder2"] << 1,0,0,0,
                              0,0,1,-shoulder_len,
                              0,-1,0,0,
                              0,0,0,1;

    M_joint["M_shoulder3"] << 1,0,0,0,
                              0,1,0,-shoulder_len,
                              0,0,1,0,
                              0,0,0,1;

    M_joint["M_forearm"] << 1,0,0,0,
                            0,cos(pi/2),-sin(pi/2),-shoulder_len,
                            0,-sin(pi/2),cos(pi/2),-arm_len,
                            0,0,0,1;

    M_joint["M_hand"] <<   1,0,0,0,
                           0,1,0,-shoulder_len,
                           0,0,1,-arm_len - forearm_len,
                           0,0,0,1;             

    M_CoM["M_C1"] << 1,0,0,0,
                     0,1,0,0,
                     0,0,1,-arm_len/2,
                     0,0,0,1;

    M_CoM["M_C2"] << 1,0,0,0,
                     0,1,0,-shoulder_len,
                     0,0,1,-arm_len - forearm_len/2,
                     0,0,0,1;                 
        

    Eigen::MatrixXd S_list; 
    S_list.resize(6,ndof);
    S_list.block(0,0,6,1) << -1, 0, 0, 0, 0, 0 ;
    S_list.block(0,1,6,1) << 0, -1, 0, 0, 0, 0 ;
    S_list.block(0,2,6,1) << 0, 0, 1, 0, 0, 0 ;
    S_list.block(0,3,6,1) << 0, -1, 0 , -arm_len, 0, 0 ;


    std::shared_ptr<HumanMotion> human_motion = std::make_shared<HumanMotion>(ndof, M_joint, M_CoM, S_list);

    Eigen::Vector4d theta_list( 0,0,0,0);
    Eigen::Matrix4d current_hand_frame, desired_hand_frame;
    Eigen::Matrix4d current_elbow_frame;
    current_hand_frame = human_motion -> FKinSpace(M_joint["M_hand"], S_list, theta_list);
    std::vector<Eigen::MatrixXd> Rp;
    std::cout <<" current hand frame \n" << current_hand_frame << std::endl; 
    Rp = human_motion->TransToRp(current_hand_frame);
    Eigen::VectorXd desired_theta ;
    desired_theta.resize(4);
    desired_theta.setZero();
    desired_hand_frame.setZero();
    desired_hand_frame(3,3) = 1;
    desired_hand_frame.block(0,0,3,3) << 0,0,-1,
                                         0,1,0,
                                         1,0,0;

    desired_hand_frame.block(0,3,3,1) << 1,0,-1;
    bool is_IK_solved;
    // is_IK_solved = human_motion->IKNumericalSolver(S_list,M_joint["M_hand"],Rp[1],desired_theta);
    is_IK_solved = human_motion -> IKinSpace(S_list,M_joint["M_hand"],desired_hand_frame,desired_theta,0.001,0.001);
    std::cout <<" desired theta :\n " << desired_theta << std::endl;
    std::cout <<" desired hand frame \n" << human_motion -> FKinSpace(M_joint["M_hand"], S_list, desired_theta) << std::endl; 

    return 0;
}
#include "RosWrapperception/RosWrapHumanOptimalTorque.h"
#include "RosWrapperception/RosWrapHumanMotion.h"
#include "memory"

int main(){

    std::shared_ptr<HumanTorque> human_torque = std::make_shared<HumanTorque>(0.1);
    Eigen::MatrixXd  J;
    J.resize(4,4);
    J.setZero();
    Eigen::VectorXd X_prec, ddX_prec;
    X_prec.resize(4);
    X_prec.setZero();

    // human_torque -> compute_hand_jerk();
    // human_torque -> compute_joint_jerk();
    // human_torque -> compute_joint_acceleration();
    // human_torque -> compute_torque_change();
    // human_torque -> compute_minimal_torque();
    // human_torque -> compute_geodesis();
    // human_torque -> compute_energy();

    human_torque -> update_objective();
    human_torque -> compute_total_objective();
    human_torque -> solve();



    // ------------------------  Test of human kinematic model ---------------------
    const int ndof = 4;
    const double arm_len = 0.271;
    const double forearm_len = 0.283;
    const double hand_len = 0.08;
    const double pi = 3.1415927;
    std::map<std::string, Eigen::Matrix4d>  M_joint;
    std::map<std::string, Eigen::Matrix4d>  M_CoM;

    M_joint["M_arm"] << 1,0,0,0,
                        0,1,0,0,
                        0,0,1,0,
                        0,0,0,1;

    M_joint["M_forearm"] << 1,0,0,0,
                            0,1,0,0,
                            0,0,1,-arm_len,
                            0,0,0,1;

    M_joint["M_hand"] <<   1,0,0,0,
                           0,1,0,0,
                           0,0,1,-arm_len - forearm_len,
                           0,0,0,1;             

    M_CoM["M_C1"] << 1,0,0,0,
                     0,1,0,0,
                     0,0,1,-arm_len/2,
                     0,0,0,1;

    M_CoM["M_C2"] << 1,0,0,0,
                     0,1,0,0,
                     0,0,1,-arm_len - forearm_len/2,
                     0,0,0,1;                 
        

    Eigen::MatrixXd S_list; 
    S_list.resize(6,ndof);
    S_list.block(0,0,6,1) << -1, 0, 0, 0, 0, 0 ;
    S_list.block(0,1,6,1) << 0, -1, 0, 0, 0, 0 ;
    S_list.block(0,2,6,1) << 0, 0, 1, 0, 0, 0 ;
    S_list.block(0,3,6,1) << 0, -1, 0 , -arm_len, 0, 0 ;


    std::shared_ptr<HumanMotion> human_motion = std::make_shared<HumanMotion>(ndof, M_joint, M_CoM, S_list);

    Eigen::Vector4d theta_list(pi/2,0,0,0);
    Eigen::Matrix4d current_frame;
    
    current_frame = human_motion -> FKinSpace(M_joint["M_hand"], S_list, theta_list);

    std::cout <<" current hand frame  :\n " << current_frame << std::endl;

    return 0;
}
#include "humanmodel/humanmodel.h"
#include "iostream"
#include <boost/shared_ptr.hpp>
#include "memory.h"
#include "ros/ros.h"
#include <fstream>

int main(int argc, char **argv){

    double L1 = 1. , L2 = 1., L_hand = 10, pi = 3.1415; 
    int ndof = 2, N = 5;
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

    S_list.resize(6,ndof) ;
    S_list.block(0,0,6,1) << 0, 0, 1, 0, 0, 0;
    S_list.block(0,1,6,1) << 0, 0, 1, 0, -1, 0;

    double V_max = 1;


    std::shared_ptr<humanModel> human_model  = std::make_shared<humanModel>(ndof,M_list);

    Eigen::Vector2d dq, q, ddq ;
    dq << 1,1;
    q << 0, pi/2;
    ddq.setZero();
    Eigen::VectorXd ee_vel;
    ee_vel.resize(6);

    Eigen::MatrixXd J_tool_space ;
    J_tool_space.resize(6,ndof);
    J_tool_space = human_model -> JacobianSpace(S_list,q);
    

    
    std::ofstream myfile;
    myfile.open ("/home/zheng/Bureau/hand_position.txt");


    Eigen::VectorXd ee_twist_space; 
    ee_twist_space.resize(6);
    ee_twist_space = J_tool_space*dq;

    Eigen::Matrix4d ee_trans_base;
    ee_trans_base = human_model -> FKinSpace(M_hand, S_list, q);


    ee_vel = human_model -> computeVelocity(ee_twist_space,ee_trans_base);

    std::vector<Eigen::VectorXd> hand_prediction_data;

	hand_prediction_data.resize(N-1);

    hand_prediction_data = human_model -> computeOccupancy(q,dq,ddq,N,V_max);
    
    for (int i(0); i < N-1; i++){
        myfile << hand_prediction_data[i].transpose() << '\n';   
        std::cout <<" hand occupation space : \n" << hand_prediction_data[i]<< '\n';
    }

    myfile.close();
    return 0;
}
#include "RosWrapperception/RosWrapHumanOptimalTorque.h"
#include "RosWrapperception/RosWrapHumanMotionMR.h"
#include "memory"
#include <Eigen/Dense>
#include "RosWrapperception/eigenmultivariatenormal.h"
#include "RosWrapperception/RosWrapHumanOccupancy.h"


int main(){

    const double pi = 3.1415926535897932384626433832795029;
    const int ndof = 4; 
    const double lu = 0.5, lf = 0.5, lh = 0.1, m1 = 5.0, m2 = 5.0;
    Eigen::MatrixXd M_home(4,4); 
    M_home << 0, 0, 1, 0,
              1, 0, 0, lu+lf+lh,
              0, 1, 0, 0,
              0, 0, 0, 1;

    std::vector<Eigen::MatrixXd> Mlist(ndof+1), Glist(ndof);
    Mlist[0].resize(4,4), Mlist[1].resize(4,4), Mlist[2].resize(4,4), Mlist[3].resize(4,4), Mlist[4].resize(4,4);
    Mlist[0]<< 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
    Mlist[1]<< 0, 1, 0, 0,
               0, 0, 1, 0,
               1, 0, 0, 0,
               0, 0, 0, 1;
    Mlist[2]<< 0, 1, 0, 0,
               0, 0, 1, 0,
               1, 0, 0, lu/2,
               0, 0, 0, 1;
    Mlist[3]<< 1, 0, 0, lu/2 + lf/2,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
    Mlist[4]<< 1, 0, 0, lf/2 + lh,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
    Glist[0].resize(6,6), Glist[1].resize(6,6), Glist[2].resize(6,6), Glist[3].resize(6,6);
    Glist[0].setZero(), Glist[1].setZero();

    Glist[2].setIdentity(), Glist[2].block(3,3,3,3) = Eigen::MatrixXd::Identity(3,3)*m1;
    Glist[3].setIdentity(), Glist[3].block(3,3,3,3) = Eigen::MatrixXd::Identity(3,3)*m2;


    Eigen::MatrixXd Slist(6,ndof);
    Slist.col(0) << 0, 0, 1, 0, 0, 0;
    Slist.col(1) << 0, 1, 0, 0, 0, 0;
    Slist.col(2) << 1, 0, 0, 0, 0, 0;
    Slist.col(3) << 1, 0, 0, 0, 0, -lu;

    Eigen::VectorXd q(ndof), dq(ndof), ddq(ndof);

    q << 0, 0, 0, 0;
    dq << 0, 0, 0, 0;
    ddq << 0, 0, 0, 0;
    Eigen::MatrixXd Blist(6,ndof);  
    Blist.col(0) << 0, 1, 0, 0, 0, -lu-lf-lh;
    Blist.col(1) << 1, 0, 0, 0, 0, 0;
    Blist.col(2) << 0, 0, 1, 0, lu+lf+lh, 0;
    Blist.col(3) << 0, 0, 1, 0, lf+lh, 0;


    std::shared_ptr<HumanMotionMR> humanmotion = std::make_shared<HumanMotionMR>(ndof,M_home,Slist,Blist,Mlist,Glist);
    Eigen::MatrixXd Toh, Tho;
    Toh.resize(4,4);
    Tho.resize(4,4);
    Toh = humanmotion -> FKinSpace(M_home,Slist,q);
    Tho = humanmotion ->TransInv(Toh);
  

    Eigen::MatrixXd Js(6,ndof), Jb(6,ndof);
    Js.setZero(), Jb.setZero();
    Js = humanmotion -> JacobianSpace(Slist,q);
    Jb = humanmotion -> JacobianBody(Blist, q);


    Jb.setZero();
    Jb = humanmotion -> JsToJb(Js,Tho);
    Eigen::VectorXd Vb(6), Vs(6);
    Vb = Jb*dq;
    Vs = humanmotion -> TwistBodyToBase(Vb,Toh.block(0,0,3,3));
    std::cout <<" Vb expressed in base : \n "<< Vs << std::endl;



    Eigen::Vector3d mean;
    Eigen::Matrix3d covar = Eigen::Matrix3d::Identity();
    mean << 0,0,0; // Set the mean
    covar(0,0) = 0.2;
    covar(1,1) = 0.2;
    covar(2,2) = 0.2;
  // Create a covariance matrix
  // Much wider than it is tall
  // and rotated clockwise by a bit
    

  // Create a bivariate gaussian distribution of doubles.
  // with our chosen mean and covariance
    Eigen::EigenMultivariateNormal<double,3> normX(mean,covar);

    const double dt = 0.1;
    std::shared_ptr<RosWrapHumanOccupancy> humanoccupancy = std::make_shared<RosWrapHumanOccupancy>(dt,ndof);

    Eigen::VectorXd state(7), vel(3), hand_pos(3);
    hand_pos = Toh.block(0,3,3,1);
    std::cout << "hand position :\n" << hand_pos<< std::endl;
    state <<0, 0, 0, 0, hand_pos(0), hand_pos(1), hand_pos(2);
    vel << 1, 1, 1;
    Eigen::MatrixXd J(6,ndof);
    J = Jb;
    Eigen::MatrixXd J_temp(ndof,ndof), J_pseudo_inv(ndof,6);
    J_temp = J.transpose()*J;
    J_pseudo_inv = J_temp.inverse()*J.transpose();
    state = humanoccupancy->computeDynamic(state,J_pseudo_inv,vel);
    Eigen::MatrixXd vel_sample(10,3);
    vel_sample = normX.samples(10).transpose();
    std::cout <<" vel sample :\n" << vel_sample.block(0,0,1,3) << std::endl;
    for (int i(0); i<10; i++){
        vel(0) = vel_sample(i,0);
        vel(1) = vel_sample(i,1);
        vel(2) = vel_sample(i,2);
        state = humanoccupancy->computeDynamic(state,J_pseudo_inv,vel);
        std::cout <<" state :\n" << state << std::endl;
    }
    
    return 0;
}
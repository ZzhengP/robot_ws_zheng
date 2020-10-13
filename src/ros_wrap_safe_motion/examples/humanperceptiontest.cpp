#include "RosWrapperception/RosWrapHumanOptimalTorque.h"
#include "RosWrapperception/RosWrapHumanMotion.h"
#include "memory"
#include <Eigen/Dense>


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
    const double pi = 3.1415927 ;
    const double lu = 0.5, lf= 0.5, lh = 0.1, lc1 = 0.25, lc2 = 0.3, m1 = 1, m2 = 1;
    Eigen::MatrixXd DHparam;

    DHparam.resize(ndof,4);

    // DHparam << 0, 0, 0, 0,
    //            0, pi/2, 0, pi/2,
    //            0, pi/2, 0, pi/2,
    //            0, pi/2, 0, pi/2 ;  
    DHparam << 0, 0, 0, 0,
               0, -pi/2, 0, -pi/2 + 0 ,
               0, -pi/2, 0, -pi/2 + 0 ,
               lu, 0, 0, 0 + 0  ;  

    std::shared_ptr<HumanMotion> humanmotion = std::make_shared<HumanMotion>(ndof, DHparam,lc1, lc2,lu,lf,lh, m1, m2);    


    Eigen::VectorXd theta, dtheta, ddtheta, torque;
    theta.resize(ndof);
    dtheta.resize(ndof);
    ddtheta.resize(ndof);
    torque.resize(ndof);
    theta << -pi/2, 0, 0, 0;
    dtheta <<  0, 0, 0, 0;
    ddtheta << 0, 0, 0, 0;
    humanmotion->computeForwardKinematic(theta);

    humanmotion->computeTransformationMatrixToBaseList();
    
    // std::cout <<" hand transform \n" << humanmotion->getHandTransformation()<< std::endl;
    std::vector<Eigen::Matrix4d> TransformationMatrixList, TransformationMatrixToBaseList;
    TransformationMatrixList = humanmotion->getTransformationMatrixList();
    TransformationMatrixToBaseList = humanmotion->getTransformationToBaseMatrixList();
    humanmotion->print();      
    humanmotion->computeStackedVelocity(dtheta);
    std::vector<Eigen::Vector3d> StackedLinearVelocity;
    std::vector<Eigen::Vector3d> StackedAngularVelocity;

    StackedAngularVelocity = humanmotion -> getStackedAngularVelocity();
    StackedLinearVelocity = humanmotion -> getStackedLinearVelocity();
    Eigen::Matrix3d Rot05;
    Rot05 = TransformationMatrixToBaseList[ndof].block(0,0,3,3);
    std::cout <<" wrist angular velocity :\n" << StackedAngularVelocity[ndof-1]<< std::endl;
    std::cout <<" wrist linear velocity :\n" << StackedLinearVelocity[ndof-1]<< std::endl;
    Eigen::MatrixXd Jacobian, Jacobian_C1, Jacobian_C2;
    Jacobian.resize(6,ndof);
    Jacobian_C1.resize(6,ndof);
    Jacobian_C2.resize(6,ndof);

    humanmotion -> computeJacobian();
    Jacobian_C1 =  humanmotion -> computeJacobianC1(TransformationMatrixToBaseList);
    Jacobian_C2 = humanmotion ->computeJacobianC2(TransformationMatrixToBaseList);
    humanmotion -> computeJacobianC2();
    Jacobian = humanmotion->getJacobian();
    
    Eigen::VectorXd vel;
    vel.resize(6);
    vel = Jacobian*dtheta;
    std::cout <<" linear velocity in  base frame :\n" << vel.segment(0,3) << std::endl;
    std::cout <<" angular velocity in  base frame :\n" << vel.segment(3,3) << std::endl;
    humanmotion->computeHandVelocity();
    std::cout <<" forwarkd hand linear velocity : \n" << Rot05*humanmotion->getHandLinearVelocity() << std::endl;

    Eigen::Matrix4d InertialMass;
    InertialMass.setZero();
    InertialMass = humanmotion->computeInertialMass(TransformationMatrixToBaseList, Jacobian_C1, Jacobian_C2);

    std::cout <<" inertial Mass :\n " << InertialMass << std::endl;

    torque = humanmotion -> computeForwardDynamic(dtheta, ddtheta);

    std::cout <<" torque :\n " << torque << std::endl;

    return 0;
}


#ifndef RosWrapRobotDynTree_H
#define RosWrapRobotDynTree_H
#pragma once
#pragma once

#include <map>
#include <cstdlib>


// Eigen headers
#include <Eigen/Core>
#include <Eigen/Dense>
// IDynTree headers
#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between
// iDynTree datastructures

#include <iDynTree/Core/EigenHelpers.h>

namespace robot {

struct EigenRobotState{

    void resize(int nrOfInternalDofs)
    {
        world_H_base.setIdentity();
        jointPos.resize(nrOfInternalDofs);
        jointVel.resize(nrOfInternalDofs);
        jointPos.setZero();
        jointVel.setZero();
        gravity << 0,0, -9.81;
    }

    void setFixedBase(){
        world_H_base.setIdentity();
        baseVel.setZero();
        jointPos.setZero();
        jointVel.setZero();
    }
    Eigen::MatrixXd world_H_base;
    Eigen::VectorXd jointPos;
    Eigen::Matrix<double,6,1> baseVel;
    Eigen::VectorXd jointVel;
    Eigen::Vector3d gravity;
};

/**
 * Struct containing the floating robot state
 * using iDynTree data structures.
 * For the semantics of this structures,
 * see KinDynComputation::setRobotState method.
 */
struct iDynTreeRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
    }

    void setFixedBase()
    {
        gravity(0) = 0;
        gravity(1) = 0;
        gravity(2) = -9.81;
    }

    void fromEigen(EigenRobotState& eigRobotState)
    {
        iDynTree::fromEigen(world_H_base,eigRobotState.world_H_base);
        iDynTree::toEigen(jointPos) = eigRobotState.jointPos;
        iDynTree::fromEigen(baseVel,eigRobotState.baseVel);
        iDynTree::toEigen(jointVel) = eigRobotState.jointVel;
        iDynTree::toEigen(gravity)  = eigRobotState.gravity;
    }

    iDynTree::Transform world_H_base;
    iDynTree::VectorDynSize jointPos;
    iDynTree::Twist         baseVel;
    iDynTree::VectorDynSize jointVel;
    iDynTree::Vector3       gravity;
};

struct EigenRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
        baseAcc.setZero();
        jointAcc.setZero();
    }

    void random()
    {
        baseAcc.setRandom();
        jointAcc.setRandom();
    }

    void setZero()
    {
        baseAcc.setZero();
        jointAcc.setZero();
    }

    Eigen::Matrix<double,6,1> baseAcc;
    Eigen::VectorXd jointAcc;
};

struct iDynTreeRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
        baseAcc.zero();
        jointAcc.zero();
    }

    void zero()
    {
        baseAcc.zero();
        jointAcc.zero();
    }
    iDynTree::Vector6 baseAcc;
    iDynTree::VectorDynSize jointAcc;
};



class RobotDynClass
{
public:

    RobotDynClass(const std::string& modelFile="");

    bool loadModelFromFile(const std::string& modelFile );

    void setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity);
    void setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& global_gravity_vector);
    void setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel);

    void print() const;

    void setBaseFrame(const std::string& fixed_base_or_free_floating_frame);

    const std::string& getBaseFrame() const;

    void setGravity(const Eigen::Vector3d& global_gravity_vector);

    const std::string& getFileURL() const;

    unsigned int getNrOfDegreesOfFreedom() const;

    unsigned int configurationSpaceDimension() const;

    bool frameExists(const std::string& frame_name);

    const std::map<unsigned int, std::pair<double,double> >& getJointPositionLimits();

    Eigen::Matrix< double, 4, 4, Eigen::RowMajor > getRelativeTransform(const std::string& refFrameName, const std::string& frameName);    const Eigen::Matrix<double,6,1> getFrameVel(const std::string& frameName);

    Eigen::Map< const Eigen::Matrix<double,6,1> > getFrameBiasAcc(const std::string& frameName);

    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > getFreeFloatingMassMatrix();

    const Eigen::VectorXd& getJointPos();

    const Eigen::VectorXd& getJointVel();

    const Eigen::VectorXd& generalizedBiasForces();

    Eigen::MatrixXd getJacobian(const std::string &ref_frame, const std::string &frame_name){

        const iDynTree::Model & model = kinDynComp_.model();
        iDynTree::FrameFreeFloatingJacobian idynJacobian(model);

        kinDynComp_.getRelativeJacobian(0,8,idynJacobian);
        Eigen::MatrixXd eigJacobian = iDynTree::toEigen(idynJacobian);
        return eigJacobian;
    }

    const iDynTree::Model& getRobotModel();
    unsigned int getNrOfJoints();
    std::string getJointName(unsigned int idx);
    bool isInitialized() const;
protected:


    iDynTree::KinDynComputations kinDynComp_;

    EigenRobotState eigRobotState_;
    iDynTreeRobotState idynRobotState_;
    bool is_initialized_ = false;
    std::string base_frame_;
    std::string urdf_url_;
    Eigen::Vector3d global_gravity_vector_;
    std::map<unsigned int, std::pair<double,double> > joint_pos_limits_;

};

}

#endif
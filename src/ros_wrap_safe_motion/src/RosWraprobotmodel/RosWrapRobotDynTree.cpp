#include "RosWraprobotmodel/RosWrapRobotDynTree.h"
#include <exception>
#include <stdexcept>

using namespace robot;

RobotDynClass::RobotDynClass(const std::string& modelFile)
{
    global_gravity_vector_.setZero();
    if(!modelFile.empty())
        loadModelFromFile(modelFile);
}



bool RobotDynClass::loadModelFromFile(const std::string& modelFile)
{
    bool ok = kinDynComp_.loadRobotModelFromFile(modelFile);
    if( !ok || getNrOfDegreesOfFreedom() == 0 )
    {
        throw std::runtime_error("Could not load robot model");
        return false;
    }

    urdf_url_ = modelFile;

    const iDynTree::Model & model = kinDynComp_.model();

    eigRobotState_.resize(model.getNrOfDOFs());
    idynRobotState_.resize(model.getNrOfDOFs());
    eigRobotState_.setFixedBase();
    for(unsigned int i = 0 ; i < kinDynComp_.getNrOfDegreesOfFreedom() ; i++)
    {
        double min = 0,max = 0;
        if(kinDynComp_.model().getJoint(i)->hasPosLimits())
        {
            min = kinDynComp_.model().getJoint(i)->getMinPosLimit(0);
            max = kinDynComp_.model().getJoint(i)->getMaxPosLimit(0);
            joint_pos_limits_[i] = {min,max};
        }
    }


    return ok;
}

void RobotDynClass::print() const
{
    std::cout << "Robot Model " << std::endl;
    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfJoints() ; i++)
    {
        std::cout << "      Joint " << i+1 << " " << kinDynComp_.getRobotModel().getJointName(i) << std::endl;
    }

    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfFrames() ; i++)
    {
        std::cout << "      Frame " << i << " " << kinDynComp_.getRobotModel().getFrameName(i) << std::endl;
    }
    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfLinks() ; i++)
    {
        std::cout << "      Link " << i << " " << kinDynComp_.getRobotModel().getLinkName(i) << std::endl;
    }
}

bool RobotDynClass::isInitialized() const
{
    return is_initialized_ && getNrOfDegreesOfFreedom() > 0;
}

void RobotDynClass::setGravity(const Eigen::Vector3d& g)
{
    global_gravity_vector_ = g;
}

void RobotDynClass::setBaseFrame(const std::string& base_frame)
{
    if(!frameExists(base_frame))
        throw std::runtime_error("Frame is not part of the robot");
    base_frame_ = base_frame;
    kinDynComp_.setFloatingBase( base_frame_ );
}

void RobotDynClass::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel)
{
    setRobotState(jointPos,jointVel,global_gravity_vector_);
}

void RobotDynClass::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    setRobotState(Eigen::Matrix4d::Identity(),jointPos,Eigen::Matrix<double,6,1>::Zero(),jointVel,gravity);
}

void RobotDynClass::setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    global_gravity_vector_ = gravity;

    if(getNrOfDegreesOfFreedom() == 0)
        throw std::runtime_error("Robot model is not loaded");

    if( base_frame_.empty())
        throw std::runtime_error("Base/FreeFloating frame is empty. Please use setBaseFrame before setting the robot state");

    if(global_gravity_vector_.isZero(0))
        throw std::runtime_error("Gravity vector is zero. Please use setGravity before setting the robot state");


    if(!is_initialized_)
    {

        is_initialized_ = true;
    }

    eigRobotState_.world_H_base = world_H_base;
    eigRobotState_.jointPos = jointPos;
    eigRobotState_.baseVel = baseVel;
    eigRobotState_.jointVel = jointVel;
    eigRobotState_.gravity = gravity;

    iDynTree::fromEigen(idynRobotState_.world_H_base,eigRobotState_.world_H_base);
    iDynTree::toEigen(idynRobotState_.jointPos) = eigRobotState_.jointPos;
    iDynTree::fromEigen(idynRobotState_.baseVel,eigRobotState_.baseVel);
    iDynTree::toEigen(idynRobotState_.jointVel) = eigRobotState_.jointVel;
    iDynTree::toEigen(idynRobotState_.gravity)  = eigRobotState_.gravity;

    kinDynComp_.setRobotState(idynRobotState_.world_H_base
                            ,idynRobotState_.jointPos
                            ,idynRobotState_.baseVel
                            ,idynRobotState_.jointVel
                            ,idynRobotState_.gravity);
}

const std::string& RobotDynClass::getBaseFrame() const
{
    return base_frame_;
}

const std::string& RobotDynClass::getFileURL() const
{
    return urdf_url_;
}

unsigned int RobotDynClass::getNrOfDegreesOfFreedom() const
{
    return kinDynComp_.getNrOfDegreesOfFreedom();
}

unsigned int RobotDynClass::configurationSpaceDimension() const
{
    return 6 + kinDynComp_.getNrOfDegreesOfFreedom();
}

const iDynTree::Model& RobotDynClass::getRobotModel()
{
    return kinDynComp_.getRobotModel();
}

const std::map<unsigned int, std::pair<double,double> >& RobotDynClass::getJointPositionLimits()
{
    return joint_pos_limits_;
}

bool RobotDynClass::frameExists(const std::string& frame_name)
{
    if(kinDynComp_.getFrameIndex(frame_name) < 0)
    {
        return false;
    }
    return true;
}

std::string RobotDynClass::getJointName(unsigned int idx)
{
    return kinDynComp_.model().getJointName(idx);
}

unsigned int RobotDynClass::getNrOfJoints()
{
    return kinDynComp_.model().getNrOfJoints();
}

 Eigen::Matrix<double,4,4,Eigen::RowMajor> RobotDynClass::getRelativeTransform(const std::string& refFrameName, const std::string& frameName) {
    return iDynTree::toEigen(kinDynComp_.getRelativeTransform(refFrameName,frameName).asHomogeneousTransform());
}


const Eigen::Matrix<double,6,1> RobotDynClass::getFrameVel(const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getFrameVel(frameName));
}

Eigen::Map< const Eigen::Matrix<double,6,1> > RobotDynClass::getFrameBiasAcc(const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getFrameBiasAcc(frameName));
}







const Eigen::VectorXd& RobotDynClass::getJointPos()
{
    return eigRobotState_.jointPos;
}

const Eigen::VectorXd& RobotDynClass::getJointVel()
{
    return eigRobotState_.jointVel;
}



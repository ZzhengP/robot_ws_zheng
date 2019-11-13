#include "sepPlane/sepPlane.h"


Planes::Planes(int N,int nbrRobobotPart,int nbrObs, Eigen::MatrixXd robotVertices, Eigen::MatrixXd obsVertices)
    :N_(N), nbrRobotPart_(nbrRobobotPart), nbrObs_(nbrObs){

    nbrPlane_ = (nbrRobotPart_ * nbrObs_)*(N_ - 1 );
    planesData_.resize(4,nbrPlane_);

    robotVertices_.resize(robotVertices.rows(), robotVertices.cols());
    obsVertices_.resize(obsVertices.rows(), obsVertices.cols());

}



Planes::Planes(int N,int nbrRobobotPart,int nbrObs, Eigen::MatrixXd robotVertices)
    :N_(N), nbrRobotPart_(nbrRobobotPart), nbrObs_(nbrObs){

    if ( nbrObs_ == 0){
        nbrPlane_ = nbrRobotPart_ *(N_ - 1 );
    }else {
        nbrPlane_ = (nbrRobotPart_ * nbrObs_)*(N_ - 1 );
}

    planesData_.resize(4,nbrPlane_);

    robotVertices_.resize(robotVertices.rows(), robotVertices.cols());
    InitializePlanesData();

    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << FRED("load separating planes module : \n") <<  std::endl;

    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << FYEL("separating planes module is initialized with \n") << nbrRobotPart_ << FYEL(" robot's part to take considered and ")
              << nbrObs << FYEL(" number of obstacle") <<  std::endl;

    std::cout <<FYEL("planes data are:  \n")<< planesData_ << std::endl;

}

void Planes::InitializePlanesData(){

    if (nbrObs_ == 0){

        for (int i(0); i < nbrPlane_ ; i++)
        {
            planesData_.block(0,i,4,1) << 1.,0.,0.,1.5;
        }
    }else {

        for (int i(0); i < nbrRobotPart_ ; i++)
        {
            for (int j(0); j < nbrObs_ ; j++){
            Eigen::Vector3d ptsOnPlane, verticeRobot, verticeObs, vect_normal ;
            verticeRobot = robotVertices_.block(0,i,3,1);
            verticeObs = obsVertices_.block(0,j,3,1);
            ptsOnPlane = verticeRobot + (verticeObs - verticeRobot)/2 ;
            vect_normal = verticeObs - verticeRobot;
            vect_normal.normalize();
            double dist;
            dist = ptsOnPlane.dot(vect_normal);

            planesData_.block(0,i*j+j,4,1) << vect_normal.x(), vect_normal.y(), vect_normal.z(), dist ;
            }
        }
    }
}




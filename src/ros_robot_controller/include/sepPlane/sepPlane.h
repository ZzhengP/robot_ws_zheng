#include "iostream"
#include "ros/ros.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "geometry_msgs/PoseArray.h"
#include "commun/color.h"
#include "commun/RosCommunDefinition.h"

class Planes
{
public:
    Planes(int N,int nbrRobobotPart,int nbrObs, Eigen::MatrixXd robotVertices, Eigen::MatrixXd obsVertices);
    Planes(int N,int nbrRobobotPart,int nbrObs, Eigen::MatrixXd robotVertices);

    void InitializePlanesData();

private:

    int N_, nbrPlane_;
    int nbrRobotPart_, nbrObs_;

    // Rows defines the position of each vertice, and cols defins the number of vertice
    Eigen::MatrixXd robotVertices_, obsVertices_;

    // Rows defines the data about each plane, and cols defines the number of plane

    // 4 x (nbrRobotPart * nbrObs), first part related to all Obs, and then
    Eigen::MatrixXd planesData_;

};

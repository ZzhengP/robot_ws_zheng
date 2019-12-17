#include "iostream"
#include "ros/ros.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "geometry_msgs/PoseArray.h"
#include "commun/color.h"
#include "commun/RosCommunDefinition.h"


// This definition of structure gives a way to represent all vertices of each robot's
// link or each osbtacle in the form of matrix organized into std vector form. The size of
// vector depend to the number of robot link or obstacle, and the size of matrix into


class Planes
{
public:
    Planes(int N,int nbrRobobotPart,int nbrObs, std::vector<Eigen::MatrixXd> robotVertices, Eigen::MatrixXd obsVertices);
    Planes(int N,int nbrRobobotPart,int nbrObs, Eigen::MatrixXd robotVertices);

    void InitializePlanesData();


    struct VerticesData{

           int nbrOfObject;
           int nbrVertices;
           int N;
           // vector is indiced by the number of object
           // each composant of vector is a matrix which contains a succissive of vertices set
           // related to the horizon of prediction
//           typedef Eigen::Matrix<double, 4, 1> Matrix4;
//           std::vector<Matrix4> Vertices;
           std::vector<Eigen::MatrixXd> Vertices;

    };

    struct PlaneData{
           int nbrPlane;
           int N;
           std::vector<Eigen::MatrixXd> Planes;
    };

    PlaneData getPlanes(){
              return planesData_;
    }
private:

    int N_, nbrPlane_;
    int nbrRobotPart_, nbrObs_;

    VerticesData robotVertices_;
    VerticesData obsVertices_;

    // (nbrRobotPart * nbrObs)*4, first part related to all Obs, and the
    PlaneData  planesData_;

};

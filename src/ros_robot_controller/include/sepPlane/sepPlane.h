#pragma once
#include "iostream"
#include "ros/ros.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "geometry_msgs/PoseArray.h"
#include "commun/color.h"
#include "commun/RosCommunDefinition.h"
#include "qpOASES.hpp"
#include "solver/RosMpcSolver.h"
// This definition of structure gives a way to represent all vertices of each robot's
// link or each osbtacle in the form of matrix organized into std vector form. The size of
// vector depend to the number of robot link or obstacle, and the size of matrix into

/**
 * @brief The VerticesData struct
 */
struct VerticesData{

    /**
        * @brief number of object in this data
        */
       int nbrOfObject;
       /**
        * @brief number of vertices of each object
        */
       int nbrVertices;

       /**
        * @brief N horizon
        */
       int N;
       // vector is indiced by the number of object
       // each composant of vector is a matrix which contains a succissive of vertices set
       // related to the horizon of prediction
//           typedef Eigen::Matrix<double, 4, 1> Matrix4;
//           std::vector<Matrix4> Vertices;

       /**
        * @brief Object are stored into a vector
        * each composant of vector is a matrix which contains a succissive of vertices set
        * related to the horizon of prediction
        */
       std::vector<Eigen::MatrixXd> Vertices;

};


/**
 * @brief More like a data structure to store planes
 */
class Planes
{
public:

    /**
     * @brief Planes are initialized with predifined robot's vertices and variable obsVertices
     * @param N
     * @param nbrRobobotPart
     * @param nbrObs
     * @param robotVertices
     * @param obsVertices
     */
    Planes(int N,int nbrRobobotPart,int nbrObs, std::vector<Eigen::MatrixXd> robotVertices, Eigen::MatrixXd obsVertices);
    Planes(int N,int nbrRobobotPart,int nbrObs, Eigen::MatrixXd robotVertices);

    /**
     * @brief InitializePlanesData
     * robotVertices, obsVertices
     * Planes data
     */
    void InitializePlanesData();

    struct PlaneData{
           int nbrPlane;
           int N;
           std::vector<Eigen::MatrixXd> planeLocation;
    };

    PlaneData getPlanes(){
              return planesData_;
    }


private:
    /**
     * @brief N_: horizon of prediction
     */
    int N_, nbrPlane_;
    /**
     * @brief nbrRobotPart_
     */
    int nbrRobotPart_, nbrObs_;

    VerticesData robotVertices_;
    VerticesData obsVertices_;
    // (nbrRobotPart * nbrObs)*5, first part related to all Obs, and the
    PlaneData  planesData_;
    // Define an solver for linear programming
};

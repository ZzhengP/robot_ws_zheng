#ifndef RosWrapsepPlane_H
#define RosWrapsepPlane_H
#pragma once
#include "iostream"
#include "ros/ros.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "RosWrapcommun/color.h"
#include "RosWrapcommun/RosWrapCommunDefinition.h"
#include "RosWrapsolver/RosWrapPlaneSolver.h"
#include "memory"
#include "qpOASES.hpp"
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseArray.h"
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"
#include "std_msgs/Float64MultiArray.h"
#include "eigen_conversions/eigen_msg.h"
#include "eigen_conversions/eigen_kdl.h"
// This definition of structure gives a way to represent all vertices of each robot's
// link or each osbtacle in the form of matrix organized into std vector form. The size of
// vector depend to the number of robot link or obstacle, and the size of matrix into


namespace plane {
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
    Planes(ros::NodeHandle* nodehandle,const  int& N,const std::vector<Eigen::MatrixXd>& obsVerticesAugmented,const double& dsafe, const std::string& robotVerticesTopic);

    ~Planes(){
        ROS_INFO("Delete ros robot and obstacle vertices subscriber ")   ;
    }
    /**
     * @brief Call back function for receiving robot's position 
     * 
     * @param vertices_markers 
     */
    void robotVerticesCallback(const visualization_msgs::MarkerArray::ConstPtr & vertices_markers);

    void obsVerticesCallback(const std_msgs::Float64MultiArray::ConstPtr& obsvertices);
    /**
     * @brief InitializePlanesData
     * robotVertices, obsVertices
     * Planes data
     */
    void InitializePlanesData();

    void update();

    void run();

    struct PlaneData{
           int nbrPlane;
           int N;
           std::vector<Eigen::MatrixXd> planeLocation;
           void print(){
               std::cout << "Number of different separating plane: " << planeLocation.size() <<'\n';
               std::cout << "Number of one separating plane during interval of prediction : " << planeLocation[0].cols() << '\n';
               std::cout << "one separating plane data during interval of prediction : \n " << planeLocation[0] <<'\n' ;

           }
    };

    PlaneData getPlanes(){
              return planesData_;
    }

    PlaneData  planesData_;


    protected:

    /**
     * @brief Ros node to compute plane's location
     * 
     */
    

    ros::NodeHandle nh_;

    ros::Subscriber robot_vertices_subscriber_ ;

    ros::Subscriber obs_vertices_subscriber_ ;

    ros::Publisher plane_data_publisher_;

    std::string robot_vertices_topic_;
    
    rviz_visual_tools::RvizVisualToolsPtr planesPub_ , cubeObstacleMarkers_ ;   
    
    rviz_visual_tools::colors colorPlane_ = rviz_visual_tools::TRANSLUCENT_DARK;
    double plane_x_width_ = 2, plane_y_width_ = 2;

    /**
     * @brief N_: horizon of prediction
     */
    int N_, nbrPlane_;
    /**
     * @brief nbrRobotPart_
     */
    int nbrRobotPart_, nbrObs_;

    bool initialize = true;
    VerticesData robotVertices_;
    VerticesData obsVertices_;

    std::vector<Eigen::MatrixXd> robotVerticesAugmented_;
    std::vector<Eigen::MatrixXd> obsVerticesAugmented_;

    geometry_msgs::PoseArray obsVerticesPose_;
    Eigen::Vector3d obsCenter_;
    // (nbrRobotPart * nbrObs)*5, first part related to all Obs, and the
    // Define an solver for linear programming


    // ---------------------   Parameter related to plane solver such as H, g , A, lbA, ubA, lb, ub ----------------------------------
    /**
     * @brief Planes solver 
     * 
     */
    std::shared_ptr<PlaneSolver> plane_solver_;
    double dsafe_;
    Eigen::VectorXd gPlane_;
    rviz_visual_tools::colors colorPlane2 = rviz_visual_tools::RED;
};
}

#endif
#pragma once
#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include "visualization_msgs/MarkerArray.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "geometry_msgs/PoseArray.h"
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"

class markers{

public:
    markers(ros::NodeHandle *nh, std::vector<Eigen::Vector3d> initializePos, std::vector<Eigen::Vector4d> initializeOri);
    markers(ros::NodeHandle *nh);
    markers( );
    void setMarkersPos(std::vector<Eigen::Vector3d> ptsPos, std::vector<Eigen::Vector4d> initializeOri);
    visualization_msgs::MarkerArray getMarkers(){
        return markers_;
    }
    void setMarkers(const geometry_msgs::PoseArray &Poses);

    std::vector<Eigen::Vector3d> getMarkersPos();
    int getMarkersNbr();
    void markerPublish();
protected:
    void initializeSubscriber();
    void initializePublisher();
    void initializeMarkers(std::vector<Eigen::Vector3d> initializePos, std::vector<Eigen::Vector4d> initializeOri);
    void subscriberCallback(const geometry_msgs::PoseArray& Poses );
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher  publisher_;
    visualization_msgs::MarkerArray markers_;
    std::vector<Eigen::Vector3d> markers_pos_;

};

#pragma once
#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include "visualization_msgs/MarkerArray.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "geometry_msgs/PoseArray.h"
class markers{

public:
    markers(ros::NodeHandle *nh, std::vector<Eigen::Vector3d> initializePos, std::vector<Eigen::Vector4d> initializeOri);
    markers(ros::NodeHandle *nh);
    void setMarkersPos(std::vector<Eigen::Vector3d> ptsPos, std::vector<Eigen::Vector4d> initializeOri);
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

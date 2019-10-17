#include "visualization/RosMarkers.h"


markers::markers(ros::NodeHandle *nodehandle, std::vector<Eigen::Vector3d> initializePos, std::vector<Eigen::Vector4d> initializeOri):nh_(*nodehandle){


    initializePublisher();
    initializeMarkers(initializePos, initializeOri);
}


void markers::initializePublisher(){

        ROS_INFO("Initialize Markers array's publisher node");
        publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "/ur5/visualization_marker", 1000);
}


void markers::initializeSubscriber(){

}


void markers::setMarkersPos(std::vector<Eigen::Vector3d> ptsPos, std::vector<Eigen::Vector4d> initializeOri){

    for (size_t t(0); t < ptsPos.size(); t++){
        markers_.markers[t].pose.position.x = ptsPos[t].x();
        markers_.markers[t].pose.position.y = ptsPos[t].y();
        markers_.markers[t].pose.position.z = ptsPos[t].z();
    }
}

std::vector<Eigen::Vector3d> markers::getMarkersPos(){
        std::vector<Eigen::Vector3d> markers_position ;
        markers_position.resize(markers_.markers.size());


        for (size_t t(0); t < markers_.markers.size(); t++){
            markers_position[0].x() = markers_.markers[t].pose.position.x;
            markers_position[0].y() = markers_.markers[t].pose.position.y;
            markers_position[0].z() = markers_.markers[t].pose.position.z;

        }
        return markers_position ;
}
void markers::initializeMarkers(std::vector<Eigen::Vector3d> initializePos, std::vector<Eigen::Vector4d> initializeOri){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    for (size_t t(0); t < initializePos.size(); t++){

        marker.id = t;
        marker.pose.position.x = initializePos[t].x();
        marker.pose.position.y = initializePos[t].y();
        marker.pose.position.z = initializePos[t].z();
        marker.pose.orientation.x = initializeOri[t].x();
        marker.pose.orientation.y = initializeOri[t].y();
        marker.pose.orientation.z = initializeOri[t].z();
        marker.pose.orientation.w = initializeOri[t].w();
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(10);
        markers_.markers.push_back(marker);

    }
}

void markers::markerPublish(){
     publisher_.publish(markers_);
}

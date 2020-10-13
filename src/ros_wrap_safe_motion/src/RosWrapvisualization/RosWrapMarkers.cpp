#include "RosWrapvisualization/RosWrapMarkers.h"


markers::markers(){

}


markers::markers(ros::NodeHandle *nodehandle, std::vector<Eigen::Vector3d> initializePos, std::vector<Eigen::Vector4d> initializeOri):
    nh_(*nodehandle){


    initializePublisher();
}

markers::markers(ros::NodeHandle *nodehandle):nh_(*nodehandle){

    initializePublisher();
    initializeSubscriber();
}
void markers::initializePublisher(){

        ROS_INFO("Initialize Markers array's publisher node");
        publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "/panda/visualization_marker", 10);
}


void markers::initializeSubscriber()
{
    ROS_INFO("Initializing Subscribers");
    subscriber_ = nh_.subscribe("/panda/panda_ee_pose", 1, &markers::subscriberCallback,this);
    // add more subscribers here, as needed
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

void markers::subscriberCallback(const geometry_msgs::PoseArray &Poses){
    visualization_msgs::Marker marker, marker_virt;
     visualization_msgs::MarkerArray markers;
    marker.header.frame_id = Poses.header.frame_id;
    marker.header.stamp = Poses.header.stamp;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker_virt.header.frame_id = Poses.header.frame_id;
    marker_virt.header.stamp = Poses.header.stamp;
    marker_virt.type = visualization_msgs::Marker::CUBE;
    marker_virt.action = visualization_msgs::Marker::ADD;

    marker_virt.pose.position.x = 0.4;
    marker_virt.pose.position.y = 0.2;
    marker_virt.pose.position.z = 0.5;
    marker_virt.pose.orientation.x = Poses.poses[0].orientation.x;
    marker_virt.pose.orientation.y = Poses.poses[0].orientation.y;
    marker_virt.pose.orientation.z = Poses.poses[0].orientation.z;
    marker_virt.pose.orientation.w = Poses.poses[0].orientation.w;
    marker_virt.scale.x = 0.1;
    marker_virt.scale.y = 0.1;
    marker_virt.scale.z = 0.1;
    marker_virt.color.a = 1.0; // Don't forget to set the alpha!
    marker_virt.color.r = 1.0;
    marker_virt.color.g = 0.0;
    marker_virt.color.b = 0.0;
    marker_virt.lifetime = ros::Duration(0.0);
    markers.markers.push_back(marker_virt);

    for (size_t t(0); t < Poses.poses.size(); t++){
        marker.id = t;
        marker.pose.position.x = Poses.poses[t].position.x;
        marker.pose.position.y = Poses.poses[t].position.y;
        marker.pose.position.z = Poses.poses[t].position.z;
        marker.pose.orientation.x = Poses.poses[t].orientation.x;
        marker.pose.orientation.y = Poses.poses[t].orientation.y;
        marker.pose.orientation.z = Poses.poses[t].orientation.z;
        marker.pose.orientation.w = Poses.poses[t].orientation.w;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0.0);
        markers.markers.push_back(marker);

    }
        publisher_.publish(markers);
        markers.markers.clear();
}

void markers::initializeMarkers(std::vector<Eigen::Vector3d> initializePos, std::vector<Eigen::Vector4d> initializeOri){

}


int markers::getMarkersNbr(){
   return  markers_.markers.size();
}
void markers::markerPublish(){
     publisher_.publish(markers_);
}

void markers::setMarkers(const geometry_msgs::PoseArray &Poses){
    markers_.markers.clear();
    visualization_msgs::Marker marker, marker_virt;
     visualization_msgs::MarkerArray markers;
    marker.header.frame_id = Poses.header.frame_id;
    marker.header.stamp = Poses.header.stamp;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker_virt.header.frame_id = Poses.header.frame_id;
    marker_virt.header.stamp = Poses.header.stamp;
    marker_virt.type = visualization_msgs::Marker::CUBE;
    marker_virt.action = visualization_msgs::Marker::ADD;

    marker_virt.pose.position.x = 0.4;
    marker_virt.pose.position.y = 0.2;
    marker_virt.pose.position.z = 0.5;
    marker_virt.pose.orientation.x = Poses.poses[0].orientation.x;
    marker_virt.pose.orientation.y = Poses.poses[0].orientation.y;
    marker_virt.pose.orientation.z = Poses.poses[0].orientation.z;
    marker_virt.pose.orientation.w = Poses.poses[0].orientation.w;
    marker_virt.scale.x = 0.1;
    marker_virt.scale.y = 0.1;
    marker_virt.scale.z = 0.1;
    marker_virt.color.a = 1.0; // Don't forget to set the alpha!
    marker_virt.color.r = 1.0;
    marker_virt.color.g = 0.0;
    marker_virt.color.b = 0.0;
    marker_virt.lifetime = ros::Duration(0.0);
    markers.markers.push_back(marker_virt);

    for (size_t t(0); t < Poses.poses.size(); t++){
        marker.id = t;
        marker.pose.position.x = Poses.poses[t].position.x;
        marker.pose.position.y = Poses.poses[t].position.y;
        marker.pose.position.z = Poses.poses[t].position.z;
        marker.pose.orientation.x = Poses.poses[t].orientation.x;
        marker.pose.orientation.y = Poses.poses[t].orientation.y;
        marker.pose.orientation.z = Poses.poses[t].orientation.z;
        marker.pose.orientation.w = Poses.poses[t].orientation.w;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0.0);
        markers_.markers.push_back(marker);

    }
}

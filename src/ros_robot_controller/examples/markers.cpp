// ros
#include "ros/ros.h"
#include <fstream>
#include <thread>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include "cmath"
#include "visualization/RosMarkers.h"
#include "iostream"
#include <boost/timer.hpp>

using namespace  std;


int main(int argc, char **argv)
{

    ros::init(argc,argv,"markers_visualization");
    ros::NodeHandle n;

    ROS_INFO("instantiating a node for markers visualization");
    markers markers(&n);
    ros::spin();

    return 0;
}

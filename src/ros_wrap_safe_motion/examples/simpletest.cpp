#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/Float32MultiArray.h"
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"
#include "Eigen/Core"
#include "Eigen/Dense"
class PointCloudToImage
{
public:

  PointCloudToImage () : cloud_topic_("/camera/depth_registered/points"),image_topic_("/pcl/output")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
     
     keypoints_sub_ = nh_.subscribe("/keypoints", 30, &PointCloudToImage::keyCallback, this);
                      
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);
    keypoint_marker_.reset(new rviz_visual_tools::RvizVisualTools("camera_rgb_optical_frame","/Keypoints"));

    pcl_cloud_ = new pcl::PCLPointCloud2;
    cloud_rgb_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    key_point_.resize(8,2);
    key_point_.setZero();
    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }


  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    width_ = cloud->width ;
    height_ = cloud->height;
    cloud_rgb_->resize(width_*height_);
    pcl::fromROSMsg(*cloud, *cloud_rgb_);
    Eigen::MatrixXd poses;
    Eigen::Vector3d pose;
    poses.resize(8,3) ;
    std::cout <<" cloud frame : "<< cloud->header.frame_id << std::endl;
    for (int i(0) ; i < 8; i++){

    // std::cout <<" keypoints : x " << cloud_rgb_->points[key_point_(i,0)*key_point_(i,1)].x <<std::endl;
    // std::cout <<" keypoints : y " << cloud_rgb_->points[key_point_(i,0)*key_point_(i,1)].y <<std::endl;
    // std::cout <<" keypoints : z " << cloud_rgb_->points[key_point_(i,0)*key_point_(i,1)].z <<std::endl;
    poses(i,0) = cloud_rgb_->points[key_point_(i,0)*key_point_(i,1)].x;
    poses(i,1)  = cloud_rgb_->points[key_point_(i,0)*key_point_(i,1)].y;
    poses(i,2)  = cloud_rgb_->points[key_point_(i,0)*key_point_(i,1)].z;
    std::cout << " keypoint :\n "  << poses << std::endl; 
    pose << poses(i,0), poses(i,1), poses(i,2);
    keypoint_marker_->publishSphere(pose,rviz_visual_tools::GREEN,0.05);

    }
            // cubeObstacleMarkers->publishCylinder(point1,point2,colorPlane2,0.1);
    keypoint_marker_->trigger(); 
    // pcl::toROSMsg(*cloud_rgb_,image_);
    keypoint_marker_->deleteAllMarkers();
  }

void keyCallback(const std_msgs::Float32MultiArrayConstPtr& msg){
   std_msgs::Float32MultiArray new_data ;
   new_data.data = msg->data;

   for (int i(0); i < 8; i ++){
     key_point_(i,0) = msg->data[2*i];
     key_point_(i,1) = msg->data[2*i+1];
   }                            
}

  ~PointCloudToImage(){
    delete [] pcl_cloud_;
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
  ros::Subscriber keypoints_sub_;
  pcl::PCLPointCloud2 *pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_;
  int width_, height_ ;
  Eigen::MatrixXd key_point_;
  rviz_visual_tools::RvizVisualToolsPtr keypoint_marker_;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "keypoint_node");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
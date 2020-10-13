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
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    pcl_cloud_ = new pcl::PCLPointCloud2;
    cloud_rgb_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
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
    // pcl::toROSMsg (*cloud, image_); //convert the cloud
    pcl::fromROSMsg(*cloud, *cloud_rgb_);
    pcl::toROSMsg (*cloud_rgb_, image_);
   
    // pcl::toROSMsg(*cloud_rgb_,image_);
    image_pub_.publish (image_); //publish our cloud image
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
  pcl::PCLPointCloud2 *pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_;
  int width_, height_ ;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}





// #include <iostream>
// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include "std_msgs/Float32MultiArray.h"
// #include "sensor_msgs/Image.h"
// #include "Eigen/Core"
// #include "Eigen/Dense"
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <boost/typeof/typeof.hpp>
// // #include <opencv2/opencv.hpp>

// static const std::string OPENCV_WINDOW = "Image Window";

// typedef union U_FloatParse {
//     float float_data;
//     unsigned char byte_data[4];
// } U_FloatConvert;

// class depthInference{

//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     image_transport::Publisher image_pub_;
//     ros::Subscriber keypoints_sub_;
// public:

//   depthInference(): it_(nh_){
//     std::cout <<" hello node image:" <<std::endl;
//      image_sub_ = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 5, &depthInference::depthCallback, this);
//     // image_sub_ = it_.subscribe("/camera/depth_registered/hw_registered/image_rect_raw", 5, &depthInference::depthCallback, this);
//     keypoints_sub_ = nh_.subscribe("/keypoints", 5, &depthInference::keyCallback, this);
//     key_point_.resize(8,2);
//     key_point_.setZero();
//     depth_data_.resize(8);
//     keypoint_name_.resize(8);
//     keypoint_name_ = {"nose", "neck", "r_sho", "r_elb", "r_wri", "l_sho", "l_elb", "l_wri"};

//     cv::namedWindow(OPENCV_WINDOW);
// }


// ~depthInference(){
//     cv::destroyWindow(OPENCV_WINDOW);
// }


// void depthCallback(const sensor_msgs::ImageConstPtr& msg){


  
//     for (int i(0); i < 8; i ++){
    
//      depth_data_(i) = msg->data[key_point_(i,0),key_point_(i,1)];
//      std::cout <<" depth_data_ : " << depth_data_(i) <<std::endl;
//    }                          

//     cv_bridge::CvImagePtr cv_depth_ptr; 
//     cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1 );
//     cv::Mat depth_mat ; 

//      cv_depth_ptr -> image.convertTo(depth_mat, CV_16UC1);

//     std::cout << "depth mat size : " << depth_mat.rows << ", "<< depth_mat.cols << "\n";
//     for (int i(0); i < 8; i ++){
//       cv::circle(cv_depth_ptr->image, cv::Point(key_point_(i,0),key_point_(i,1)), 10, CV_RGB(255,0,0));
//       // std::cout <<" depth 8 points : "<< keypoint_name_.at(i) << " is : " << depth_mat.at<float>(key_point_(i,0),key_point_(i,1))<< '\n';
//        std::cout <<" depth 8 points : "<< keypoint_name_.at(i)<< " is : " << cv_depth_ptr->image.at<float>(key_point_(i,0),key_point_(i,1))<<'\n'; 
//       // std::cout <<" cv mat  : " << depth_mat << std::endl;
//    }
  
//     cv::imshow(OPENCV_WINDOW,cv_depth_ptr->image);
//     cv::waitKey(3);


// }

// void keyCallback(const std_msgs::Float32MultiArrayConstPtr& msg){
//    std_msgs::Float32MultiArray new_data ;
//    new_data.data = msg->data;

//    for (int i(0); i < 8; i ++){
//      key_point_(i,0) = msg->data[2*i];
//      key_point_(i,1) = msg->data[2*i+1];
//    }                            
// }

// private:
//   Eigen::MatrixXd key_point_;
//   Eigen::VectorXd depth_data_;
//   std::vector<std::string> keypoint_name_;
//   // image_transport::Subscriber depth_image_sub_ ; 
// };  



// int main(int argc, char ** argv) {
   

//     ros::init(argc, argv, "depth_Publish_node");

//     depthInference depthFc;

//     ros::spin();

//     return 0;

// }

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "eigen_conversions/eigen_kdl.h"


Eigen::MatrixXd msgToEigen(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  Eigen::MatrixXd eigenMatrix;
  int rows, cols, rows_stride, cols_stride ;

  rows = msg->layout.dim[0].size;
  cols = msg->layout.dim[1].size;

  rows_stride = msg->layout.dim[0].stride;
  cols_stride = msg->layout.dim[1].stride;

  eigenMatrix.resize(rows,cols);
  eigenMatrix.setZero();
  int ii =0;

 for (int i = 0; i < rows; ++i){
      for (int j = 0; j < cols; ++j){
          eigenMatrix(i,j) = msg->data[ii++];
      }
 }
	
  
  return eigenMatrix;
}

void msgToEigenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){

    Eigen::MatrixXd eigenMatrix;

  eigenMatrix = msgToEigen(msg);

  std::cout << " Received Matrix :\n " << eigenMatrix << std::endl;
}

int main(int argc, char **argv){
    
ros::init(argc, argv, "subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("plane_data", 1, msgToEigenCallback);
	ros::spin();
  return 0;
}
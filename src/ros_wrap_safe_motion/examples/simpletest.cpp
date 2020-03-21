#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "eigen_conversions/eigen_kdl.h"

int main(int argc, char **argv){
    
    ros::init(argc,argv,"test_eigen");
    ros::NodeHandle n;

  
    Eigen::MatrixXd testMatrix;

    testMatrix.resize(5,5);
     testMatrix << 1,2,3,4,5,
                  6,7,8,9,10,
                  11,12,13,14,15,
                  16,17,18,19,20,
                  21,22,23,24,25;

    std_msgs::Float64MultiArray msgMatrix; 


    tf::matrixEigenToMsg(testMatrix,msgMatrix);    
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("test_matrix", 1000);

    ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

 
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msgMatrix);

    ros::spinOnce();

    loop_rate.sleep();
  }


    return 0;
}
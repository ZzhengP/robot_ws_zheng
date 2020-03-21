#include "ros/ros.h"


int main (int argc, char **argv){

    ros::init(argc,argv,"getparameter");
    ros::NodeHandle nh;
    int N;
    double W1, W2;
    nh.getParam("/N",N);
    nh.getParam("/task_1_weight",W1);
    nh.getParam("/task_2_weight", W2);

    
    std::cout <<" N prediciton : \n" << N << std::endl;

    std::cout <<" weight of task 1 :\n  " << W1 << std::endl;
    std::cout <<" weight of task 2 :\n  " << W2 << std::endl;

    return 0;
}
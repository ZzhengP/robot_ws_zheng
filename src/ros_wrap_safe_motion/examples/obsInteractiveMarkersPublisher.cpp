#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "eigen_conversions/eigen_kdl.h"
#include <fstream>
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"
#include "visualization_msgs/InteractiveMarkerUpdate.h"
#include "geometry_msgs/Pose.h"
using namespace  std;

const double pi = 3.1415927;


Eigen::MatrixXd ComputeObstacleVertices(Eigen::Vector3d centre, Eigen::Vector3d size){

    Eigen::MatrixXd Vertices;
    Vertices.resize(3,8);

    Vertices.block(0,0,1,8) << centre(0) - size(0), centre(0) - size(0), centre(0) - size(0), centre(0) - size(0),
                               centre(0) + size(0), centre(0) + size(0), centre(0) + size(0), centre(0) + size(0);

    Vertices.block(1,0,1,8) << centre(1) + size(1), centre(1) + size(1), centre(1) - size(1), centre(1) - size(1),
                               centre(1) + size(1), centre(1) + size(1), centre(1) - size(1), centre(1) - size(1);

    Vertices.block(2,0,1,8) << centre(2) - size(2), centre(2) + size(2), centre(2) - size(2), centre(2) + size(2),
                               centre(2) - size(2), centre(2) + size(2), centre(2) - size(2), centre(2) + size(2);

    return  Vertices;
}



#define MAXBUFSIZE  ((int) 1e6)

Eigen::MatrixXd readMatrix(const char *filename)
    {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    ifstream infile;
    infile.open(filename);
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        }

    infile.close();

    rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    return result;
    };


class simpleObsMarkerPublisher {   

    public:

        simpleObsMarkerPublisher(ros::NodeHandle* nodehandle, const int & N, const int &numberOfObstacle, const Eigen::Vector3d & obsSize, const Eigen::Vector3d & obsCentre)
                                :nh_(*nodehandle), N_(N), number_of_obstacle_(numberOfObstacle),obs_centre_(obsCentre),obs_size_(obsSize){


              ROS_INFO("simpleObsMarkerPublisher constructor ");

              obs_nbr_vertices_ = 8;
              obs_vertices.resize(3*number_of_obstacle_,obs_nbr_vertices_);

              obs_vertices = ComputeObstacleVertices(obs_centre_,obs_size_);


              obs_vertices_augmented_.resize(number_of_obstacle_);

              for (int i(0); i < number_of_obstacle_; i++){
                      obs_vertices_augmented_[i].resize(3,obs_nbr_vertices_*N_);
                      }
              for (int j(0); j < number_of_obstacle_; j++){
                  for (int i(0) ; i < N_ ; i++) {
                        obs_vertices_augmented_[j].block(0,i*obs_nbr_vertices_,3,obs_nbr_vertices_) = obs_vertices;
                      }
                    }

              ROS_INFO("simpleObsMarkerPublisher constructor terminate ");

        };





        // cubeObstacleMarkers->publishCylinder(point1,point2,colorPlane2,0.1);

    // cubeObstacleMarkers->trigger();
    // cubeObstacleMarkers->deleteAllMarkers();


        void callback(const geometry_msgs::Pose::ConstPtr& imarker_msg);


        void initRos(){

              chatter_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("obstacle_vertices", 1000);
              colorPlane2_  = rviz_visual_tools::RED ;
              cube_obs_publisher_.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/testObsVertices"));
              interactive_marker_subscriber   = nh_.subscribe("/interactivePose",100, &simpleObsMarkerPublisher::callback, this);

        };
        void update(){  

                  

     
                  obs_centre_ << interactive_marker_pos_.position.x, interactive_marker_pos_.position.y, interactive_marker_pos_.position.z ;
//                obsCentre << 0.7 , -0.2, 0.1;

                  obs_vertices_augmented_[0].block(0,0,3,8) = ComputeObstacleVertices(obs_centre_,obs_size_);
                  obs_vertices_augmented_[0].block(0,1*8,3,8) = ComputeObstacleVertices(obs_centre_,obs_size_);
                  obs_vertices_augmented_[0].block(0,2*8,3,8) = ComputeObstacleVertices(obs_centre_,obs_size_);
                  obs_vertices_augmented_[0].block(0,3*8,3,8) = ComputeObstacleVertices(obs_centre_,obs_size_);
                
              
           

                   tf::matrixEigenToMsg(obs_vertices_augmented_[0],msg_matrix_);    
                   chatter_pub_.publish(msg_matrix_);

                   Eigen::Vector3d point1, point2, point3, point4;
                   point1 << obs_vertices_augmented_[0](0,0) , obs_centre_(1), obs_centre_(2) ;
                   point2 << obs_vertices_augmented_[0](0,4) , obs_centre_(1), obs_centre_(2) ;
                   point3 << obs_vertices_augmented_[0](0,4*8),obs_centre_(1),obs_centre_(2);
                   point4 << obs_vertices_augmented_[0](0,4*8+4),obs_centre_(1),obs_centre_(2);

                   cube_obs_publisher_->publishCylinder(point1,point2,colorPlane2_,0.1);

                   cube_obs_publisher_->trigger();
                   cube_obs_publisher_->deleteAllMarkers();
           };

    

        void run(){

             ros::Rate loop_rate(1000);

             while (ros::ok){

                 update();
                 ros::spinOnce();
                 loop_rate.sleep();

             }
        };
    private:

       geometry_msgs::Pose interactive_marker_pos_;    
       ros::NodeHandle nh_ ;
       ros::Subscriber interactive_marker_subscriber;
       ros::Publisher chatter_pub_;
        rviz_visual_tools::RvizVisualToolsPtr cube_obs_publisher_;

       std_msgs::Float64MultiArray msgMatrix; 
       int N_, number_of_obstacle_ ;
       Eigen::Vector3d obs_size_, obs_centre_;
       Eigen::MatrixXd obs_vertices;
       std::vector<Eigen::MatrixXd> obs_vertices_augmented_;
       int obs_nbr_vertices_ ;
       rviz_visual_tools::colors colorPlane2_;


       std_msgs::Float64MultiArray msg_matrix_; 


};

void simpleObsMarkerPublisher::callback(const geometry_msgs::Pose::ConstPtr& imarker_msg){
      
                interactive_marker_pos_.position =  imarker_msg->position;

  };
int main(int argc, char **argv)
{


    // ------------------------------------   Initialize ------------------------------------------
    ros::init(argc,argv,"obs_vertices_publisher");
    ros::NodeHandle n;
 
 

    int N ;
    n.getParam("/N",N);

    Eigen::Vector3d obsSize, obsCentre;

    int numberOfObstacle;
    n.getParam("/numberOfObstacle", numberOfObstacle);

    for (size_t i(0); i < 3; i++ ){
        std::string s = std::to_string(i); ;
        n.getParam("/obsSize/s"+s , obsSize[i]);
        n.getParam("/obsCentre/s"+s ,obsCentre[i]);

    }

    simpleObsMarkerPublisher imaker_obs_publisher(&n,N,numberOfObstacle,obsSize,obsCentre);

    imaker_obs_publisher.initRos();
    imaker_obs_publisher.run();
    
    return 0;
}
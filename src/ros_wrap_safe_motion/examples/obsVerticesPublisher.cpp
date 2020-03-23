#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "eigen_conversions/eigen_kdl.h"
#include <fstream>
#include "rviz_visual_tools/rviz_visual_tools.h"//#include "rviz_visual_tools/rviz_visual_tools.h"

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



int main(int argc, char **argv)
{


    // ------------------------------------   Initialize ------------------------------------------
    ros::init(argc,argv,"obs_vertices_publisher");
    ros::NodeHandle n;

    char *file("/home/zheng/Bureau/obsCentrePath.txt");
    Eigen::MatrixXd obsPath ;

    obsPath = readMatrix(file);


    std_msgs::Float64MultiArray msgMatrix; 


    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("obstacle_vertices", 1000);

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

    Eigen::MatrixXd obsVertices;
    std::vector<Eigen::MatrixXd> obsVerticesAugmented;
    int obsNbrVertices = 8;
    obsVertices.resize(3*numberOfObstacle,obsNbrVertices);

    obsVertices = ComputeObstacleVertices(obsCentre,obsSize);

    obsVerticesAugmented.resize(numberOfObstacle);
    for (int i(0); i < numberOfObstacle; i++){
         obsVerticesAugmented[i].resize(3,obsNbrVertices*N);
    }

    for (int j(0); j < numberOfObstacle; j++){
        for (int i(0) ; i < N ; i++) {
            obsVerticesAugmented[j].block(0,i*obsNbrVertices,3,obsNbrVertices) = obsVertices;
        }
    }
    rviz_visual_tools::colors colorPlane2 = rviz_visual_tools::RED;
    rviz_visual_tools::RvizVisualToolsPtr cubeObstacleMarkers;
    cubeObstacleMarkers.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/testObsVertices"));
   
    ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int ite = 0;
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

     for (int j(0); j < numberOfObstacle; j++){

        for (int i(0) ; i < N ; i++) {
            if (ite > obsPath.rows() - 10){
                obsCentre << obsPath(obsPath.rows()-10,0), obsPath(obsPath.rows()-10,1), obsPath(obsPath.rows()-10,2);
//                obsCentre << 0.7 , -0.2, 0.1;
                obsVerticesAugmented[j].block(0,8*i,3,8) = ComputeObstacleVertices(obsCentre,obsSize);
            }else {
             obsCentre << obsPath(ite+i,0), obsPath(ite+i,1), obsPath(ite+i,2);
//                obsCentre << 0.7 , -0.2, 0.1;

             obsVerticesAugmented[j].block(0,8*i,3,8) = ComputeObstacleVertices(obsCentre,obsSize);
            }

        }
    }

        Eigen::Vector3d point1, point2, point3, point4;
        point1 << obsVerticesAugmented[0](0,0) , obsCentre(1), obsCentre(2) ;
        point2 << obsVerticesAugmented[0](0,4) , obsCentre(1), obsCentre(2) ;
        point3 << obsVerticesAugmented[0](0,4*8),obsCentre(1),obsCentre(2);
        point4 << obsVerticesAugmented[0](0,4*8+4),obsCentre(1),obsCentre(2);

        // cubeObstacleMarkers->publishCylinder(point1,point2,colorPlane2,0.1);

    // cubeObstacleMarkers->trigger();
    // cubeObstacleMarkers->deleteAllMarkers();
    tf::matrixEigenToMsg(obsVerticesAugmented[0],msgMatrix);    
    chatter_pub.publish(msgMatrix);

    ros::spinOnce();

    loop_rate.sleep();

    ite ++ ;
  }


    return 0;
}
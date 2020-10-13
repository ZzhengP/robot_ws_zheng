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
#include "Eigen/Core"
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <string>
using namespace  std;
using namespace Eigen;
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

    Eigen::Vector3d obsCentre;
    std::vector<Eigen::Vector3d> obsCentrePath;
    obsCentre << 1.2, 0.1, 0.1;

    std::ofstream myfile;
    myfile.open ("/home/zheng/Bureau/obsCentrePath.txt");
    double obsVel = 0.5;
    double dt = 0.05;
    if (myfile.is_open()){
        myfile <<obsCentre.transpose() << '\n';
        int count(0);
        for (int i(0) ; i < 4/dt ; i++)
        {
            obsCentre(0) = obsCentre(0);
            obsCentre(1) = obsCentre(1);
            obsCentre(2) = obsCentre(2);
            obsCentrePath.push_back(obsCentre);
            myfile <<obsCentre.transpose() << '\n';
            count ++ ;
        }

    while(obsCentre(0) > 0.8){
        obsCentre(0) = obsCentre(0) - obsVel*dt;
        obsCentre(1) = obsCentre(1);
        obsCentre(2) = obsCentre(2);

        obsCentrePath.push_back(obsCentre);
        myfile <<obsCentre.transpose() << '\n';
        count ++;
    }
    std::cout<<"time that human is starting leave :\n " << count *dt << std::endl;

    for (int i(0) ; i < 4/dt ; i++)
    {
        obsCentre(0) = obsCentre(0);
        obsCentre(1) = obsCentre(1);
        obsCentre(2) = obsCentre(2);
        obsCentrePath.push_back(obsCentre);
        myfile <<obsCentre.transpose() << '\n';
        count ++;
    }

    std::cout<<"time that human is starting leave :\n " << count *dt << std::endl;

    while(obsCentre(0) < 1.2){
        obsCentre(0) = obsCentre(0) + obsVel*dt;
        obsCentre(1) = obsCentre(1);
        obsCentre(2) = obsCentre(2);

        obsCentrePath.push_back(obsCentre);
        myfile <<obsCentre.transpose() << '\n';

    }

    for (int i(0) ; i < 20 ; i++)
    {
        obsCentre(0) = obsCentre(0);
        obsCentre(1) = obsCentre(1);
        obsCentre(2) = obsCentre(2);
        obsCentrePath.push_back(obsCentre);
        myfile <<obsCentre.transpose() << '\n';

    }
}else {
 std::cout <<" cant open file " << std::endl;
}
    myfile.close();

    return 0;
}

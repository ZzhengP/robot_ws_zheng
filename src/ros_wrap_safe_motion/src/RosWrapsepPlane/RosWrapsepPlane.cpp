#include "RosWrapsepPlane/RosWrapsepPlane.h"



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

using namespace plane;

std::pair<int,int> closestPoint(Eigen::MatrixXd robotVertices, Eigen::MatrixXd obsVertices){

      std::pair<int,int> indexClosestPts;

      double dmin = 20;
       for (int i = 0; i < robotVertices.cols(); ++i) {
           for (int j=0; j < obsVertices.cols(); ++j) {
                Eigen::Vector3d dist_temp;
                dist_temp = robotVertices.block(0,i,3,1) -obsVertices.block(0,j,3,1);
                if (dist_temp.norm() < dmin){
                     dmin = dist_temp.norm();
                     indexClosestPts.first = i;
                     indexClosestPts.second = j;
                }
           }

       }
       return indexClosestPts;
}

Planes::Planes(int N,int nbrRobobotPart,int nbrObs, std::vector<Eigen::MatrixXd> robotVertices, Eigen::MatrixXd obsVertices)
    :N_(N), nbrRobotPart_(nbrRobobotPart), nbrObs_(nbrObs){

    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << FRED("load separating planes module : \n") <<  std::endl;

    // Initialize data's size
    robotVertices_.N = N_;
    robotVertices_.nbrOfObject = nbrRobobotPart;
    robotVertices_.nbrVertices = robotVertices[0].cols();
    robotVertices_.Vertices.resize(robotVertices_.nbrOfObject);

    for (int i = 0; i < robotVertices_.nbrOfObject; ++i) {
        robotVertices_.Vertices[i].resize(3,N_*robotVertices_.nbrVertices);
        for (int j = 0 ; j < N_; j++){
            robotVertices_.Vertices[i].block(0,robotVertices_.nbrVertices*j,3,robotVertices_.nbrVertices ) = robotVertices[i];
        }
    }


    obsVertices_.N = N_;
    obsVertices_.nbrOfObject = nbrObs;
    obsVertices_.nbrVertices = obsVertices.cols();
    obsVertices_.Vertices.resize(obsVertices_.nbrOfObject);
    for (int i =0; i < obsVertices_.nbrOfObject; i++) {

        obsVertices_.Vertices[i].resize(3,N_*obsVertices_.nbrVertices);

        for (int j = 0 ; j < N_; j++){
            obsVertices_.Vertices[i].block(0,obsVertices_.nbrVertices*j,3,obsVertices_.nbrVertices ) = obsVertices.block(3*i,0,3,obsVertices_.nbrVertices);
        }
    }


        nbrPlane_ = robotVertices_.nbrOfObject*obsVertices_.nbrOfObject;
        planesData_.N = N_;
        planesData_.nbrPlane = nbrPlane_;

        planesData_.planeLocation.resize(nbrPlane_);

        for (int i = 0; i < nbrPlane_; i++) {

            if (N_ == 1){
                planesData_.planeLocation[i].resize(5,N_);
            }else {
                planesData_.planeLocation[i].resize(5,N_-1);

            }
            planesData_.planeLocation[i].setZero();
        }


        InitializePlanesData();


        std::cout <<"---------------------------------------------" << std::endl;
        std::cout << FYEL("separating planes module is initialized with \n") << nbrRobotPart_ << FYEL(" robot's part to take considered and ")
                  << nbrObs << FYEL(" number of obstacle") <<  std::endl;
//        std::cout <<FYEL("planes data are:  \n")<< planesData_.planeLocation[0] << std::endl;

//        std::cout <<FYEL("planes data are:  \n")<< planesData_.planeLocation[1] << std::endl;

}

void Planes::robotVerticesCallback(const visualization_msgs::MarkerArray::ConstPtr & vertices_markers){
    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << FRED("Reading data from robot Vertices message  : \n") <<  std::endl;

    // Initialize data's size
    robotVertices_.N = N_;
    robotVertices_.nbrOfObject =  vertices_markers->markers.size()/N_;
    robotVertices_.nbrVertices = 1;
    robotVertices_.Vertices.resize(robotVertices_.nbrOfObject);

    

    nbrRobotPart_ = robotVertices_.nbrOfObject;
    robotVerticesAugmented_.resize(robotVertices_.nbrOfObject);


    for (int j = 0; j < robotVertices_.nbrOfObject; ++j) {
            robotVertices_.Vertices[j].resize(3,robotVertices_.nbrVertices);
            robotVertices_.Vertices[j].block(0,j,3,1) << vertices_markers->markers[robotVertices_.nbrVertices*j].pose.position.x,
                                                         vertices_markers->markers[robotVertices_.nbrVertices*j].pose.position.y,
                                                         vertices_markers->markers[robotVertices_.nbrVertices*j].pose.position.z ;
        for (size_t i=0; i < N_; i++){
            robotVerticesAugmented_[j].block(0,i,3,1) << vertices_markers->markers[i].pose.position.x,
                                                         vertices_markers->markers[i].pose.position.y,
                                                         vertices_markers->markers[i].pose.position.z ;
        }   
    }
  
        
    std::cout << " robot vertices received :\n"<< robotVertices_.Vertices[0]  << std::endl;

  

}
    
void Planes::obsVerticesCallback(const std_msgs::Float64MultiArray::ConstPtr & obsVerticesMsg){

    obsVerticesAugmented_[0] = msgToEigen(obsVerticesMsg);

   std::cout << " obstacle augmented vertices received :\n"<<  obsVerticesAugmented_[0]  << std::endl;

}
void Planes::update(){
  
    int numberOfObstacle = 1;
    int obsNbrVertices = obsVertices_.nbrVertices;
    int nbrObsPart = obsVertices_.nbrOfObject;
    int nbrRobotPart = robotVertices_.nbrOfObject;
    bool is_plane_solved ;


for (int i(0);i<nbrObsPart; i++){
            for (int j(0);j<nbrRobotPart;j++){
                for (int k(0); k < N_-1; k++){


                gPlane_ << planesData_.planeLocation[i].block(0,k,4,1),0 ;
                plane_solver_ -> setCost(gPlane_);
                plane_solver_ -> setCstMatrix(robotVerticesAugmented_[j].block(0,k,3,2),obsVerticesAugmented_[0].block(0,k*obsNbrVertices,3,2*obsNbrVertices),
                                                planesData_.planeLocation[i].block(0,k,3,1));

                is_plane_solved = plane_solver_ -> solve();
            
                planesData_.planeLocation[i].block(0,k,5,1) = plane_solver_ -> getSolution();
                planesData_.planeLocation[i].block(0,k,3,1) = planesData_.planeLocation[i].block(0,k,3,1)/planesData_.planeLocation[i].block(0,k,3,1).norm();
                planesData_.planeLocation[i](3,k) = planesData_.planeLocation[i](3,k)*planesData_.planeLocation[i].block(0,k,3,1).norm();
                if (!is_plane_solved){
                    std::cout <<" plane solution failed :\n" << std::endl;
                    break;}
                }

                }
            }

    std_msgs::Float64MultiArray msgPlaneData; 

    tf::matrixEigenToMsg(planesData_.planeLocation[0],msgPlaneData);    

    plane_data_publisher_.publish(msgPlaneData);
    planesPub_->publishABCDPlane(planesData_.planeLocation[0](0,0),
                                 planesData_.planeLocation[0](1,0),
                                 planesData_.planeLocation[0](2,0),
                                 -planesData_.planeLocation[0](3,0),colorPlane_,plane_x_width_,plane_y_width_);
    planesPub_->trigger();

    for (int i(0); i <8;i++){
        obsVerticesPose_.poses[i].position.x = obsVerticesAugmented_[0](0,i);
        obsVerticesPose_.poses[i].position.y = obsVerticesAugmented_[0](1,i);
        obsVerticesPose_.poses[i].position.z = obsVerticesAugmented_[0](2,i);

    }

    for (int i(0); i <8;i++){
         cubeObstacleMarkers_->publishSphere(obsVerticesPose_.poses[i]);
    }
    Eigen::Vector3d point1, point2, point3, point4;
    point1 << obsVerticesAugmented_[0](0,0) , obsCenter_(1), obsCenter_(2) ;
    point2 << obsVerticesAugmented_[0](0,4) , obsCenter_(1), obsCenter_(2) ;
    point3 << obsVerticesAugmented_[0](0,4*8),obsCenter_(1),obsCenter_(2);
    point4 << obsVerticesAugmented_[0](0,4*8+4),obsCenter_(1),obsCenter_(2);

    cubeObstacleMarkers_->publishCylinder(point1,point2,colorPlane2,0.1);

    cubeObstacleMarkers_ -> trigger();
    planesPub_ ->deleteAllMarkers();
    cubeObstacleMarkers_->deleteAllMarkers();
}

Planes::Planes(ros::NodeHandle* nodehandle, int N, std::vector<Eigen::MatrixXd> obsVerticesAugmented,double dsafe)
        :nh_(*nodehandle),N_(N), dsafe_(dsafe){
    

    // ----------------------------------  Initialize robot's vertices at position t = 0 ----------------------------------
     ROS_INFO( " Initialize robot's vertices at position t = 0");

    robotVerticesAugmented_.resize(1);
    robotVertices_.N = N_;
    robotVertices_.nbrOfObject = 1;
    robotVertices_.nbrVertices = 1;
    robotVertices_.Vertices.resize(robotVertices_.nbrOfObject);
 
    nbrRobotPart_ = robotVertices_.nbrOfObject;
    robotVerticesAugmented_.resize(robotVertices_.nbrOfObject);
    robotVerticesAugmented_[0].resize(3,N_);

    for (int j = 0; j < robotVertices_.nbrOfObject; ++j) {
            robotVertices_.Vertices[j].resize(3,robotVertices_.nbrVertices);
            robotVertices_.Vertices[j].block(0,j,3,1) << 0.5,
                                                         0.3,
                                                         0.1 ;
        for (size_t i=0; i < N_; i++){
            robotVerticesAugmented_[j].block(0,i,3,1) << 0.5,
                                                         0.3,
                                                         0.1 ;
        }   
    }
  
    obsCenter_ << 1.2 , 0.1, 0.1;
    obsVerticesAugmented_ = obsVerticesAugmented;
    obsVertices_.N = N_;
    nbrObs_ = obsVerticesAugmented.size();
    obsVertices_.nbrOfObject = nbrObs_;
    obsVertices_.nbrVertices = obsVerticesAugmented[0].cols() / N_;
    obsVertices_.Vertices.resize(obsVertices_.nbrOfObject);
    for (int i =0; i < obsVertices_.nbrOfObject; i++) {

        obsVertices_.Vertices[i].resize(3,N_*obsVertices_.nbrVertices);
        obsVertices_.Vertices[i].block(0,0,3,obsVertices_.nbrVertices ) = obsVerticesAugmented[i].block(0,0,3,obsVertices_.nbrVertices);
        
    }

    nbrPlane_ = 1*obsVertices_.nbrOfObject;
    planesData_.N = N_;
    planesData_.nbrPlane = nbrPlane_;

    planesData_.planeLocation.resize(nbrPlane_);

    for (int i = 0; i < nbrPlane_; i++) {

        if (N_ == 1){
             planesData_.planeLocation[i].resize(5,N_);
        }else {
            planesData_.planeLocation[i].resize(5,N_-1);

        }
            for (int j(0); j < N_-1 ; j++){
                planesData_.planeLocation[i].block(0,j,5,1)<< 1,0,0,0.8,0;
            }
    }

    planesPub_.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/seperatingPlane"));
    cubeObstacleMarkers_.reset(new rviz_visual_tools::RvizVisualTools("panda_link0","/simpleCube"));


    obsVerticesPose_.poses.resize(8);
    std::cout << "obsVerticesAugmented : " << obsVerticesAugmented_[0] << std::endl;
    for (int i(0); i <8;i++){
        obsVerticesPose_.poses[i].position.x = obsVerticesAugmented_[0](0,i);
        obsVerticesPose_.poses[i].position.y = obsVerticesAugmented_[0](1,i);
        obsVerticesPose_.poses[i].position.z = obsVerticesAugmented_[0](2,i);

    }

    
    // ------------------------ Initialize Plane solver parameters ----------------------------------------------------------
    ROS_INFO( "Initialize Plane solver parameters");
    plane_solver_ = std::make_shared<PlaneSolver>(10,dsafe_);
    gPlane_.resize(5);
    gPlane_.setZero();

    plane_data_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>("plane_data",1000);
    // ------------------------ finish  Plane solver parameters ---------------------------------------------------------- 
    robot_vertices_subscriber_ = nh_.subscribe("/robotVerticesPublisher/panda/pandaVertices", 10, &Planes::robotVerticesCallback,this);  
    obs_vertices_subscriber_ = nh_.subscribe("obstacle_vertices",10,&Planes::obsVerticesCallback, this);

    // std::cout <<"---------------------------------------------" << std::endl;
    // std::cout << FYEL("separating planes module is initialized with \n") << nbrRobotPart_ << FYEL(" robot's part to take considered and ")
    //               << nbrObs << FYEL(" number of obstacle") <<  std::endl;
//        std::cout <<FYEL("planes data are:  \n")<< planesData_.planeLocation[0] << std::endl;

//        std::cout <<FYEL("planes data are:  \n")<< planesData_.planeLocation[1] << std::endl;
           
}

void Planes::InitializePlanesData(){
    Eigen::Vector3d ptsOnPlane, verticeRobotClosest, verticeObsclosest, vect_normal ;
    Eigen::MatrixXd verticeRobotTemp, verticeObsTemp;
    double dist;
    int count = 0;
    if (nbrObs_ == 0){
       for (int k(0); k < nbrPlane_ ; k ++){
        for (int j(0); j < N_ ; j++)
        {
            planesData_.planeLocation[k].block(5*k,j,5,1) << 1., 0., 0., 1.5,0;
        }
    }
    }else {

        for (int i(0); i < robotVertices_.nbrOfObject; i ++ ) {
            for (int l(0) ; l < obsVertices_.nbrOfObject; l++){
                if (N_ ==1){

                    verticeRobotTemp = robotVertices_.Vertices[i].block(0,robotVertices_.nbrVertices*0,3,robotVertices_.nbrVertices);
                    verticeObsTemp = obsVertices_.Vertices[l].block(0,obsVertices_.nbrVertices*0,3,obsVertices_.nbrVertices );

                    std::pair<int,int> indexClosestPts ;
                    indexClosestPts = closestPoint(verticeRobotTemp,verticeObsTemp);
                    verticeRobotClosest = robotVertices_.Vertices[i].block(0,indexClosestPts.first,3,1);
                    verticeObsclosest = obsVertices_.Vertices[l].block(0,indexClosestPts.second,3,1);
                    vect_normal = verticeObsclosest - verticeRobotClosest;
                    vect_normal.normalize();

                    Eigen::Vector3d AB;
                    AB = verticeObsclosest - verticeRobotClosest;
                    ptsOnPlane = verticeRobotClosest + AB.norm()*vect_normal/2  ;

                    dist = ptsOnPlane.dot(vect_normal);

                    planesData_.planeLocation[count].block(0,0,5,1) <<  vect_normal.x(), vect_normal.y(), vect_normal.z(), dist,0;

                }else {
                for (int j(0); j < N_-1; j++) {
                    verticeRobotTemp = robotVertices_.Vertices[i].block(0,robotVertices_.nbrVertices*j,3,robotVertices_.nbrVertices);
                    verticeObsTemp = obsVertices_.Vertices[l].block(0,obsVertices_.nbrVertices*j,3,obsVertices_.nbrVertices );

                    std::pair<int,int> indexClosestPts ;
                    indexClosestPts = closestPoint(verticeRobotTemp,verticeObsTemp);
                    verticeRobotClosest = robotVertices_.Vertices[i].block(0,indexClosestPts.first,3,1);
                    verticeObsclosest = obsVertices_.Vertices[l].block(0,indexClosestPts.second,3,1);
                    vect_normal = verticeObsclosest - verticeRobotClosest;
                    vect_normal.normalize();

                    Eigen::Vector3d AB;
                    AB = verticeObsclosest - verticeRobotClosest;
                    ptsOnPlane = verticeRobotClosest + AB.norm()*vect_normal/2  ;

                    dist = ptsOnPlane.dot(vect_normal);

                    planesData_.planeLocation[count].block(0,j,5,1) <<  vect_normal.x(), vect_normal.y(), vect_normal.z(), dist,0;

                }
                count ++ ;
                }
            }
        }

    }
    std::cout << "----------- debugging----------" << std::endl;
    std::cout << "vertices robot : \n" << robotVertices_.Vertices[0] << std::endl;
    std::cout << "vertices obstacle : \n" << obsVertices_.Vertices[0] << std::endl;
    std::cout << "number  obstacle : \n" << obsVertices_.Vertices[0] << std::endl;

    std::cout << "plane data 1 :\n " << planesData_.planeLocation[0] << std::endl;
//    std::cout << "plane data 2 :\n " << planesData_.planeLocation[1] << std::endl;

}


void Planes::run(){

    ROS_INFO( "Start running");

    ros::Rate r(100);// this node will work at 100hz  
    while (ros::ok()){

        update();
        ros::spinOnce();
        r.sleep();
    }
    
}




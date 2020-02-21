#include "sepPlane/sepPlane.h"

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
    std::cout <<" robotVertices nbr vertices \n" << robotVertices_.nbrVertices << std::endl;

    for (int i = 0; i < robotVertices_.nbrOfObject; ++i) {
        robotVertices_.Vertices[i].resize(3,N_*robotVertices_.nbrVertices);
        for (int j = 0 ; j < N_; j++){
            robotVertices_.Vertices[i].block(0,robotVertices_.nbrVertices*j,3,robotVertices_.nbrVertices ) = robotVertices[i];
        }
    }


    obsVertices_.N = N_;
    obsVertices_.nbrOfObject = nbrObs;
    obsVertices_.nbrVertices = 1;
    obsVertices_.Vertices.resize(obsVertices_.nbrOfObject);

    for (int i =0; i < obsVertices_.nbrOfObject; i++) {

        obsVertices_.Vertices[i].resize(3,N_*obsVertices_.nbrVertices);
        for (int j = 0 ; j < N_; j++){
            obsVertices_.Vertices[i].block(3*i,obsVertices_.nbrOfObject*j,3,1 ) = obsVertices;
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
        std::cout <<" nbr planes \n"<< planesData_.nbrPlane << std::endl;

        InitializePlanesData();



        std::cout <<"---------------------------------------------" << std::endl;
        std::cout << FYEL("separating planes module is initialized with \n") << nbrRobotPart_ << FYEL(" robot's part to take considered and ")
                  << nbrObs << FYEL(" number of obstacle") <<  std::endl;
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







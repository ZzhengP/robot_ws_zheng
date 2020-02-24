#include "constraint/RosAvoidanceCst.h"



ObsAvoidanceCSt::ObsAvoidanceCSt(int ndof, int N, double dt, double dsafe, std::string name,Eigen::MatrixXd Px, Eigen::MatrixXd Pu)
    :ndof_(ndof),N_(N), dt_(dt),cst_name_(name),Px_(Px),Pu_(Pu),dsafe_(dsafe){

             lbA_.resize(2*(N_-1));
             ubA_.resize(2*(N_-1));
             A_.resize(2*(N_-1),ndof*N_);
             nbrRobotPart_ = 1;
             nbrObstacle_ = 0;
             cstData_.name_ = cst_name_;
             cstData_.lowBound_.resize(2*(N_-1));
             cstData_.lowBound_.setZero();

             cstData_.upBound_.resize(2*(N_-1));
             cstData_.upBound_.setZero();

             cstData_.cstMatrix_.resize(2*(N_-1),ndof*N_);
             cstData_.cstMatrix_.setZero();

}


ObsAvoidanceCSt::~ObsAvoidanceCSt()
{

}

void ObsAvoidanceCSt::setLowerBound(){
    lbA_.setConstant(-100000);
    cstData_.lowBound_ = lbA_;
}

void ObsAvoidanceCSt::setUpperBoundAndConstraint(const std::vector<Eigen::MatrixXd> &RobotVertices,
                                    const Planes::PlaneData &PlaneData, const Eigen::VectorXd &robotState,
                                    const Eigen::MatrixXd &JacobianHorizon,const Eigen::VectorXd &qHorizonPrecedent)
{
     nbrRobotPart_ = RobotVertices.size();
    // 2*2 because 2 vertices and 2 instants
    lbA_.resize(2*PlaneData.nbrPlane*(N_ - 1));
    lbA_.setConstant(-100000);
    ubA_.resize(2*PlaneData.nbrPlane*(N_ - 1));
    ubA_.setZero();
    A_.resize(2*PlaneData.nbrPlane*(N_ - 1),7*N_);
    Eigen::VectorXd bLarge;
    bLarge.resize(2*(N_ - 1));
    bLarge.setZero();
    Eigen::MatrixXd nLarge;
    nLarge.resize(2*(N_-1), 6*(N_-1));
    nLarge.setZero();

    Eigen::MatrixXd JacobianEnlarge, MatrixZeros;
    JacobianEnlarge.resize(6*(N_-1), 2*7*(N_-1));
    JacobianEnlarge.setZero();
    MatrixZeros.resize(3,7);
    MatrixZeros.setZero();
    Eigen::MatrixXd PxPBP, PuPBP;
    Eigen::VectorXd VerticesPrecedent, qLargePrecedent;

    for (int nbrOfPlane(0); nbrOfPlane < PlaneData.nbrPlane ; nbrOfPlane ++ ){
    for (int k(0); k < N_ -1 ; k ++  ){
        bLarge.segment(2*k,2) << PlaneData.planeLocation[nbrOfPlane](3,k)- dsafe_, PlaneData.planeLocation[nbrOfPlane](3,k) - dsafe_;
    }

    for (int k(0); k< N_-1 ; k++){
        nLarge.block(2*k,6*k,2,6) << PlaneData.planeLocation[nbrOfPlane].block(0,k,3,1).transpose(), 0,0,0,
                                     0,0,0,PlaneData.planeLocation[nbrOfPlane].block(0,k,3,1).transpose();
    }

    for (int k(0); k<N_-1; k++){
        JacobianEnlarge.block(6*k,14*k,6,14) << JacobianHorizon.block(6*k,7*k,3,7),MatrixZeros,
                                                 MatrixZeros,JacobianHorizon.block(6*k,7*k,3,7);
    }

    PxPBP.resize(14*(N_-1), 14);
    PuPBP.resize(14*(N_-1),7*N_);

    for (int k(0); k<N_-1;k++){
        PxPBP.block(14*k,0,14,14) = Px_.block(7*k,0,14,14);
        PuPBP.block(14*k,0,14,7*N_) = Pu_.block(7*k,0,14,7*N_);
    }

    VerticesPrecedent.resize(3*2*(N_-1));
    qLargePrecedent.resize(14*(N_-1));

    for (int k(0); k < N_-1; k++){
        VerticesPrecedent.segment(6*k,6) << RobotVertices[0].block(0,2*k,3,1), RobotVertices[0].block(0,2*(k+1),3,1);
        qLargePrecedent.segment(14*k,14) << qHorizonPrecedent.segment(7*k,7) , qHorizonPrecedent.segment(7*(k+1),7) ;
    }
    A_.block(2*(N_-1)*nbrOfPlane,0,2*(N_-1),7*N_) = nLarge*JacobianEnlarge*PuPBP;

//    ubA_ = bLarge - nLarge*(VerticesPrecedent + JacobianEnlarge*(PxPBP*robotState - qHorizonPrecedent));
    ubA_.segment(2*(N_-1)*nbrOfPlane,2*(N_-1)) = bLarge- nLarge*(VerticesPrecedent + JacobianEnlarge*(PxPBP*robotState - qLargePrecedent));
    }
    std::ofstream myfile_;

    myfile_.open ("/home/zheng/Bureau/obsAvoidance.txt");
    myfile_ << "bLarge :\n " << bLarge <<'\n';
    myfile_ << "nLarge :\n " << nLarge <<'\n';
    myfile_ << "JacobianLarge :\n" << JacobianEnlarge <<'\n';
    myfile_ << "PxPBP :\n" << PxPBP <<'\n';
    myfile_ << "PuPBP :\n" << PuPBP <<'\n';
    myfile_ << "A_ :\n" << A_ <<'\n';
    myfile_ << "ubA_ :\n" << ubA_ <<'\n';
    myfile_ << "lbA_ :\n" << lbA_ <<'\n';
    myfile_ << "bLarge :\n " << bLarge <<'\n';
    myfile_ <<" robot state :\n" << robotState << '\n';
    myfile_ <<" VerticesPrecedent :\n" << VerticesPrecedent << '\n';
    myfile_ << "qLargePrecedent :\n" << qLargePrecedent<<'\n';

    myfile_ << "nLarge*(VerticesPrecedent) :\n" << nLarge*(VerticesPrecedent) <<'\n';
    myfile_ << "nLarge*(JacobianEnlarge*(PxPBP*robotState ) \n" << nLarge*(JacobianEnlarge*(PxPBP*robotState )) <<'\n';
    myfile_ << "nLarge*(JacobianEnlarge*(- qLargePrecedent) ) \n" <<nLarge*(JacobianEnlarge*(- qLargePrecedent) ) <<'\n';
//    std::cout <<"ubA_ constraint avoidance :\n" << bLarge- nLarge*VerticesPrecedent - nLarge*JacobianEnlarge*PxPBP*robotState + nLarge*JacobianEnlarge*qLargePrecedent <<  "\n" ;
    myfile_.close();

    std::cout <<cst_name_ << " ubA :\n " << ubA_.size() << '\n' ;
    cstData_.upBound_.resize(ubA_.size());

    cstData_.upBound_.setZero(1000);
    cstData_.lowBound_.resize(lbA_.size());
    cstData_.cstMatrix_.resize(2*PlaneData.nbrPlane*(N_ - 1),7*N_);

    cstData_.upBound_= ubA_;
    cstData_.lowBound_.setConstant(-1000000);
    cstData_.cstMatrix_ = A_;
}



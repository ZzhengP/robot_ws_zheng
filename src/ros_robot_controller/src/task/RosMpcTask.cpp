#include "task/RosMpcTask.h"

MPC_Task::MPC_Task(int N, double Q, double R, int ndof, double dt, std::string robot_name):N_(N), ndof(ndof), dt_(dt),weightTask1_(Q),weightTask2_(R){

    Id_.resize(ndof,ndof);
    Id_.setIdentity();

    std::cout <<"---------------------------------------------" << std::endl;
    std::cout <<BOLD(FRED("add tracking task for "))<< robot_name << std::endl;
    std::cout <<"---------------------------------------------" << std::endl;

}

bool MPC_Task::init()
{
    // robot dynamical model
    A_.resize(2*ndof,2*ndof);
    A_.setZero();
    A_.block(0,0,ndof,ndof).setIdentity();
    A_.block(0,ndof,ndof,ndof) = Id_*dt_;
    A_.block(ndof, ndof, ndof, ndof).setIdentity();

    B_.resize(2*ndof,ndof);
    B_.setZero();
    B_.block(0,0,ndof,ndof)= Id_*(dt_*dt_/2);
    B_.block(ndof,0,ndof,ndof)= Id_*dt_;

    C_q_.resize(ndof,2*ndof);
    C_q_.setZero();
    C_q_.block(0,0,ndof,ndof).setIdentity();

    C_dq_.resize(ndof,2*ndof);
    C_dq_.setZero();
    C_dq_.block(0,ndof,ndof,ndof).setIdentity();


    // Augmented state for MPC
    Px_.resize(ndof*N_,2*ndof);
    Px_.setZero();
    computePx();

    Pu_.resize(ndof*N_,ndof*N_);
    Pu_.setZero();
    computePu();

    Px_dq_.resize(ndof*N_,2*ndof);
    Px_dq_.setZero();
    computePxDq();
    Pu_dq_.resize(ndof*N_,ndof*N_);
    Pu_dq_.setZero();
    computePuDq();
    // quadratic form for MPC

    // size of hessien depend to the size of input variable (NofD * N_prediction)
    Hessien_.resize(N_*ndof,N_*ndof);
    Hessien_.setZero();
    gradient_.resize(N_*ndof);
    gradient_.setZero();
    E_.resize(N_*ndof,N_*ndof);
    E_.setZero();
    f_.resize(N_*ndof);
    f_.setZero();
    E_dq_.resize(N_*ndof,N_*ndof);
    E_dq_.setZero();
    f_dq_.resize(N_*ndof);
    f_dq_.setZero();


    Acc_reg_.resize(N_*ndof,N_*ndof);
    Acc_reg_.setIdentity();
    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << FYEL("The task is successively initialized: ") << std::endl;
    std::cout << FYEL("With position tracking weight : ") << weightTask1_ <<'\n'
              << FYEL("With velocity tracking weight : ") << weightTask2_ <<'\n'
              << FYEL("With acceleration tracking weight :") << weightTask2_ << std::endl;

    std::cout << FYEL("State matrix A is: \n") << A_.format(CleanFmt)  << "\n"
              << FYEL("Input matrix B is: \n") << B_.format(CleanFmt) << "\n"
//              << "Joint position output matrix C_q: \n"<< C_q_.format(CleanFmt)<< "\n"
//              << "Joint velocity output matrix C_dq: \n"<< C_dq_.format(CleanFmt) << "\n"
//              << "Augmented state matrix Px is : \n" << Px_.format(CleanFmt)<< "\n"
//              << "Augmented input matrix Pu is : \n" << Pu_.format(CleanFmt)<< std::endl;
                <<"---------------------------------------------" << std::endl;
    return true;

}

void MPC_Task::computePx()
{
    for (int i(0); i<N_;i++)
    {
        Eigen::MatrixXd A;
        A = matPow(i+1,A_);
        Px_.block(ndof*i,0,ndof,2*ndof) = C_q_*A;
    }

}

void MPC_Task::computePu()
{
    for (int i(0); i<N_; i ++)
    {
        for (int j(0); j<i+1;j++)
        {
            Eigen::MatrixXd A;
            A = matPow(i-j,A_);
            Pu_.block(ndof*i,ndof*j,ndof,ndof) = C_q_*A*B_;
        }
    }
}

void MPC_Task::computePxDq()
{
    for (int i(0); i<N_;i++)
    {
        Eigen::MatrixXd A;
        A = matPow(i+1,A_);
        Px_dq_.block(ndof*i,0,ndof,2*ndof) = C_dq_*A;
    }
}

void MPC_Task::computePuDq()
{

    for (int i(0); i<N_; i ++)
    {
        for (int j(0); j<i+1;j++)
        {
            Eigen::MatrixXd A;
            A = matPow(i-j,A_);
            Pu_dq_.block(ndof*i,ndof*j,ndof,ndof) = C_dq_*A*B_;
        }
    }
}

void MPC_Task::computeHandg(Eigen::MatrixXd J_horizon, Eigen::VectorXd robot_state, Eigen::VectorXd q_horz_Des)

{
//        Hessien_ = weightTask1_*E_.transpose()*E_ + weightTask2_*E_dq_.transpose()*E_dq_;
     Eigen::MatrixXd Id;
     Id.resize(Hessien_.rows(),Hessien_.cols());
     Id.setIdentity();
     computeE();
     computef(robot_state, q_horz_Des);
    // computeEdq(J_horizon);
    // computefdq(robot_state,J_horizon);
     Hessien_ = E_.transpose()*E_ + 0.0001*E_dq_.transpose()*E_dq_ + 0.0001*Id.transpose()*Id;
     gradient_ = E_.transpose()*f_+ 0.0001*E_dq_.transpose()*f_dq_ ;
//     Hessien_ = E_.transpose()*E_ + 0.0001*Id.transpose()*Id;
//     gradient_ = E_.transpose()*f_ ;
}

void MPC_Task::computeHandg(Eigen::MatrixXd J_horizon, Eigen::VectorXd q_horizon, Eigen::VectorXd cartPos_horizon, Eigen::VectorXd robot_state, Eigen::VectorXd PosDes)
{
//        Hessien_ = weightTask1_*E_.transpose()*E_ + weightTask2_*E_dq_.transpose()*E_dq_;

     computeE(J_horizon);
     computef(J_horizon,cartPos_horizon,q_horizon,robot_state, PosDes);
     computeEdq(J_horizon);
     computefdq(robot_state,J_horizon);
     Eigen::MatrixXd Id;

     Id.resize(Hessien_.rows(),Hessien_.cols());
     Id.setIdentity();
     Hessien_ = E_.transpose()*E_ + 0.0001*E_dq_.transpose()*E_dq_ + 0.0001*Id.transpose()*Id ;
     gradient_ = E_.transpose()*f_+ 0.0001*E_dq_.transpose()*f_dq_ ;
}


void MPC_Task::computeE(Eigen::MatrixXd J_horizon )
{
    E_ = J_horizon*Pu_;

}

void MPC_Task::computeE()
{
    E_ = Pu_;

}


void MPC_Task::computef(Eigen::MatrixXd J_horizon, Eigen::VectorXd cartPos_horizon, Eigen::VectorXd q_horizon, Eigen::VectorXd robot_state, Eigen::VectorXd PosDes)
{
    f_ = cartPos_horizon + J_horizon*Px_*robot_state - J_horizon*q_horizon - PosDes;
}

void MPC_Task::computef(const Eigen::VectorXd& robot_state, const Eigen::VectorXd& q_horz_Des)
{
    f_ = Px_*robot_state - q_horz_Des;
}


void MPC_Task::computeEdq(Eigen::MatrixXd J_horizon)
{
    E_dq_ = J_horizon*Pu_dq_;
}

void MPC_Task::computefdq(Eigen::VectorXd robot_state, Eigen::MatrixXd J_horizon)
{
   f_dq_ = J_horizon*Px_dq_*robot_state ;
}




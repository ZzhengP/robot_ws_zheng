#include "Eigen/Core"
#include "iostream"
#include "unsupported/Eigen/MatrixFunctions"
#include "Eigen/Dense"
Eigen::MatrixXd matPow(int N, const Eigen::MatrixXd&  A)
{
    Eigen::MatrixXd A_pow;
    A_pow.resize(A.rows(),A.cols());
    A_pow.setIdentity();
    for (int i(0); i<N;i++)
    {
        A_pow = A_pow*A;
    }
    return A_pow;
}

class  MPC_Task
{
public:
    MPC_Task(int N, double Q, double R, int ndof, double dt);


    bool init();


    void computePx();
    void computePu();

    void computePxDq();
    void computePuDq();


    void computeHandg(Eigen::MatrixXd J_horizon,Eigen::VectorXd robot_state,Eigen::VectorXd q_horz_Des);
    void computeHandg(Eigen::MatrixXd J_horizon, Eigen::VectorXd q_horizon,Eigen::VectorXd cartPos_horizon, Eigen::VectorXd robot_state,Eigen::VectorXd PosDes);

    void computeE(Eigen::MatrixXd J_horizon );
    void computeE();

    void computef(Eigen::MatrixXd J_horizon,Eigen::VectorXd cartPos_horizon,Eigen::VectorXd q_horizon, Eigen::VectorXd robot_state, Eigen::VectorXd PosDes);

    void computef(const Eigen::VectorXd& robot_state, const Eigen::VectorXd& q_horz_Des);

    void computeEdq(Eigen::MatrixXd J_horizon);
    void computefdq(Eigen::VectorXd robot_state,Eigen::MatrixXd J_horizon);


    /**
     * @brief get all matrix needed
     * @return
     */

    Eigen::MatrixXd getStateA()
    {
        return A_;
    }
    Eigen::MatrixXd getStateB()
    {
        return B_;
    }
    Eigen::MatrixXd getMatrixH()
    {
        return Hessien_;
    }
    Eigen::VectorXd getVectorg()
    {
        return gradient_;
    }
    Eigen::MatrixXd getMatrixE()
    {
        return E_;
    }
    Eigen::MatrixXd getVectorf()
    {
        return f_;
    }
    Eigen::MatrixXd getMatrixdE()
    {
        return E_dq_;
    }
    Eigen::MatrixXd getVectordf()
    {
        return f_dq_;
    }
    Eigen::MatrixXd getMatrixPx()
    {
        return Px_;
    }
    Eigen::MatrixXd getMatrixPu()
    {
        return Pu_;
    }
    Eigen::MatrixXd getMatrixPxdq()
    {
        return Px_dq_;
    }
    Eigen::MatrixXd getMatrixPudq()
    {
        return Pu_dq_;
    }
private:
    const int N_, ndof;
    double weightTask1_, weightTask2_;
    const double dt_;
    /**
     * @brief Matrix and and vector for quadratic optimal problem
     */
    Eigen::MatrixXd Hessien_, E_, E_dq_, Acc_reg_, Id_;

    Eigen::VectorXd gradient_, f_, f_dq_;

    /**
     * @brief The dynamic model for the robot
     */
    Eigen::MatrixXd A_, B_,C_q_, C_dq_;

    /**
     * @brief Matrix enlarged from the MPC problem
     */

    Eigen::MatrixXd Px_, Pu_, Px_dq_, Pu_dq_;

};

#include "Eigen/Core"
#include "iostream"
#include "unsupported/Eigen/MatrixFunctions"
#include "Eigen/Dense"


class  jnt_vel_cst{
public:
        jnt_vel_cst(int N_pre, int Ndof, Eigen::VectorXd dotq_l, Eigen::VectorXd dotq_u, Eigen::MatrixXd Px_dq, Eigen::MatrixXd Pu_dq)
        {
         robotNrofDegre_ = Ndof;
         N_Prediction_ = N_pre;

         dotq_lower_.resize(robotNrofDegre_*N_Prediction_);
         dotq_upper_.resize(robotNrofDegre_*N_Prediction_);

         dotq_lower_ = dotq_l;
         dotq_upper_ = dotq_u;

         Px_dq_.resize(robotNrofDegre_*N_Prediction_,2*robotNrofDegre_);
         Pu_dq_.resize(robotNrofDegre_*N_Prediction_,robotNrofDegre_*N_Prediction_);

         Px_dq_ = Px_dq;
         Pu_dq_ = Pu_dq;
        }

        Eigen::MatrixXd compute_C_dotq(){
               return Pu_dq_;
        }

        Eigen::VectorXd compute_low_dq(Eigen::VectorXd X_hat){
               return dotq_lower_ - Px_dq_*X_hat ;
        }

        Eigen::VectorXd compute_upper_dq(Eigen::VectorXd X_hat){
               return dotq_upper_ - Px_dq_*X_hat ;
        }
private:
        Eigen::VectorXd dotq_lower_, dotq_upper_;
        Eigen::MatrixXd Px_dq_, Pu_dq_;
        int robotNrofDegre_, N_Prediction_;

};

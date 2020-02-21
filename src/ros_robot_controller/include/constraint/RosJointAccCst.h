#include "constraint/RosGenericCst.h"

/**
 * @brief Acceleration constraint
 */
class  jntAccCst : public generic_cst
{

public:

    jntAccCst(int ndof, int N,double dt, std::string name, Eigen::MatrixXd Px, Eigen::MatrixXd Pu)
        : generic_cst(ndof, N,dt, name, Px, Pu) {

    }

     ~jntAccCst(){}


    void setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){

         min_ = min ;
         max_ = max;
    }


protected:


};

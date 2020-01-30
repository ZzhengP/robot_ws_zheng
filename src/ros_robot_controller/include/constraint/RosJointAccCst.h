#include "constraint/RosGenericCst.h"

/**
 * @brief Acceleration constraint
 */
class  jntAccCst : public generic_cst
{

public:

    jntAccCst(int ndof, int N,double dt, std::string name) : generic_cst(ndof, N,dt, name) {

    }

     ~jntAccCst(){}


    void setLimit(const Eigen::VectorXd &min, const Eigen::VectorXd &max){


         min_ = min ;
         max_ = max;
    }


protected:


};

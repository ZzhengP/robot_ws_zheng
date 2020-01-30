#include "constraint/RosGenericCst.h"


generic_cst::generic_cst(int ndof, int N, double dt, std::string name):n_(ndof),N_(N), dt_(dt),cst_name_(name){

             lbA_.resize(ndof*N);
             ubA_.resize(ndof*N);
             A_.resize(ndof*N,ndof*N);
             min_.resize(ndof*N);
             max_.resize(ndof*N);


}

generic_cst::~generic_cst()
{

}



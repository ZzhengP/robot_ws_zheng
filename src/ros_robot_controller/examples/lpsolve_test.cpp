#include <iostream>
extern "C"{
#include "glpk.h"
}
#include "solver/lpsolve.h"
#include <boost/shared_ptr.hpp>
#include "memory"
#include "Eigen/Core"
#include "Eigen/Dense"

int main()
{
    int Nv = 5, Nc = 3;
    std::shared_ptr<lpSolver> lp = std::make_shared<lpSolver>(Nv,Nc);
    lp->setCost();
    lp->setRowConstraint();

    Eigen::MatrixXd Atest_;
    Atest_.resize(3,5);
    Atest_<< 1,2,3,1,1,
             4,5,6,1,1,
             7,8,9,1,1;
    lp->setEqualityConstraintMatrix(Atest_);
    lp->loadEqualityMatrix();
    lp->print();
    lp->solve();

    Eigen::VectorXd solution;
    solution.resize(5);
    solution = lp->getSolution();
    std::cout << glp_version() << std::endl;
    return 0;
}

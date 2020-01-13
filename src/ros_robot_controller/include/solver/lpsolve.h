#ifndef LPSOLVE_H
#define LPSOLVE_H

#endif // LPSOLVE_H

#include "glpk.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "commun/color.h"
#include "map"
#include <cstring>


// This is a C++ Eigen-Based wrapp for solving linear programming problem
// In lp_solve file the problem is formulated is this way:
//        Max xg'
//        s.t
//          Ax<= B
//          x >= lb
class lpSolver{

public:
    lpSolver(int Nv, int Nc);

    void setCost();

    void print();

    // Row constraint is related to the equality constraint wich correspond to auxiliary variable
    void setRowConstraint();

    void setConstraintMatrix(const Eigen::MatrixXd & robotPartielVertices,
                             const Eigen::MatrixXd & obsPartielVertices, const Eigen::MatrixXd & dataPlanePrecedent);

    void setEqualityConstraintMatrix();

    void setEqualityConstraintMatrix(Eigen::MatrixXd A);

    void loadEqualityMatrix(){

        glp_load_matrix(lp_,l_*col_,ia_,ja_,ar_);

    }

    void solve(){
        glp_simplex(lp_,NULL);

        for (int i(0); i<Nv_; i++){
            Solution_(i)= glp_get_col_prim(lp_,i+1);
        }
    }

    Eigen::VectorXd getSolution(){
            return Solution_;
    }
private:
    glp_prob *lp_;
    int Nc_, Nv_ , *colno_ = NULL;
    int ret_;
    std::map<std::string, int> structureVar_;
    std::map<std::string, int> auxiliaryVar_;
    std::map<int,int> structureVarBnds_;
    Eigen::MatrixXd A_;
    int ia_[31], ja_[31];
    double ar_[31];
    int l_,col_;
    Eigen::VectorXd Solution_;
};
